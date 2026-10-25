"""
Mini-Aplicación: Enseñar y Reproducir
Arquitectura multihilo: Red (TCP), Gráficos (PyBullet) y CLI (Consola).
"""

import socket
import struct
import time
import math
import sys
import os
import json
import threading
import pybullet
from UR3e_rg2.dynamic_ur3e import UR3eRG2

PORT = 30001
MAX_VEL_RAD_S = 0.3    # Constante de seguridad
MAX_ACC_RAD_S2 = 0.5   # Constante de seguridad
CONFIG_FILE = "robots.json"

def cargar_ip():
    """Lee dinámicamente la IP del primer robot como única fuente de verdad."""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                robots_cfg = json.load(f)
                if isinstance(robots_cfg, list) and len(robots_cfg) > 0:
                    ip_leida = robots_cfg[0].get("ip")
                    if ip_leida:
                        return ip_leida
        except Exception as e:
            print(f"[AVISO] Error al analizar {CONFIG_FILE}: {e}")
            
    ip_defecto = "10.172.21.203"
    print(f"[AVISO] No se pudo leer {CONFIG_FILE}. Usando IP de contingencia: {ip_defecto}")
    return ip_defecto

class UR3eController:
    """
    Clase gestora de la comunicación bidireccional y telemetría de seguridad.
    """
    def __init__(self, ip, puerto):
        self.ip = ip
        self.puerto = puerto
        self.sock = None
        self.lock = threading.Lock()
        
        self.q_actual = [0.0]*6
        self.is_power_on = False
        self.is_program_running = False
        self.is_emergency_stopped = False
        self.is_protective_stopped = False
        self.ultimo_error = None
        self.conectado = False
        self.activo = True

    def iniciar(self):
        self.hilo = threading.Thread(target=self._bucle_red, daemon=True)
        self.hilo.start()

    def enviar_script(self, comando: str):
        """
        Transmite URScript ASCII mediante una conexión TCP efímera (One-Shot).
        Forzar una nueva conexión por comando resetea la sesión del compilador 
        del robot, eliminando la excepción Subtipo 6.
        """
        try:
            if not comando.endswith("\n"):
                comando += "\n"
            # Socket temporal dedicado exclusivamente al envío
            sock_envio = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock_envio.settimeout(2.0)
            sock_envio.connect((self.ip, self.puerto))
            sock_envio.sendall(comando.encode("utf-8"))
            sock_envio.close()
            return True
        except OSError as e:
            print(f"[ERROR TCP] Fallo al enviar comando: {e}")
            return False

    def enviar_movej(self, q_objetivo: list):
        """
        Genera la orden controlando límites y rangos radianes.
        """
        for val in q_objetivo:
            if abs(val) > 2 * math.pi:
                print("[ERROR CRÍTICO] Ángulo fuera del rango de seguridad (-2pi a 2pi).")
                return False
        # Formato URScript inyectando las Constantes de Seguridad y envoltorio def...end
        cmd = f"def seq():\n  movej({q_objetivo}, a={MAX_ACC_RAD_S2}, v={MAX_VEL_RAD_S})\nend\n"
        return self.enviar_script(cmd)

    def _bucle_red(self):
        """
        Recepción ininterrumpida orientada a capturar banderas de parada y estados.
        """
        while self.activo:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            try:
                self.sock.connect((self.ip, self.puerto))
                with self.lock:
                    self.conectado = True
                    self.ultimo_error = None
                
                buf = bytearray()
                while self.activo:
                    datos = self.sock.recv(8192)
                    if not datos:
                        break
                    buf.extend(datos)
                    
                    while len(buf) >= 5:
                        L, T = struct.unpack_from(">iB", buf, 0)
                        if L < 5 or L > 65536:
                            raise ValueError(f"Buffer corrupto: L={L}")
                        if len(buf) < L:
                            break
                            
                        paquete = bytes(buf[:L])
                        del buf[:L]
                        
                        if T == 16:  
                            self._parsear_estado(paquete, L)
                        elif T == 20: 
                            self._parsear_mensajes_robot(paquete, L)
                            
            except Exception:
                with self.lock:
                    self.conectado = False
                time.sleep(2.0)
            finally:
                if self.sock:
                    self.sock.close()
                with self.lock:
                    self.conectado = False

    def _parsear_estado(self, paquete: bytes, longitud_total: int):
        """
        Recupera la q_actual y las banderas de operación (Running/Protective Stop).
        """
        pos = 5
        while pos + 5 <= longitud_total:
            tam_sub, tipo_sub = struct.unpack_from(">iB", paquete, pos)
            if tam_sub < 5 or (pos + tam_sub) > longitud_total:
                break
                
            if tipo_sub == 0 and tam_sub >= 47:
                datos = struct.unpack_from(">Q???????B", paquete, pos + 5)
                with self.lock:
                    self.is_power_on = datos[3]             
                    self.is_emergency_stopped = datos[4]    
                    self.is_protective_stopped = datos[5]   
                    self.is_program_running = datos[6]      
                    
            elif tipo_sub == 1 and tam_sub == 251:
                q = []
                for j in range(6):
                    (q_j,) = struct.unpack_from(">d", paquete, pos + 5 + (j * 41))
                    q.append(q_j)
                
                # Ignorar el modo_actual para poder registrar el movimiento físico (Modo 6).
                if len(q) == 6 and not all(v == 0.0 for v in q):
                    with self.lock:
                        self.q_actual = q
            pos += tam_sub

    def _parsear_mensajes_robot(self, paquete: bytes, L: int):
        """
        Detección de Alarmas: Paradas de emergencia o excepciones de runtime.
        """
        if L >= 15:
            _, _, _, _, subtipo = struct.unpack_from(">iBQbb", paquete, 0)
            if subtipo in (6, 10):  
                with self.lock:
                    self.ultimo_error = f"Excepción interna del controlador UR (Tipo 20, Subtipo {subtipo})"

''' INTERFAZ DE CONTROL '''
modo_previsualizacion = False
q_previsualizacion = [0.0]*6

def ejecutar_secuencia(robot: UR3eController, puntos: list):
    """
    Lógica de Lazo Cerrado (Closed-Loop Control) exigida para la supervisión.
    """
    print("\n--- EJECUCIÓN EN LAZO CERRADO (ROBOT REAL) ---")
    for i, p in enumerate(puntos):
        print(f"Ejecutando paso {i+1}/{len(puntos)}...")
        
        if not robot.enviar_movej(p):
            print("Fallo en la comunicación TCP.")
            return

        t_envio = time.time()
        arrancado = False
        while time.time() - t_envio < 1.5:
            with robot.lock:
                if robot.is_program_running:
                    arrancado = True
                    break
            time.sleep(0.02)
            
        if not arrancado:
            with robot.lock:
                err = robot.ultimo_error
            if err:
                print(f"[¡BLOQUEO!] {err}")
                return
            print("[AVISO] El script no se lanzó. Asegure el 'Control Remoto'.")
            return
 
        while True:
            with robot.lock:
                en_marcha = robot.is_program_running
                prot_stop = robot.is_protective_stopped
                err = robot.ultimo_error
                q_act = list(robot.q_actual)
  
            if prot_stop or err or robot.is_emergency_stopped:
                print(f"\n[¡EMERGENCIA!] Parada activa o Seta pulsada.")
                return
                
            if not en_marcha and arrancado:
                error_articular = max([abs(a - b) for a, b in zip(q_act, p)])
                if error_articular < 0.01:
                    print(f"Paso {i+1} completado OK. (Margen de error: {error_articular:.4f} rad)")
                    break
                else:
                    print(f"\n[FALLO] Cinemática abortada lejos del destino (Error: {error_articular:.4f} rad).")
                    return
                    
            time.sleep(0.05)
    print("--- TAREA COMPLETADA ---")

def previsualizar_secuencia(puntos: list):
    """
    Validación Digital Twin sin actuación física real.
    """
    global modo_previsualizacion, q_previsualizacion
    print("\n--- PREVISUALIZACIÓN VIRTUAL ---")
    modo_previsualizacion = True
    for i, p in enumerate(puntos):
        print(f"Mostrando waypoint {i+1}...")
        q_previsualizacion = list(p)
        time.sleep(1.5)
    modo_previsualizacion = False

def hilo_consola(robot: UR3eController, vis: UR3eRG2):
    """
    Menú asíncrono para el Menú de Control.
    """
    global modo_previsualizacion
    secuencia = []
    
    while robot.activo:
        print("\n=== MENÚ DE CONTROL ===")
        print("1. Guardar punto actual")
        print("2. Guardar punto desde deslizadores")
        print("3. Previsualizar secuencia")
        print("4. Ejecutar secuencia")
        print("5. Borrar secuencia")
        print("6. Salir")
        
        opcion = input("Opción: ")

        bloqueado = False   
        with robot.lock:
            if robot.ultimo_error or robot.is_protective_stopped or robot.is_emergency_stopped:
                print(f"\n[ATENCIÓN] Sistema bloqueado: {robot.ultimo_error or 'Parada Protectora activa'}")
                robot.ultimo_error = None
                bloqueado = True

        if bloqueado:
            continue
        
        if opcion == "1":
            with robot.lock:
                q = list(robot.q_actual)
                encendido = robot.is_power_on
            if not encendido:
                print("\n[AVISO] Operación denegada: El brazo robótico está apagado.")
            elif all(v == 0.0 for v in q):
                print("\n[AVISO] Operación denegada: El robot aún no ha transmitido datos articulares válidos.")
            else:
                secuencia.append(q)
                print(f"Punto guardado. Puntos de trayectoria totales: {len(secuencia)}")
            
        elif opcion == "2":
            params = vis.get_param_values()
            q_slider = list(params.values())[:6]
            secuencia.append(q_slider)
            print(f"Punto guiado guardado. Puntos de trayectoria totales: {len(secuencia)}")
            
        elif opcion == "3":
            if secuencia: previsualizar_secuencia(secuencia)
                
        elif opcion == "4":
            if secuencia: ejecutar_secuencia(robot, secuencia)
                
        elif opcion == "5":
            secuencia.clear()
            print("Memoria de trayectoria borrada.")
            
        elif opcion == "6":
            robot.activo = False
            break

def main():
    ip_destino = cargar_ip()
    vis = UR3eRG2()
    robot = UR3eController(ip_destino, PORT)
    robot.iniciar()
    
    t_consola = threading.Thread(target=hilo_consola, args=(robot, vis), daemon=True)
    t_consola.start()
    
    periodo_render = 1.0 / 60.0
    try:
        while robot.activo:
            t_inicio = time.time()

            if modo_previsualizacion:
                vis.move_robot(0, list(q_previsualizacion))
            else:
                with robot.lock:
                    q_actual = list(robot.q_actual)
                if q_actual and not all(q == 0.0 for q in q_actual):
                    vis.move_robot(0, list(q_actual))
                    
            time.sleep(max(0, periodo_render - (time.time() - t_inicio)))
                
    except (KeyboardInterrupt, pybullet.error):
        pass
    finally:
        robot.activo = False
        sys.exit(0)

if __name__ == "__main__":
    main()