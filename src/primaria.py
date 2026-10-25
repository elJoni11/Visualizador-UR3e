"""
Interfaz Primaria (30001)
Descripción: Cliente TCP multihilo que decodifica paquetes jerárquicos a 10 Hz.
"""

import socket
import struct
import time
import os
import threading
import json
import pybullet
from UR3e_rg2.dynamic_ur3e import UR3eRG2

PORT = 30001
FPS_VISUALIZADOR = 60.0
CONFIG_FILE = "robots.json"

estado_robot = {
    "q_actual": None,
    "isProgramRunning": False,
    "isEmergencyStopped": False
}
lock = threading.Lock()

def cargar_ip(tipo_interfaz="primaria"):
    """
    Filtra el JSON buscando específicamente el tipo de interfaz requerida.
    Evita conectarse al puerto 30001 de un robot configurado para tiempo real.
    """
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                robots_cfg = json.load(f)
                if isinstance(robots_cfg, list):
                    for r in robots_cfg:
                        if r.get("tipo") == tipo_interfaz:
                            return r.get("ip")
                    print(f"[AVISO] No se encontró ningún robot de tipo '{tipo_interfaz}' en {CONFIG_FILE}.")
        except Exception as e:
            print(f"[AVISO] Error al analizar {CONFIG_FILE}: {e}")
            
    ip_defecto = "10.172.21.203"
    print(f"[AVISO] Usando IP de contingencia: {ip_defecto}")
    return ip_defecto

def decodificar_mensaje_estado(mensaje):
    """Decodificación mediante navegación por subpaquetes (TLV)."""
    longitud_total = len(mensaje)
    posicion = 5 
    q_actual = None
    is_program_running = False
    is_emergency_stopped = False
    robot_mode = -1
    
    while posicion + 5 <= longitud_total:
        tam_sub, tipo_sub = struct.unpack_from(">iB", mensaje, posicion)
        if tam_sub < 5 or (posicion + tam_sub) > longitud_total:
            break
            
        if tipo_sub == 0:
            if tam_sub >= 47:
                datos_modo = struct.unpack_from(">Q???????B", mensaje, posicion + 5)
                # Índices 4 y 5 capturan seta física y parada de software respectivamente
                is_emergency_stopped = datos_modo[4] or datos_modo[5]
                is_program_running = datos_modo[6]
                robot_mode = datos_modo[8]

        elif tipo_sub == 1:
            q_actual_temp = []
            for j in range(6):
                offset_art = posicion + 5 + (j * 41)
                if offset_art + 8 <= longitud_total:
                    (q_j,) = struct.unpack_from(">d", mensaje, offset_art)
                    q_actual_temp.append(q_j)
            if len(q_actual_temp) == 6:
                q_actual = q_actual_temp

        posicion += tam_sub

    if robot_mode == 7:    
        return q_actual, is_program_running, is_emergency_stopped
    else:
        return None, is_program_running, is_emergency_stopped

def hilo_receptor(ip_destino):
    """Aísla la recepción de red para no congelar los gráficos."""
    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        try:
            print(f"[RED] Conectando a {ip_destino}:{PORT}...")
            sock.connect((ip_destino, PORT))
            print("[RED] ¡Conexión establecida!")
            
            buffer = bytearray()
            while True:
                datos = sock.recv(4096)
                if not datos:
                    print("[RED] El robot cerró la conexión (FIN TCP).")
                    break
                    
                buffer.extend(datos)
                while len(buffer) >= 5:
                    L, T = struct.unpack_from(">iB", buffer, 0)
                    if L < 5 or L > 65536:
                        raise ValueError("Buffer desalineado.")
                    if len(buffer) < L:
                        break

                    mensaje = bytes(buffer[:L])
                    del buffer[:L]
                    
                    if T == 16: 
                        q, is_running, is_stop = decodificar_mensaje_estado(mensaje)
                        with lock:
                            if q is not None:
                                estado_robot["q_actual"] = q
                            estado_robot["isProgramRunning"] = is_running
                            estado_robot["isEmergencyStopped"] = is_stop

        except Exception as e:
            print(f"[RED] Error: {e}. Reintentando en 2s...")
            time.sleep(2.0)
        finally:
            sock.close()

def main():
    ip_destino = cargar_ip("primaria")
    print("Iniciando visualizador PyBullet...")
    vis = UR3eRG2()
    robot_id = 0
    periodo_dibujo = 1.0 / FPS_VISUALIZADOR
    
    t = threading.Thread(target=hilo_receptor, args=(ip_destino,), daemon=True)
    t.start()

    alerta_previa = False
    
    try:
        while True:
            t_inicio = time.time()
            
            # Copia atómica del estado para liberar el cerrojo rápido
            with lock:
                q_ultimo = estado_robot["q_actual"]
                alerta_actual = estado_robot["isEmergencyStopped"]
            
            if q_ultimo is not None:
                vis.move_robot(robot_id, list(q_ultimo))

            if alerta_actual and not alerta_previa:
                print("\n[ALERTA] Parada protectora o Seta física registrada.")
            alerta_previa = alerta_actual
                
            tiempo_espera = periodo_dibujo - (time.time() - t_inicio)
            if tiempo_espera > 0:
                time.sleep(tiempo_espera)
                
    except (KeyboardInterrupt, pybullet.error):
        print("\nCerrando la conexión...")

if __name__ == "__main__":
    main()