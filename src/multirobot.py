"""
Visualizador de Múltiples Robots
Soporta múltiples conexiones independientes TCP mediante hilos.
"""

import socket
import struct
import time
import json
import os
import sys
import threading
import pybullet
from UR3e_rg2.dynamic_ur3e import UR3eRG2

CONFIG_FILE = "robots.json"
FPS_REFRESCO = 60.0

class UR3eWorker(threading.Thread):
    """Hilo independiente por cada robot. Evita cuelgues cruzados."""
    def __init__(self, cfg: dict, lock: threading.Lock, estado_compartido: dict):
        super().__init__(daemon=True)
        self.cfg = cfg
        self.robot_id = cfg["id"]
        self.ip = cfg["ip"]
        self.puerto = cfg["puerto"]
        self.tipo_interfaz = cfg["tipo"]
        self.lock = lock
        self.estado_compartido = estado_compartido
        self.activo = True

    def run(self):
        while self.activo:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            try:
                sock.connect((self.ip, self.puerto))
                with self.lock:
                    self.estado_compartido[self.robot_id]["conectado"] = True
                    self.estado_compartido[self.robot_id]["e_stop"] = False
                print(f"[CONECTADO] Robot {self.robot_id} en {self.ip}:{self.puerto}")
                
                buffer = bytearray()
                while self.activo:
                    datos = sock.recv(16384)
                    if not datos:
                        print(f"[FIN] Robot {self.robot_id} cerró la conexión.")
                        break
                    
                    buffer.extend(datos)
                    # Polimorfismo basado en la interfaz
                    if self.tipo_interfaz == "realtime":
                        self._procesar_buffer_realtime(buffer)
                    else:
                        self._procesar_buffer_primaria(buffer)

            except Exception as err:
                with self.lock:
                    if self.estado_compartido[self.robot_id]["conectado"]:
                        print(f"[DESCONEXIÓN] Robot {self.robot_id}: Enlace perdido ({err}).")
                    self.estado_compartido[self.robot_id]["conectado"] = False
                time.sleep(2.0)
            finally:
                sock.close()
                with self.lock:
                    self.estado_compartido[self.robot_id]["conectado"] = False

    def _procesar_buffer_realtime(self, buffer: bytearray):
        while len(buffer) >= 4:
            (L,) = struct.unpack_from(">i", buffer, 0)
            if L < 300 or L > 65536:
                raise ValueError("Longitud RT inválida")
            if len(buffer) < L:
                break
                
            paquete = bytes(buffer[:L])
            del buffer[:L]
            
            (robot_mode,) = struct.unpack_from(">d", paquete, 756)
            (safety_mode,) = struct.unpack_from(">d", paquete, 764)
            
            with self.lock:
                self.estado_compartido[self.robot_id]["e_stop"] = (int(safety_mode) > 2)
            
            if int(robot_mode) == 7:
                q_rad = struct.unpack_from(">6d", paquete, 252)
                with self.lock:
                    self.estado_compartido[self.robot_id]["q"] = list(q_rad)

    def _procesar_buffer_primaria(self, buffer: bytearray):
        while len(buffer) >= 5:
            L, T = struct.unpack_from(">iB", buffer, 0)
            if L < 5 or L > 65536:
                raise ValueError("Longitud Primaria inválida")
            if len(buffer) < L:
                break
                
            paquete = bytes(buffer[:L])
            del buffer[:L]
            
            if T == 16:
                pos, q_rad, robot_mode = 5, None, -1
                while pos + 5 <= L:
                    tam_sub, tipo_sub = struct.unpack_from(">iB", paquete, pos)
                    if tam_sub < 5 or (pos + tam_sub) > L:
                        break
                        
                    if tipo_sub == 0 and tam_sub >= 47:
                        datos = struct.unpack_from(">Q???????B", paquete, pos + 5)
                        robot_mode = datos[8]
                        with self.lock:
                            self.estado_compartido[self.robot_id]["e_stop"] = datos[4] or datos[5]
                            
                    elif tipo_sub == 1:
                        q_rad = []
                        for j in range(6):
                            (q_j,) = struct.unpack_from(">d", paquete, pos + 5 + (j * 41))
                            q_rad.append(q_j)
                            
                    pos += tam_sub

                if q_rad and len(q_rad) == 6 and robot_mode == 7:
                    with self.lock:
                        self.estado_compartido[self.robot_id]["q"] = q_rad

def main():
    if not os.path.exists(CONFIG_FILE):
        print(f"Error: No se encuentra el archivo de configuración {CONFIG_FILE}")
        sys.exit(1)
        
    with open(CONFIG_FILE, "r") as f:
        robots_cfg = json.load(f)
        
    vis = UR3eRG2()
    estado_compartido = {}
    lock = threading.Lock()
    workers = []
    
    for r in robots_cfg:
        r_id = r["id"]
        if r_id > 0:
            real_id = vis.spawn_robot(r["base_pos"])
            r["id"] = real_id 
            r_id = real_id
            
        # Se inicializa e_stop para evitar el KeyError
        estado_compartido[r_id] = {"q": [0.0]*6, "conectado": False, "e_stop": False}

        w = UR3eWorker(r, lock, estado_compartido)
        w.start()
        workers.append(w)
        
    periodo = 1.0 / FPS_REFRESCO
    print("\nVisualizador Multirobot activo.")
    
    alerta_previa = False
    try:
        while True:
            t0 = time.time()
            hay_emergencia = False
            
            with lock:
                copia_estado = {}
                for k, v in estado_compartido.items():
                    if v["conectado"]:
                        copia_estado[k] = list(v["q"])
                        if v["e_stop"]: 
                            hay_emergencia = True
                
            for r_id, q_vector in copia_estado.items():
                if q_vector and not all(q == 0.0 for q in q_vector):
                    vis.move_robot(r_id, list(q_vector))
            
            if hay_emergencia and not alerta_previa:
                print("\n[ALERTA] Emergencia física activada en al menos un brazo robótico.")
            alerta_previa = hay_emergencia
            
            time.sleep(max(0, periodo - (time.time() - t0)))
                
    except (KeyboardInterrupt, pybullet.error):
        print("\nCerrando la conexión...")
        for w in workers:
            w.activo = False
        sys.exit(0)

if __name__ == "__main__":
    main()