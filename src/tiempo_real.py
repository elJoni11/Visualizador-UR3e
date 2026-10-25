"""
Interfaz de Tiempo Real (30003)
Descripción: Cliente TCP de alto rendimiento sin telemetría intrusiva.
"""

import socket
import struct
import time
import math
import os
import json
import pybullet
from UR3e_rg2.dynamic_ur3e import UR3eRG2

PORT = 30003
FPS_VISUALIZADOR = 60.0
CONFIG_FILE = "robots.json"

def cargar_ip(tipo_interfaz="realtime"):
    """Filtra el JSON buscando un robot válido para la interfaz RT."""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                robots_cfg = json.load(f)
                if isinstance(robots_cfg, list):
                    for r in robots_cfg:
                        if r.get("tipo") == tipo_interfaz:
                            return r.get("ip")
                    print(f"[AVISO] No se encontró ningún robot '{tipo_interfaz}'.")
        except Exception as e:
            print(f"[AVISO] Error al analizar {CONFIG_FILE}: {e}")
            
    ip_defecto = "10.172.21.203"
    print(f"[AVISO] Usando IP de contingencia: {ip_defecto}")
    return ip_defecto

def main():
    ip_destino = cargar_ip("realtime")
    print("Iniciando visualizador PyBullet...")
    vis = UR3eRG2()
    robot_id = 0
    periodo_dibujo = 1.0 / FPS_VISUALIZADOR

    alerta_previa = False

    while True:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        
        try:
            print(f"Conectando a la interfaz de tiempo real en {ip_destino}:{PORT}...")
            sock.connect((ip_destino, PORT))
            print("¡Conexión establecida a 500 Hz!")
            
            buffer = bytearray()
            t_ultimo_dibujo = 0.0
            q_ultimo = None
            
            while True:
                datos = sock.recv(16384)
                if not datos:
                    print("[DESCONEXIÓN] El robot cerró la conexión abruptamente.")
                    break
                    
                buffer.extend(datos)
                while len(buffer) >= 4:
                    (L,) = struct.unpack_from(">i", buffer, 0)
                    if L < 300 or L > 65536:
                        raise ValueError(f"Longitud de paquete inválida ({L} bytes).")
                    if len(buffer) < L:
                        break
                        
                    paquete = bytes(buffer[:L])
                    del buffer[:L]
                    
                    # Offset fijo de la documentación UR para Tiempo Real
                    (robot_mode,) = struct.unpack_from(">d", paquete, 756)
                    (safety_mode,) = struct.unpack_from(">d", paquete, 764)
                    
                    if int(robot_mode) == 7:
                        q_ultimo = struct.unpack_from(">6d", paquete, 252)
                
                ahora = time.time()
                
                # Desacoplamiento de frecuencias (500Hz red -> 60Hz gráficos)
                if q_ultimo is not None and (ahora - t_ultimo_dibujo) >= periodo_dibujo:
                    vis.move_robot(robot_id, list(q_ultimo))
                    t_ultimo_dibujo = ahora

                # Detección de paradas usando el Safety Mode
                alerta_actual = (int(safety_mode) > 2)
                if alerta_actual and not alerta_previa:
                    print("\n[ALERTA] Parada protectora o Seta registrada por RT.")
                alerta_previa = alerta_actual

        except Exception as e:
            print(f"[ERROR DE RED] {e}. Reintentando en 2 segundos...")
            time.sleep(2.0)
        except (KeyboardInterrupt, pybullet.error):
            print("\nCerrando la conexión...")
            sock.close()
            break
        finally:
            sock.close()

if __name__ == "__main__":
    main()