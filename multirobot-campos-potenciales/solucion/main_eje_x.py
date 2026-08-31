import time
import cv2 as cv
import numpy as np

from src.config import SEND_PERIOD
from src.robot_movil import Robot
from src.vision import Vision
from src.controlador import Controlador

# ----- protegido -----
# Robot von ArUco 1 (protegido)
protegido = Robot("192.168.0.101")  #microusb
PROTEGIDO_VEL_LIN = 0.5
TIEMPO_MOVIMIENTO_PROTEGIDO = 10.0  # segundos
# ----------------------

#------- atacante -----
# Robot con ArUco 2 (atacante)
atacante = Robot("192.168.0.100") #usb-c (caimanes amarillos)
#-----------------------

vision = Vision()
controlador = Controlador()

# AruCo 0 = es la referencia
# ArUco 1 = es el robot protegido 
# ArUco 2 = es el robot atacante

def ejecutar():
    ultimo_envio_protegido = 0
    ultimo_envio_atacante = 0
    tecla = ""

    try:

        vision.iniciar_camara()
        inicio = time.time()

        while True:
            if tecla == ord('q'):
                break

            ahora = time.time()

            # ==================================================
            # 1) CONTROL DEL ROBOT PROTEGIDO
            # ==================================================
            
            vr_protegido, vl_protegido = protegido.calcular_velocidades_ruedas(PROTEGIDO_VEL_LIN, 0)
            if ahora - inicio < TIEMPO_MOVIMIENTO_PROTEGIDO:
                vr_protegido, vl_protegido = protegido.calcular_velocidades_ruedas(PROTEGIDO_VEL_LIN, 0)
            else:
                vr_protegido, vl_protegido = 0, 0

            if ahora - ultimo_envio_protegido >= SEND_PERIOD:
                protegido.enviar_velocidades(vl_protegido, vr_protegido)
                ultimo_envio_protegido = ahora
        
            # ==================================================
            # 2) CONTROL DEL ROBOT ATACANTE
            # ==================================================
            
            frame, poses, tecla = vision.obtener_poses()
            if (1 in poses and 2 in poses):
                robot_atacante_pose = poses[2]
                robot_protegido_pose = poses[1]

                # ==================================================
                # DEBUG: POSICIONES Y ORIENTACIONES
                # ==================================================
                """
                dx = robot_protegido_pose["x"] - robot_atacante_pose["x"]
                dy = robot_protegido_pose["y"] - robot_atacante_pose["y"]

                theta_r = robot_atacante_pose["theta"]

                theta_g = np.arctan2(dy, dx)

                theta_e = np.arctan2(
                    np.sin(theta_g - theta_r),
                    np.cos(theta_g - theta_r))

                print(
                    f"\n"
                    f"Atacante: "
                    f"x={robot_atacante_pose['x']:.3f}, "
                    f"y={robot_atacante_pose['y']:.3f}, "
                    f"theta={np.degrees(theta_r):.1f}°\n"
                    f"Objetivo:  "
                    f"x={robot_protegido_pose['x']:.3f}, "
                    f"y={robot_protegido_pose['y']:.3f}\n"
                    f"dx={dx:.3f}, "
                    f"dy={dy:.3f}\n"
                    f"theta_g={np.degrees(theta_g):.1f}°, "
                    f"theta_e={np.degrees(theta_e):.1f}°")
                """

                # ==================================================

              
                v_atacante, w_atacante = controlador.control_potencial(
                    robot_atacante_pose["x"],
                    robot_atacante_pose["y"],
                    robot_atacante_pose["theta"],
                    robot_protegido_pose["x"],
                    robot_protegido_pose["y"])
                vr_atacante, vl_atacante = (atacante.calcular_velocidades_ruedas(v_atacante, w_atacante))
                """
                print(
                    f"Control: "
                    f"v={v_atacante:.3f} m/s, "
                    f"w={w_atacante:.3f} rad/s | "
                    f"vr={vr_atacante:.3f} rad/s, "
                    f"vl={vl_atacante:.3f} rad/s")
                """
                if ahora - ultimo_envio_atacante >= SEND_PERIOD:
                    atacante.enviar_velocidades(vr_atacante, vl_atacante)
                    ultimo_envio_atacante = ahora
                    
            else:
                # Si no hay visión del robot seguidor, detenerlo por seguridad
                atacante.detener()   

    except KeyboardInterrupt:
        print("\nInterrupción por teclado.")

    finally:
        print("Deteniendo robots...")
        protegido.detener()
        protegido.cerrar()
        
        atacante.detener()
        atacante.cerrar()

        vision.cerrar()


if __name__ == "__main__":
    ejecutar()
