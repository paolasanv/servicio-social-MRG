#main.py

import time
import cv2 as cv

from src.config import SEND_PERIOD
from src.robot_movil import Robot
from src.vision import Vision
from src.controlador import Controlador

# ----- protegido -----
#protegido = Robot("192.168.0.100")
PROTEGIDO_VEL_LIN = 0.5
TIEMPO_MOVIMIENTO_PROTEGIDO = 10.0  # segundos
# ----------------------

#------- atacante -----
# Robot con ArUco 2 (atacante)
atacante = Robot("192.168.0.100")
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
            """
            vr_protegido, vl_protegido = protegido.calcular_velocidades_ruedas(PROTEGIDO_VEL_LIN, 0)
            if ahora - inicio < TIEMPO_MOVIMIENTO_PROTEGIDO:
                vr_protegido, vl_protegido = protegido.calcular_velocidades_ruedas(PROTEGIDO_VEL_LIN, 0)
            else:
                vr_protegido, vl_protegido = 0, 0

            if ahora - ultimo_envio_protegido >= SEND_PERIOD:
                protegido.enviar_velocidades(vl_protegido, vr_protegido)
                ultimo_envio_protegido = ahora
            """
            # ==================================================
            # 2) CONTROL DEL ROBOT ATACANTE
            # ==================================================
            
            frame, poses, tecla = vision.obtener_poses()
            if (1 in poses and 2 in poses):
                robot_atacante_pose = poses[2]
                robot_protegido_pose = poses[1]

                v_atacante, w_atacante = controlador.control_potencial(
                    robot_atacante_pose["x"],
                    robot_atacante_pose["y"],
                    robot_atacante_pose["theta"],
                    robot_protegido_pose["x"],
                    robot_protegido_pose["y"])
                vr_atacante, vl_atacante = (atacante.calcular_velocidades_ruedas(v_atacante, w_atacante))

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
        #protegido.detener()
        #protegido.cerrar()
        
        atacante.detener()
        atacante.cerrar()

        vision.cerrar()


if __name__ == "__main__":
    ejecutar()
