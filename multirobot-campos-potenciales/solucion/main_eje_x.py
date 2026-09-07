import time
import cv2 as cv
import numpy as np

from src.config import SEND_PERIOD
from src.robot_movil import Robot
from src.vision import Vision
from src.controlador import Controlador

# ----- protegido -----
# Robot von ArUco 1 (protegido)
protegido = Robot("192.168.0.101")  # caiman verde y blanco

PROTEGIDO_VEL_LIN = 0.2
TIEMPO_ESPERA_PROTEGIDO = 3.0      # segundos antes de comenzar
TIEMPO_MOVIMIENTO_PROTEGIDO = 10.0  # segundos que permanece avanzando
# ----------------------

# ----- atacante -----
# Robot con ArUco 2 (atacante)
atacante = Robot("192.168.0.100")  # caimanes amarillos
# ---------------------

vision = Vision()
controlador = Controlador()

# ArUco 0 = referencia
# ArUco 1 = robot protegido
# ArUco 2 = robot atacante


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
            tiempo_transcurrido = ahora - inicio

            # ==============================================
            # ESTADO DEL ROBOT PROTEGIDO
            # ==============================================
            protegido_avanzar = (tiempo_transcurrido >= TIEMPO_ESPERA_PROTEGIDO and tiempo_transcurrido < ( TIEMPO_ESPERA_PROTEGIDO + TIEMPO_MOVIMIENTO_PROTEGIDO))

            # ==============================================
            # 1) CONTROL DEL ROBOT PROTEGIDO
            # ==============================================

            if protegido_avanzar:
                vr_protegido, vl_protegido = (protegido.calcular_velocidades_ruedas(PROTEGIDO_VEL_LIN,0))

                if ahora - ultimo_envio_protegido >= SEND_PERIOD:
                    protegido.enviar_velocidades(vl_protegido, vr_protegido)
                    ultimo_envio_protegido = ahora

            else:
                protegido.detener()

            frame, poses, tecla = vision.obtener_poses()

            # ==============================================
            # 3) CONTROL DEL ROBOT ATACANTE
            # ==============================================

            if protegido_avanzar and (1 in poses and 2 in poses):
                robot_atacante_pose = poses[2]
                robot_protegido_pose = poses[1]

                v_atacante, w_atacante = (controlador.control_potencial_atrac_rep(
                        robot_atacante_pose["x"],
                        robot_atacante_pose["y"],
                        robot_atacante_pose["theta"],
                        robot_protegido_pose["x"],
                        robot_protegido_pose["y"]))
                
                vr_atacante, vl_atacante = (atacante.calcular_velocidades_ruedas(v_atacante, w_atacante))

                if ahora - ultimo_envio_atacante >= SEND_PERIOD:
                    atacante.enviar_velocidades(vr_atacante, vl_atacante)
                    ultimo_envio_atacante = ahora

            else:
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
