#main.py

import time

from src.config import SEND_PERIOD
from src.robot_movil import Robot
#from src.vision import Vision
#from src.controlador import Controlador

protegido = Robot("192.168.0.100")
PROTEGIDO_VEL_LIN = 1
TIEMPO_MOVIMIENTO_PROTEGIDO = 10.0  # segundos
#vision = Vision()
#controlador = Controlador()


def ejecutar():
    ultimo_envio = 0
    vr = 0
    vl = 0
    tecla = ""

    try:

        #vision.iniciar_camara()
        inicio = time.time()

        while True:

            #frame, poses, tecla = vision.obtener_poses()

            if tecla == ord('q'):
                break

            # ==================================================
            # 1) CONTROL DEL ROBOT PROTEGIDO
            # ==================================================

            vr, vl = protegido.calcular_velocidades_ruedas(PROTEGIDO_VEL_LIN, 0)

            ahora = time.time()

            if ahora - inicio < TIEMPO_MOVIMIENTO_PROTEGIDO:
                vr, vl = protegido.calcular_velocidades_ruedas(PROTEGIDO_VEL_LIN, 0)
            else:
                # Detener el robot
                vr, vl = 0, 0

            if ahora - ultimo_envio >= SEND_PERIOD:
                protegido.enviar_velocidades(vr, vl)
                ultimo_envio = ahora


    except KeyboardInterrupt:
        print("\nInterrupción por teclado.")

    finally:
        protegido.detener()
        protegido.cerrar()
        #vision.cerrar()


if __name__ == "__main__":
    ejecutar()
