import time
import cv2 as cv
import numpy as np

from src.config import SEND_PERIOD
from src.robot_movil import Robot
from src.vision import Vision
#from src.controlador import Controlador

#direcciones IP = 192.168.0.101 (caiman verde y blanco), 192.168.0.100 (caimanes amarillos)

# ----- protegido -----
# Robot von ArUco 1 (protegido)
#protegido = Robot("192.168.0.101", 0.9, 1)

PROTEGIDO_VEL_LIN = 0.2
TIEMPO_ESPERA_PROTEGIDO = 5.0       # segundos antes de comenzar
TIEMPO_MOVIMIENTO_PROTEGIDO = 30.0  # segundos que permanece avanzando
# ----------------------

# ----- atacante -----
# Robot con ArUco 2 (atacante)
atacante = Robot("192.168.0.100", 0.2, 1) #kv, kw
# ---------------------

# ----- defensor -----
# Robot con ArUco 3 (defensor)
defensor = Robot("192.168.0.101", 0.8, 1) #kv, kw
# ---------------------

# ----- parámetros de control -----
K_R_ATACANTE = 0.2           # saturación del error de distancia (atracción)
K_REP_ATACANTE = 0.2         # retroceso al invadir la distancia de seguridad del protegido
K_REP_OBST_ATACANTE = 0.5    # repulsión del defensor (obstáculo del atacante)

K_R_DEFENSOR = 0.3           # atracción al punto medio
K_REP_DEFENSOR = 0.5         # repulsión de protegido y atacante

# Si algún ArUco no se ve durante más de este tiempo, se detienen los robots
TIMEOUT_MARCADORES = 0.3     # segundos
# ---------------------------------

vision = Vision()
#controlador = Controlador()

# ArUco 0 = referencia
# ArUco 1 = robot protegido
# ArUco 2 = robot atacante
# ArUco 3 = robot defensor


def ejecutar():
    ultimo_envio_protegido = 0
    ultimo_envio = 0
    ultima_vista = time.time()
    tecla = ""

    try:

        vision.iniciar_camara()
        inicio = time.time()

        while True:

            if tecla == ord('q'):
                break

            # ESTADO DEL ROBOT PROTEGIDO
            tiempo_transcurrido = time.time() - inicio
            protegido_avanzar = (tiempo_transcurrido >= TIEMPO_ESPERA_PROTEGIDO and tiempo_transcurrido < ( TIEMPO_ESPERA_PROTEGIDO + TIEMPO_MOVIMIENTO_PROTEGIDO))

            # ==============================================
            # 1) CONTROL DEL ROBOT PROTEGIDO (esta parte NO es necesaria si SOLO hay ArUco)
            # ==============================================

            """
            if protegido_avanzar:
                vr_protegido, vl_protegido = (protegido.calcular_velocidades_ruedas(PROTEGIDO_VEL_LIN,0))

                if ahora - ultimo_envio_protegido >= SEND_PERIOD:
                    protegido.enviar_velocidades(vl_protegido, vr_protegido)
                    ultimo_envio_protegido = ahora

            else:
                protegido.detener()
            """

            # Control de los robots atacante y defensor (si ambos están presentes)

            frame, poses, tecla = vision.obtener_poses()

            # Se toma el tiempo DESPUÉS de capturar/procesar el frame
            ahora = time.time()

            # protegido_avanzar and
            if (1 in poses and 2 in poses and 3 in poses):
                ultima_vista = ahora

                robot_atacante_pose = poses[2]
                robot_protegido_pose = poses[1]
                robot_defensor_pose = poses[3]

                # ==============================================
                # 2) CONTROL DEL ROBOT ATACANTE
                #    meta: protegido | obstáculo: defensor
                # ==============================================

                v_atacante, w_atacante = atacante.control_potencial_atacante(
                    robot_atacante_pose["x"], robot_atacante_pose["y"], robot_atacante_pose["theta"],
                    # meta
                    robot_protegido_pose["x"], robot_protegido_pose["y"],
                    # obstáculo
                    robot_defensor_pose["x"], robot_defensor_pose["y"],
                    K_R_ATACANTE, K_REP_ATACANTE, K_REP_OBST_ATACANTE)

                vr_atacante, vl_atacante = atacante.calcular_velocidades_ruedas(v_atacante, w_atacante)

                # ==============================================
                # 3) CONTROL DEL ROBOT DEFENSOR
                #    meta: punto medio atacante-protegido
                # ==============================================

                x = (robot_atacante_pose["x"] + robot_protegido_pose["x"]) / 2
                y = (robot_atacante_pose["y"] + robot_protegido_pose["y"]) / 2

                v_defensor, w_defensor = defensor.control_potencial_defensor(
                    robot_defensor_pose["x"], robot_defensor_pose["y"], robot_defensor_pose["theta"],
                    # objetivo: punto medio
                    x, y,
                    # protegido: obstáculo
                    robot_protegido_pose["x"], robot_protegido_pose["y"],
                    # atacante: obstáculo
                    robot_atacante_pose["x"], robot_atacante_pose["y"],
                    K_R_DEFENSOR, K_REP_DEFENSOR)

                vr_defensor, vl_defensor = defensor.calcular_velocidades_ruedas(v_defensor, w_defensor)

                if ahora - ultimo_envio >= SEND_PERIOD:
                    atacante.enviar_velocidades(vr_atacante, vl_atacante)
                    defensor.enviar_velocidades(vr_defensor, vl_defensor)
                    ultimo_envio = ahora

            elif ahora - ultima_vista > TIMEOUT_MARCADORES:
                # Falta algún ArUco: los robots NO deben seguir con la última orden
                if ahora - ultimo_envio >= SEND_PERIOD:
                    atacante.detener()
                    defensor.detener()
                    ultimo_envio = ahora

    except KeyboardInterrupt:
        print("\nInterrupción por teclado.")

    finally:
        print("Deteniendo robots...")
        #protegido.detener()
        #protegido.cerrar()

        # Primero detener (varias veces, UDP no garantiza entrega) y luego cerrar el socket
        for robot in (atacante, defensor):
            try:
                for _ in range(3):
                    robot.detener()
            except OSError as e:
                print(f"No se pudo enviar la parada: {e}")
            robot.cerrar()

        vision.cerrar()

if __name__ == "__main__":
    ejecutar()