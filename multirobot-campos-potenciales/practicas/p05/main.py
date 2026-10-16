import time
import cv2 as cv

from src.config import SEND_PERIOD
from src.robot_movil import Robot
from src.vision import Vision


# ======================================================
# CONFIGURACIÓN DEL SISTEMA
# ======================================================

# Identificadores de los marcadores ArUco
ARUCO_REFERENCIA = 0
ARUCO_OBJETIVO = 1
ARUCO_ROBOT = 2

# Dirección IP del ESP32 del robot asociado al ArUco 2
robot = Robot("192.168.0.102", 1, 1)  # kv, kw

# Tiempo máximo sin detectar ambos marcadores antes de detener el robot
TIMEOUT_MARCADORES = 0.3  # segundos

# Ganancias del controlador de campos potenciales
K_R = 1.0
K_REP = 1.0

# Inicialización del sistema de visión
vision = Vision()


# ======================================================
# PROGRAMA PRINCIPAL
# ======================================================

def ejecutar():

    ultimo_envio = 0.0
    ultima_vista = time.time()
    tecla = -1

    try:

        # Inicializar la cámara
        vision.iniciar_camara()

        while True:

            # ==============================================
            # OBTENER POSES DE LOS MARCADORES
            # ==============================================

            frame, poses, tecla = vision.obtener_poses()

            ahora = time.time()

            # Salir al presionar la tecla 'q'
            if tecla == ord('q'):
                break

            # ==============================================
            # VERIFICAR LA PRESENCIA DE LOS MARCADORES
            # ==============================================

            if ARUCO_ROBOT in poses and ARUCO_OBJETIVO in poses:

                # Se actualiza el instante de la última detección
                # simultánea del robot y del objetivo.
                ultima_vista = ahora

                # ------------------------------------------
                # Obtener las poses del robot y del objetivo
                # ------------------------------------------

                robot_pose = poses[ARUCO_ROBOT]
                objetivo_pose = poses[ARUCO_OBJETIVO]

                # Posición y orientación del robot
                xr = robot_pose["x"]
                yr = robot_pose["y"]
                theta_r = robot_pose["theta"]

                # Posición del objetivo
                xo = objetivo_pose["x"]
                yo = objetivo_pose["y"]

                # ------------------------------------------
                # CONTROL MEDIANTE CAMPOS POTENCIALES
                # ------------------------------------------
                # El robot asociado al ArUco 2 debe seguir
                # al objetivo asociado al ArUco 1.
                #
                # El ArUco 0 se utiliza como referencia
                # del sistema de coordenadas y sus
                # transformaciones son realizadas en vision.py.
                # ------------------------------------------

                v, w = robot.control_potencial_atrac_rep(xr, yr, theta_r, xo, yo, K_R, K_REP)

                # ------------------------------------------
                # Convertir velocidades del robot en
                # velocidades angulares de las ruedas
                # ------------------------------------------

                vr_robot, vl_robot = robot.calcular_velocidades_ruedas(v, w)

                # ------------------------------------------
                # Enviar comandos al robot periódicamente
                # ------------------------------------------

                if ahora - ultimo_envio >= SEND_PERIOD:
                    robot.enviar_velocidades(vr_robot, vl_robot)
                    ultimo_envio = ahora

            # ==============================================
            # DETENCIÓN POR PÉRDIDA DE MARCADORES
            # ==============================================

            elif ahora - ultima_vista > TIMEOUT_MARCADORES:

                # Si no se detecta el robot o el objetivo durante el tiempo establecido, detenerlo.
                if ahora - ultimo_envio >= SEND_PERIOD:
                    robot.detener()
                    ultimo_envio = ahora

    except KeyboardInterrupt:
        print("\nInterrupción por teclado.")

    finally:

        print("Deteniendo el robot...")
        try:
            robot.detener()
        except OSError as e:
            print(f"No se pudo enviar la orden de parada: {e}")

        robot.cerrar()
        vision.cerrar()


if __name__ == "__main__":
    ejecutar()
