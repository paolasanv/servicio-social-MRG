import time
import cv2 as cv

from src.config import SEND_PERIOD
from src.robot_movil import Robot
from src.xbox_controller import XboxController
from src.vision import Vision
from src.controlador import Controlador


# Robot con ArUco 1 (protegido) controlado por Xbox
robot_protegido = Robot("192.168.0.100")

# Robot con ArUco 2 (atacante)
#robot_atacante = Robot("192.168.0.101")

# Robot con ArUco 3 (defensor)
#self.robot_atacante = Robot("192.168.0.X")


# ============
# MÓDULOS
# ============
control_xbox = XboxController()
vision = Vision()
controlador = Controlador()

# ========================
# LOOP PRINCIPAL
# ========================

def ejecutar():
    ultimo_envio_protegido = 0
    #ultimo_envio_atacante = 0
    try:
        while control_xbox.actualizar():

           # ==================================================
            # 1) CONTROL DEL ROBOT PROTEGIDO (XBOX)
            # ==================================================
            v_protegido, w_protegido = (control_xbox.leer_velocidades())
            vr_protegido, vl_protegido = (robot_protegido.calcular_velocidades_ruedas(v_protegido,w_protegido))


            ahora = time.time()

            if ahora - ultimo_envio_protegido >= SEND_PERIOD:
                robot_protegido.enviar_velocidades(vl_protegido, vr_protegido)
                ultimo_envio_protegido = ahora

            # ==================================================
            # 2) VISIÓN ARUCO
            # ==================================================
            frame, poses = (vision.obtener_poses())

            # ==================================================
            # 3) CONTROL DEL ROBOT ATACANTE
            # ==================================================

            #if (1 in poses and 2 in poses):
                #robot_atacante_pose = poses[2]
                #robot_protegido_pose = poses[1]

                #v_atacante, w_atacante = (controlador.control_potencial(robot_atacante_pose, robot_protegido_pose))
                #vr_atacante, vl_atacante = (robot_atacante.calcular_velocidades_ruedas(v_atacante, w_atacante))

                #if ahora - ultimo_envio_atacante >= SEND_PERIOD:
                #    robot_atacante.enviar_velocidades(vr_atacante,vl_atacante)
                #    ultimo_envio_atacante = ahora
            
            #else:
                # Si no hay visión del robot seguidor, detenerlo por seguridad
                #robot_atacante.detener()

            # ==================================================
            # SALIDA
            # ==================================================
            if frame is not None:
                cv.imshow("Seguimiento ArUco", frame)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("Deteniendo robots...")
        robot_protegido.detener()
        robot_protegido.cerrar()

        #robot_atacante.detener()
        #robot_atacante.cerrar()

        control_xbox.cerrar()
        vision.cerrar()


if __name__ == "__main__":
    ejecutar()
