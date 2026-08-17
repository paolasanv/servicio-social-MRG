import time
import pygame

from src.config import SEND_PERIOD
from src.robot_movil import Robot
from src.xbox_controller import XboxController

robot = Robot("192.168.0.100")
control = XboxController()

def ejecutar():
    ultimo_envio = 0
    try:
        while control.actualizar():
            v, w = control.leer_velocidades()
            vr, vl = robot.calcular_velocidades_ruedas(v, w)
            ahora = time.time()
            if ahora - ultimo_envio >= SEND_PERIOD:
                robot.enviar_velocidades(vr, vl)
                ultimo_envio = ahora
            pygame.time.wait(10)
    finally:
        robot.detener()
        robot.cerrar()
        control.cerrar()

if __name__ == "__main__":
    ejecutar()
