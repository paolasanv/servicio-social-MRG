import time

from src.config import SEND_PERIOD
from src.robot_movil import Robot
from src.xbox_controller import XboxController

robot = Robot("192.168.0.100")
control = XboxController()

def ejecutar(self):
    ultimo_envio = 0
    try:
        while self.control.actualizar():
            v, w = self.control.leer_velocidades()
            vr, vl = self.robot.calcular_velocidades_ruedas(v, w)
            ahora = time.time()
            if ahora - ultimo_envio >= SEND_PERIOD:
                self.robot.enviar_velocidades(vr, vl)
                ultimo_envio = ahora
            pygame.time.wait(10)
    finally:
        self.robot.detener()
        self.robot.cerrar()
        self.control.cerrar()

if __name__ == "__main__":
    ejecutar()