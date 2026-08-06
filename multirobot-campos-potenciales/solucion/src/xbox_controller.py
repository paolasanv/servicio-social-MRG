# xobox_controller.py

import pygame

from src.config import (AXIS_V, AXIS_W, DEADZONE, V_MAX, W_MAX, SIGNO_V, SIGNO_W)
								
class XboxController:

    def __init__(self):

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No se encontró un control Xbox.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def aplicar_deadzone(self, valor):

        if abs(valor) < DEADZONE:
            return 0

        return valor

    def limitar(self, valor, minimo, maximo):
        return max(min(valor, maximo), minimo)

    def leer_velocidades(self):

        pygame.event.pump()

        raw_v = -self.joystick.get_axis(AXIS_V)
        raw_w = self.joystick.get_axis(AXIS_W)

        raw_v = self.aplicar_deadzone(raw_v)
        raw_w = self.aplicar_deadzone(raw_w)

        v = SIGNO_V * raw_v * V_MAX
        w = SIGNO_W * raw_w * W_MAX

        v = self.limitar(v, -V_MAX, V_MAX)
        w = self.limitar(w, -W_MAX, W_MAX)

        return v, w

    def actualizar(self):

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                return False

        return True

    def cerrar(self):
        pygame.quit()