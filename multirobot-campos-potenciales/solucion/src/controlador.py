# controlador.py

import numpy as np


class Controlador:

    def __init__(self):
        self.kv = 0.1
        self.kw = 0.1
        self.v_max = 3
        self.w_max = 1
        self.k_r = 0.15
        self.dist_seguridad = 0.50

    # ======================================================
    # CAMPOS POTENCIALES 
    # ======================================================

    def control_potencial(self, xr, yr, theta_r, xo, yo):
        dx = xo - xr
        dy = yo - yr

        d = np.hypot(dx, dy)

        theta_g = np.arctan2(dy, dx)
        theta_e = np.arctan2(np.sin(theta_g - theta_r), np.cos(theta_g - theta_r))

        # Distancia restante descontando la zona de seguridad
        error = max(0.0, d - self.dist_seguridad)

        v = self.kv * min(error, self.k_r)
        w = self.kw * np.sin(theta_e)

        if d <= self.dist_seguridad:
            v = 0
            w = 0

        v = np.clip(v,0, self.v_max)
        w = np.clip( w, self.w_max, self.w_max)

        return v, w