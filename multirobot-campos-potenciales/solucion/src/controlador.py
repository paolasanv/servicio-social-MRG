# controlador.py

import numpy as np

class Controlador:

    def __init__(self):
        self.kv = 0.9
        self.kw = 1
        self.v_max = 3
        self.w_max = 1
        self.k_r = 0.15
        self.dist_seguridad = 0.5
        
        self.dist_repulsion = 0.6
        self.k_rep = 0.2

        # Movimiento en eje x
        self.direccion = 1
        self.velocidad_x = 0.1

    # ======================================================
    # CAMPOS POTENCIALES
    # ======================================================

    def control_potencial(self, xr, yr, theta_r, xo, yo):
        dx = xo - xr
        dy = yo - yr

        d = np.hypot(dx, dy)

        theta_g = np.arctan2(dy, dx)
        theta_e = np.arctan2(np.sin(theta_g - theta_r),np.cos(theta_g - theta_r))

        error = max(0.0, d - self.dist_seguridad)

        v = self.kv * min(error, self.k_r)
        w = - self.kw * np.sin(theta_e)

        if d <= self.dist_seguridad:
            v = 0
            w = 0

        v = np.clip(v, 0, self.v_max)
        w = np.clip(w, -self.w_max, self.w_max)

        return v, w

    def control_potencial_atrac_rep(self, xr, yr, theta_r, xo, yo):

        dx = xo - xr
        dy = yo - yr

        d = np.hypot(dx, dy)

        # ==========================================
        # DIRECCIÓN HACIA EL PROTEGIDO
        # ==========================================

        theta_g = np.arctan2(dy, dx)

        theta_e = np.arctan2(np.sin(theta_g - theta_r), np.cos(theta_g - theta_r))

        # ==========================================
        # CAMPO ATRACTIVO
        # ==========================================
        error_atractivo = max(0.0, d - self.dist_seguridad)

        v_atractiva = self.kv * min(error_atractivo, self.k_r)
        w_atractiva = -self.kw * np.sin(theta_e)

        # ==========================================
        # CAMPO REPULSIVO
        # ==========================================

        if d < self.dist_seguridad and d > 0:

            theta_repulsiva = np.arctan2(yr - yo, xr - xo)
            theta_rep_error = np.arctan2( np.sin(theta_repulsiva - theta_r), np.cos(theta_repulsiva - theta_r))

            # --------------------------------------
            # Fuerza repulsiva
            # --------------------------------------

            fuerza_repulsiva = self.k_rep * (self.dist_seguridad - d)
            v_repulsiva = fuerza_repulsiva
            w_repulsiva = self.kw * np.sin(theta_rep_error)

        else:

            v_repulsiva = 0.0
            w_repulsiva = 0.0

        # ==========================================
        # COMBINACIÓN DE CAMPOS
        # ==========================================

        if d >= self.dist_seguridad:

            v = v_atractiva
            w = w_atractiva

        else:
            v = -v_repulsiva
            w = w_repulsiva

        # ==========================================
        # LIMITES
        # ==========================================

        v = np.clip(v,-self.v_max, self.v_max)
        w = np.clip(w,-self.w_max,self.w_max)

        return v, w

