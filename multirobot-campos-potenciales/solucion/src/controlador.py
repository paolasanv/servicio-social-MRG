# controlador.py

import numpy as np

class Controlador:

    def __init__(self):
        self.kv = 0.9
        self.kw = 1

        self.v_max = 3
        self.w_max = 1

        self.dist_seguridad = 0.55
        
        self.dist_repulsion = 0.5

    # ======================================================
    # CAMPOS POTENCIALES
    # ======================================================

    def control_potencial(self, xr, yr, theta_r, xo, yo, k_r):
        dx = xo - xr
        dy = yo - yr

        d = np.hypot(dx, dy)

        theta_g = np.arctan2(dy, dx)
        theta_e = np.arctan2(np.sin(theta_g - theta_r),np.cos(theta_g - theta_r))

        error = max(0.0, d - self.dist_seguridad)

        v = self.kv * min(error, k_r)
        w = - self.kw * np.sin(theta_e)

        if d <= self.dist_seguridad:
            v = 0
            w = 0

        v = np.clip(v, 0, self.v_max)
        w = np.clip(w, -self.w_max, self.w_max)

        return v, w

    def control_potencial_atrac_rep(self, xr, yr, theta_r, xo, yo, k_r, k_rep):

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

            v_atractiva = self.kv * min(error_atractivo, k_r)
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

                fuerza_repulsiva = k_rep * (self.dist_seguridad - d)
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

    def control_potencial_defensor(
            self,
            xr, yr, theta_r,
            objetivo_x, objetivo_y,
            protegido_x, protegido_y,
            atacante_x, atacante_y,
            kr,
            krep):

        # ======================================================
        # 1) CAMPO ATRACTIVO HACIA EL PUNTO MEDIO
        # ======================================================

        dx_obj = objetivo_x - xr
        dy_obj = objetivo_y - yr

        d_obj = np.hypot(dx_obj, dy_obj)

        # Fuerza atractiva
        f_ax = kr * dx_obj
        f_ay = kr * dy_obj

        # ======================================================
        # 2) CAMPO REPULSIVO DEL ROBOT PROTEGIDO
        # ======================================================

        dx_prot = xr - protegido_x
        dy_prot = yr - protegido_y

        d_prot = np.hypot(dx_prot, dy_prot)

        f_rx_prot = 0.0
        f_ry_prot = 0.0

        if 0 < d_prot < self.dist_repulsion:

            # Vector unitario desde el protegido hacia el defensor
            ux = dx_prot / d_prot
            uy = dy_prot / d_prot

            # Fuerza repulsiva
            fuerza = krep * (
                self.dist_repulsion - d_prot
            )

            f_rx_prot = fuerza * ux
            f_ry_prot = fuerza * uy

        # ======================================================
        # 3) CAMPO REPULSIVO DEL ROBOT ATACANTE
        # ======================================================

        dx_atac = xr - atacante_x
        dy_atac = yr - atacante_y

        d_atac = np.hypot(dx_atac, dy_atac)

        f_rx_atac = 0.0
        f_ry_atac = 0.0

        if 0 < d_atac < self.dist_repulsion:

            # Vector unitario desde el atacante hacia el defensor
            ux = dx_atac / d_atac
            uy = dy_atac / d_atac

            # Fuerza repulsiva
            fuerza = krep * (
                self.dist_repulsion - d_atac
            )

            f_rx_atac = fuerza * ux
            f_ry_atac = fuerza * uy

        # ======================================================
        # 4) SUMAR LOS CAMPOS
        # ======================================================

        fx = (
            f_ax
            + f_rx_prot
            + f_rx_atac
        )

        fy = (
            f_ay
            + f_ry_prot
            + f_ry_atac
        )

        # ======================================================
        # 5) DIRECCIÓN RESULTANTE
        # ======================================================

        theta_resultante = np.arctan2(fy, fx)

        theta_error = np.arctan2(
            np.sin(theta_resultante - theta_r),
            np.cos(theta_resultante - theta_r)
        )

        # ======================================================
        # 6) VELOCIDADES
        # ======================================================

        magnitud = np.hypot(fx, fy)

        v = self.kv * magnitud
        w = -self.kw * np.sin(theta_error)

        # ======================================================
        # 7) LIMITAR VELOCIDADES
        # ======================================================

        v = np.clip(v, 0, self.v_max)
        w = np.clip(w, -self.w_max, self.w_max)

        return v, w