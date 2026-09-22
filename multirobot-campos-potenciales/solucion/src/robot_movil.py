# robot_movil.py

import socket
import numpy as np

from src.config import UDP_PORT, R, L


def _wrap(angulo):
    """Normaliza un ángulo a [-pi, pi]."""
    return np.arctan2(np.sin(angulo), np.cos(angulo))


class Robot:

    def __init__(self, ip, kv, kw):
        self.ip = ip
        self.kv = kv
        self.kw = kw

        self.v_max = 3
        self.w_max = 1

        self.dist_seguridad = 0.55
        self.dist_repulsion = 0.55

        self.port = UDP_PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def calcular_velocidades_ruedas(self, v, w):
        vr = (v + w * L / 2) / R
        vl = (v - w * L / 2) / R
        return vr, vl

    def enviar_velocidades(self, vr, vl):
        mensaje = f"{vr:.3f},{vl:.3f}"
        self.sock.sendto(mensaje.encode(), (self.ip, self.port))
        print(f"Enviado -> derecha: {vr:+.3f} rad/s | izquierda: {vl:+.3f} rad/s")

    def detener(self):
        self.enviar_velocidades(0, 0)

    def cerrar(self):
        self.sock.close()

    # ======================================================
    # UTILIDADES COMUNES PARA CAMPOS POTENCIALES
    # ======================================================

    def _repulsion(self, xr, yr, xo, yo, d0, k_rep, gx, gy, k_tang):
        """
        Fuerza repulsiva del obstáculo (xo, yo) sobre el robot (xr, yr).

        - Componente radial: aleja al robot del obstáculo (lineal, 0 en d0).
        - Componente tangencial: rodea al obstáculo por el lado hacia el
          que está la meta (gx, gy = vector unitario hacia la meta).
          Rompe el mínimo local cuando meta y obstáculo están alineados.
        """
        dx, dy = xr - xo, yr - yo
        d = np.hypot(dx, dy)

        if d < 1e-6 or d >= d0:
            return 0.0, 0.0

        ux, uy = dx / d, dy / d
        mag = k_rep * (d0 - d) / d0

        cruz = ux * gy - uy * gx
        signo = 1.0 if cruz >= 0 else -1.0

        fx = mag * ux + signo * k_tang * mag * (-uy)
        fy = mag * uy + signo * k_tang * mag * (ux)
        return fx, fy

    def _fuerza_a_velocidades(self, fx, fy, theta_r, umbral=1e-3):
        """
        Convierte un campo resultante (fx, fy) en (v, w) para un robot
        diferencial, conservando la convención de signo w = -kw*sin(theta_e).

        - Si la fuerza apunta detrás del robot (|theta_e| > 90°), retrocede
          en lugar de girar 180° avanzando hacia el obstáculo.
        - v se atenúa con cos(theta_e): solo avanza en la medida en que
          está alineado con la fuerza.
        """
        magnitud = np.hypot(fx, fy)
        if magnitud < umbral:
            return 0.0, 0.0

        theta_resultante = np.arctan2(fy, fx)
        theta_e = _wrap(theta_resultante - theta_r)

        sentido = 1.0
        if abs(theta_e) > np.pi / 2:
            theta_e = _wrap(theta_e + np.pi)
            sentido = -1.0

        v = sentido * self.kv * magnitud * max(0.0, np.cos(theta_e))
        w = -self.kw * np.sin(theta_e)

        v = np.clip(v, -self.v_max, self.v_max)
        w = np.clip(w, -self.w_max, self.w_max)

        return v, w

    # ======================================================
    # CAMPOS POTENCIALES
    # ======================================================

    def control_potencial(self, xr, yr, theta_r, xo, yo, k_r):
        dx = xo - xr
        dy = yo - yr

        d = np.hypot(dx, dy)

        theta_g = np.arctan2(dy, dx)
        theta_e = np.arctan2(np.sin(theta_g - theta_r), np.cos(theta_g - theta_r))

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
        """Versión original (sin considerar al defensor). Se conserva por compatibilidad."""

        dx = xo - xr
        dy = yo - yr

        d = np.hypot(dx, dy)

        theta_g = np.arctan2(dy, dx)
        theta_e = np.arctan2(np.sin(theta_g - theta_r), np.cos(theta_g - theta_r))

        error_atractivo = max(0.0, d - self.dist_seguridad)

        v_atractiva = self.kv * min(error_atractivo, k_r)
        w_atractiva = -self.kw * np.sin(theta_e)

        if d < self.dist_seguridad and d > 0:

            theta_repulsiva = np.arctan2(yr - yo, xr - xo)
            theta_rep_error = np.arctan2(np.sin(theta_repulsiva - theta_r), np.cos(theta_repulsiva - theta_r))

            fuerza_repulsiva = k_rep * (self.dist_seguridad - d)
            v_repulsiva = fuerza_repulsiva
            w_repulsiva = self.kw * np.sin(theta_rep_error)

        else:

            v_repulsiva = 0.0
            w_repulsiva = 0.0

        if d >= self.dist_seguridad:
            v = v_atractiva
            w = w_atractiva
        else:
            v = -v_repulsiva
            w = w_repulsiva

        v = np.clip(v, -self.v_max, self.v_max)
        w = np.clip(w, -self.w_max, self.w_max)

        return v, w

    # ======================================================
    # ATACANTE: meta = protegido, obstáculo = defensor
    # ======================================================

    def control_potencial_atacante(self, xr, yr, theta_r,
                                   protegido_x, protegido_y,
                                   defensor_x, defensor_y,
                                   k_r, k_rep, k_rep_obs, k_tang=1.0):
        """
        k_r       : saturación del error de distancia del campo atractivo (m)
        k_rep     : ganancia de retroceso si invade la distancia de seguridad
                    del protegido
        k_rep_obs : ganancia de repulsión del defensor (obstáculo)
        k_tang    : peso de la componente tangencial al rodear al defensor
        """
        # 1) Campo atractivo hacia el protegido (radial, con signo)
        dx = protegido_x - xr
        dy = protegido_y - yr
        d = np.hypot(dx, dy)

        if d < 1e-6:
            return 0.0, 0.0

        gx, gy = dx / d, dy / d

        if d >= self.dist_seguridad:
            f_att = min(d - self.dist_seguridad, k_r)
        else:
            # demasiado cerca del protegido: retrocede
            f_att = -k_rep * (self.dist_seguridad - d)

        fx = f_att * gx
        fy = f_att * gy

        # 2) Repulsión del defensor (obstáculo)
        f_ox, f_oy = self._repulsion(
            xr, yr, defensor_x, defensor_y,
            self.dist_repulsion, k_rep_obs, gx, gy, k_tang)

        fx += f_ox
        fy += f_oy

        # 3) Campo resultante -> (v, w)
        return self._fuerza_a_velocidades(fx, fy, theta_r)

    # ======================================================
    # DEFENSOR: meta = punto medio, obstáculos = protegido y atacante
    # ======================================================

    def control_potencial_defensor(self, xr, yr, theta_r,
                                   objetivo_x, objetivo_y,
                                   protegido_x, protegido_y,
                                   atacante_x, atacante_y,
                                   kr, krep, k_tang=0.6, tol_meta=0.05):

        # 1) Campo atractivo hacia el punto medio
        dx_obj = objetivo_x - xr
        dy_obj = objetivo_y - yr
        d_obj = np.hypot(dx_obj, dy_obj)

        if d_obj > 1e-6:
            gx, gy = dx_obj / d_obj, dy_obj / d_obj
        else:
            gx, gy = 0.0, 0.0

        magnitud_atractiva = kr * min(d_obj, 1.0)
        f_ax = magnitud_atractiva * gx
        f_ay = magnitud_atractiva * gy

        # 2) Radio de repulsión dinámico: la meta (a d_ap/2 de cada uno)
        #    debe quedar fuera de ambos radios de repulsión
        d_ap = np.hypot(atacante_x - protegido_x, atacante_y - protegido_y)
        d0 = min(self.dist_repulsion, 0.4 * d_ap)

        f_px, f_py = self._repulsion(xr, yr, protegido_x, protegido_y,
                                     d0, krep, gx, gy, k_tang)
        f_qx, f_qy = self._repulsion(xr, yr, atacante_x, atacante_y,
                                     d0, krep, gx, gy, k_tang)

        # 3) Campo resultante
        fx = f_ax + f_px + f_qx
        fy = f_ay + f_py + f_qy

        # Zona muerta: en la meta y sin repulsión significativa, quedarse quieto
        if d_obj < tol_meta and np.hypot(fx, fy) < 0.05:
            return 0.0, 0.0

        return self._fuerza_a_velocidades(fx, fy, theta_r)