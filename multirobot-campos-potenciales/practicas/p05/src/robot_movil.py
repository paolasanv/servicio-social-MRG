# robot_movil.py

# ======================================================
# IMPORTACIÓN DE LIBRERÍAS
# ======================================================

# Librería utilizada para establecer la comunicación UDP
# entre el programa de control y el robot.
import socket

# Librería utilizada para realizar operaciones matemáticas
# y trabajar con vectores, ángulos y funciones trigonométricas.
import numpy as np

# Importación de los parámetros de configuración del robot.
#
# UDP_PORT: puerto utilizado para la comunicación UDP.
# R: radio de las ruedas.
# L: distancia entre las ruedas.
from src.config import UDP_PORT, R, L

# ======================================================
# CLASE ROBOT
# ======================================================

class Robot:
    """
    Representa un robot móvil de accionamiento diferencial.

    La clase permite:
    - Configurar la comunicación con el robot mediante UDP.
    - Convertir velocidades lineales y angulares en velocidades
      de las ruedas.
    - Enviar las velocidades de las ruedas.
    - Detener el robot.
    - Aplicar diferentes estrategias de control mediante
      campos potenciales.
    """

    def __init__(self, ip, kv, kw):
        """
        Inicializa un robot.

        Parámetros:
            ip: dirección IP del robot.
            kv: ganancia asociada a la velocidad lineal.
            kw: ganancia asociada a la velocidad angular.
        """

        # Dirección IP del robot al que se enviarán los comandos.
        self.ip = ip

        # Ganancia utilizada para calcular la velocidad lineal.
        self.kv = kv

        # Ganancia utilizada para calcular la velocidad angular.
        self.kw = kw

        # Velocidad lineal máxima permitida.
        self.v_max = 3

        # Velocidad angular máxima permitida.
        self.w_max = 1

        # Distancia de seguridad utilizada alrededor de un robot
        # en los campos potenciales.
        self.dist_seguridad = 0.55

        # Distancia máxima a partir de la cual se considera la
        # influencia repulsiva de un obstáculo.
        self.dist_repulsion = 0.55

        # Puerto UDP utilizado para enviar las velocidades.
        self.port = UDP_PORT

        # Creación del socket UDP.
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    # ======================================================
    # CINEMÁTICA DEL ROBOT
    # ======================================================

    def calcular_velocidades_ruedas(self, v, w):
        """
        Convierte la velocidad lineal y angular del robot
        en velocidades angulares para las ruedas.

        El robot utiliza una configuración diferencial.

        Parámetros:
            v: velocidad lineal del robot.
            w: velocidad angular del robot.

        Retorna:
            vr: velocidad angular de la rueda derecha.
            vl: velocidad angular de la rueda izquierda.
        """

        # Velocidad de la rueda derecha.
        vr = (v + w * L / 2) / R

        # Velocidad de la rueda izquierda.
        vl = (v - w * L / 2) / R

        return vr, vl

    # ======================================================
    # COMUNICACIÓN CON EL ROBOT
    # ======================================================

    def enviar_velocidades(self, vr, vl):
        """
        Envía las velocidades de las ruedas al robot mediante UDP.

        Parámetros:
            vr: velocidad angular de la rueda derecha.
            vl: velocidad angular de la rueda izquierda.
        """

        # Construcción del mensaje que será enviado.
        #
        # Las velocidades se representan con tres cifras decimales
        # y se separan mediante una coma.
        mensaje = f"{vr:.3f},{vl:.3f}"

        # Envío del mensaje mediante UDP.
        self.sock.sendto(mensaje.encode(),(self.ip, self.port))

        # Impresión de las velocidades enviadas para
        # facilitar la supervisión del sistema.
        print(
            f"Enviado -> derecha: {vr:+.3f} rad/s | "
            f"izquierda: {vl:+.3f} rad/s"
        )

    def detener(self):
        """
        Detiene el robot enviando velocidad cero a ambas ruedas.
        """

        self.enviar_velocidades(0, 0)

    def cerrar(self):
        """
        Cierra el socket utilizado para la comunicación UDP.
        """

        self.sock.close()

    # ======================================================
    # CAMPOS POTENCIALES
    # ======================================================

    def control_potencial_atrac_rep(self, xr, yr, theta_r, xo, yo, k_r, k_rep):
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
