# config.py



# ======================================================
# CONFIGURACIÓN UDP
# ======================================================

UDP_PORT = 12345


# ======================================================
# PARÁMETROS DEL ROBOT DIFERENCIAL
# ======================================================

R = 0.05          # Radio de la rueda [m]
L = 0.20          # Distancia entre ruedas [m]


# ======================================================
# VELOCIDADES MÁXIMAS
# ======================================================

V_MAX = 0.5       # Velocidad lineal máxima [m/s]
W_MAX = 1.0       # Velocidad angular máxima [rad/s]


# ======================================================
# CONFIGURACIÓN DEL CONTROL XBOX
# ======================================================

AXIS_V = 1        # Stick izquierdo Y
AXIS_W = 3        # Stick derecho X

DEADZONE = 0.12

SIGNO_V = 1.0
SIGNO_W = 1.0


# ======================================================
# TEMPORIZACIÓN
# ======================================================

SEND_PERIOD = 0.1    # Tiempo entre envíos UDP [s] (original 0.1)
