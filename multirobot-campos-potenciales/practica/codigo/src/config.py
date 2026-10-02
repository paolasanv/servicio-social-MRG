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

V_MAX = 1.0       # Velocidad lineal máxima [m/s]
W_MAX = 1.0       # Velocidad angular máxima [rad/s]


# ======================================================
# TEMPORIZACIÓN
# ======================================================

SEND_PERIOD = 0.1    # Tiempo entre envíos UDP [s] (original 0.1)
