# cine_dife_continuo.py
import socket
import time

# ======================================================
# CONFIGURACIÓN UDP
# ======================================================
UDP_IP = "192.168.0.100"   # Cambia esto por la IP real de tu ESP32
UDP_PORT = 12345

# ======================================================
# PARÁMETROS DEL ROBOT DIFERENCIAL
# ======================================================
R = 0.05   # Radio de rueda [m]
L = 0.20   # Distancia entre ruedas [m]

# ======================================================
# SEGURIDAD
# ======================================================
PERIODO_ENVIO = 0.05       # 50 ms -> 20 Hz
OMEGA_MAX = 12.0           # rad/s, igual al límite sugerido en ESP32


def limitar(valor, minimo, maximo):
    return max(min(valor, maximo), minimo)


def cinematica_diferencial(v, w):
    """
    Convierte velocidad lineal y angular del robot a velocidades angulares de rueda.

    Entrada:
        v: velocidad lineal del robot [m/s]
        w: velocidad angular del robot [rad/s]

    Salida:
        omega_r: velocidad angular rueda derecha [rad/s]
        omega_l: velocidad angular rueda izquierda [rad/s]
    """

    omega_r = (2.0 * v + w * L) / (2.0 * R)
    omega_l = (2.0 * v - w * L) / (2.0 * R)

    omega_r = limitar(omega_r, -OMEGA_MAX, OMEGA_MAX)
    omega_l = limitar(omega_l, -OMEGA_MAX, OMEGA_MAX)

    return omega_r, omega_l


def enviar(sock, omega_r, omega_l):
    """
    Envía velocidades angulares a la ESP32.

    Formato:
        motorA,motorB

    IMPORTANTE:
    Aquí se asume:
        Motor A = rueda derecha
        Motor B = rueda izquierda

    Si tu conexión física es diferente, invierte el orden.
    """

    data = f"{omega_r:.3f},{omega_l:.3f}"
    sock.sendto(data.encode(), (UDP_IP, UDP_PORT))
    print(f"Enviado: derecha = {omega_r:.3f} rad/s | izquierda = {omega_l:.3f} rad/s")


def detener(sock):
    sock.sendto("0,0".encode(), (UDP_IP, UDP_PORT))
    print("Comando de paro enviado: 0,0")


if __name__ == "__main__":
    print("====================================")
    print(" Prueba continua robot diferencial")
    print("====================================")
    print("Recomendado para empezar:")
    print("v = 0.15 a 0.30 m/s")
    print("w = 0.0 rad/s")
    print()

    v = float(input("Ingresa velocidad lineal v [m/s]: "))
    w = float(input("Ingresa velocidad angular w [rad/s]: "))

    omega_r, omega_l = cinematica_diferencial(v, w)

    print()
    print("Velocidades calculadas:")
    print(f"Rueda derecha:   {omega_r:.3f} rad/s")
    print(f"Rueda izquierda: {omega_l:.3f} rad/s")
    print()
    print("Enviando continuamente. Presiona Ctrl + C para detener.")
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            enviar(sock, omega_r, omega_l)
            time.sleep(PERIODO_ENVIO)

    except KeyboardInterrupt:
        print("\nInterrupción del usuario.")

    finally:
        detener(sock)
        time.sleep(0.1)
        detener(sock)
        sock.close()
