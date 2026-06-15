# xbox_dife.py
import socket
import pygame
import time
import sys

# ======================================================
# CONFIGURACIÓN UDP
# ======================================================
# Esta debe ser la IP fija que asignaremos a la ESP32
UDP_IP = "192.168.0.100"
UDP_PORT = 12345

# ======================================================
# PARÁMETROS DEL ROBOT DIFERENCIAL
# ======================================================
R = 0.05
L = 0.20

# ======================================================
# VELOCIDADES MÁXIMAS
# ======================================================
V_MAX = 0.5
W_MAX = 1.0

# ======================================================
# CONFIGURACIÓN DEL CONTROL XBOX
# ======================================================
AXIS_V = 1
AXIS_W = 3

DEADZONE = 0.12
SEND_PERIOD = 0.10

SIGNO_V = 1.0
SIGNO_W = 1.0


def aplicar_deadzone(valor, zona):
    if abs(valor) < zona:
        return 0.0
    return valor


def limitar(valor, minimo, maximo):
    return max(min(valor, maximo), minimo)


def calcular_velocidades_ruedas(v, w):
    """
    Cinemática diferencial:

    v_r = (v + w * L/2) / R
    v_l = (v - w * L/2) / R

    Salida en rad/s.
    """

    v_r = (v + (w * L / 2.0)) / R
    v_l = (v - (w * L / 2.0)) / R

    return v_r, v_l


def enviar_velocidades(sock, v_r, v_l):
    mensaje = f"{v_r:.3f},{v_l:.3f}"
    sock.sendto(mensaje.encode(), (UDP_IP, UDP_PORT))
    print(f"Enviado -> derecha: {v_r:+.3f} rad/s | izquierda: {v_l:+.3f} rad/s")


def detener_robot(sock):
    sock.sendto(b"0.000,0.000", (UDP_IP, UDP_PORT))
    print("Robot detenido: 0,0 enviado")


def main():
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No se detectó ningún control Xbox.")
        print("Conecta el control y vuelve a ejecutar el programa.")
        sys.exit(1)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("=====================================")
    print(" Control diferencial con Xbox")
    print("=====================================")
    print(f"Control detectado: {joystick.get_name()}")
    print(f"IP fija de la ESP32: {UDP_IP}")
    print(f"Puerto UDP: {UDP_PORT}")
    print()
    print("Stick izquierdo Y: avanzar / retroceder")
    print("Stick derecho X: girar")
    print("Cierra la ventana o presiona Ctrl+C para detener.")
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    ultimo_envio = 0.0

    try:
        running = True

        while running:
            pygame.event.pump()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            raw_v = -joystick.get_axis(AXIS_V)
            raw_w = joystick.get_axis(AXIS_W)

            raw_v = aplicar_deadzone(raw_v, DEADZONE)
            raw_w = aplicar_deadzone(raw_w, DEADZONE)

            v = SIGNO_V * raw_v * V_MAX
            w = SIGNO_W * raw_w * W_MAX

            v = limitar(v, -V_MAX, V_MAX)
            w = limitar(w, -W_MAX, W_MAX)

            v_r, v_l = calcular_velocidades_ruedas(v, w)

            ahora = time.time()

            if ahora - ultimo_envio >= SEND_PERIOD:
                enviar_velocidades(sock, v_r, v_l)
                ultimo_envio = ahora

            pygame.time.wait(10)

    except KeyboardInterrupt:
        print("\nInterrupción por teclado.")

    finally:
        detener_robot(sock)
        sock.close()
        pygame.quit()


if __name__ == "__main__":
    main()
