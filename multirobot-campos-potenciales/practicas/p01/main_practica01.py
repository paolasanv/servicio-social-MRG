import socket
import time

from vision import Vision

# ==========================================
# CONFIGURACIÓN DE COMUNICACIÓN UDP
# ==========================================

UDP_IP = "10.101.77.185" #hostaname -I
UDP_PORT = 12345

SEND_PERIOD = 0.1  # Periodo de envío: 100 ms


# ==========================================
# COMUNICACIÓN UDP
# ==========================================

def enviar_pose(sock, aruco_id, x, y, theta):
    mensaje = f"{aruco_id},{x:.3f},{y:.3f},{theta:.3f}"

    print(f"Intentando enviar a {UDP_IP}:{UDP_PORT}: {mensaje}")

    try:
        enviados = sock.sendto(
            mensaje.encode("utf-8"),
            (UDP_IP, UDP_PORT)
        )
        print(f"Datagrama enviado: {enviados} bytes")
    except OSError as e:
        print(f"Error al enviar UDP: {e}")


# ==========================================
# PROGRAMA PRINCIPAL
# ==========================================

def ejecutar():

    vision = Vision()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tecla = ""
    ultimo_envio = 0.0

    try:
        # Inicialización de la cámara.
        vision.iniciar_camara()

        print("Envío de poses ArUco iniciado.")
        print("Presiona 'q' para finalizar.")

        while True:
            # Finalizar al presionar 'q'.
            if tecla == ord('q'):
                break

            # Obtención del frame y las poses detectadas.
            frame, poses, tecla = vision.obtener_poses()

            # Control del periodo de envío.
            tiempo_actual = time.time()

            if tiempo_actual - ultimo_envio >= SEND_PERIOD:

                # Recorrer los marcadores detectados.
                for aruco_id, pose in poses.items():
                    print("marcador detectado")
                    enviar_pose(sock,int(aruco_id),pose["x"],pose["y"],pose["theta"])

                ultimo_envio = tiempo_actual

    except KeyboardInterrupt:
        print("\nPrograma interrumpido.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Liberación de los recursos.
        vision.cerrar()
        sock.close()

        print("Comunicación UDP finalizada.")


if __name__ == "__main__":
    ejecutar()
