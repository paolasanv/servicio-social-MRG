import socket

# ==========================================
# CONFIGURACIÓN DE COMUNICACIÓN UDP
# ==========================================

UDP_IP = "192.168.0.100"  # Dirección IP del ESP32
UDP_PORT = 12345           # Puerto UDP del ESP32

TIMEOUT_RESPUESTA = 2.0    # Tiempo máximo de espera (segundos)


# ==========================================
# COMUNICACIÓN UDP
# ==========================================

def enviar_mensaje(sock, mensaje):

    try:
        # Convertir el mensaje a bytes y enviarlo mediante UDP.
        datos = mensaje.encode("utf-8")
        sock.sendto(datos, (UDP_IP, UDP_PORT))

        # Esperar la respuesta del ESP32.
        try:
            respuesta, direccion = sock.recvfrom(1024)

            respuesta = respuesta.decode("utf-8", errors="replace")

            print(f"Respuesta del ESP32: {respuesta}")
            print(f"Origen de la respuesta: {direccion[0]}:{direccion[1]}")

        except socket.timeout:
            print("No se recibió respuesta del ESP32.")

    except OSError as e:
        print(f"Error en la comunicación UDP: {e}")


# ==========================================
# PROGRAMA PRINCIPAL
# ==========================================

def ejecutar():

    # Crear el socket UDP.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Establecer el tiempo máximo de espera para recibir respuestas.
    sock.settimeout(TIMEOUT_RESPUESTA)

    print("====================================")
    print("       COMUNICACIÓN UDP")
    print("====================================")
    print(f"Destino: {UDP_IP}:{UDP_PORT}")
    print(f"Tiempo de espera: {TIMEOUT_RESPUESTA} segundos")
    print("Escribe cualquier mensaje para enviarlo.")
    print("Escribe 'q' para finalizar.")
    print("====================================")

    try:
        while True:

            # Solicitar un mensaje al usuario.
            mensaje = input("\nMensaje a enviar: ")

            # Finalizar si el usuario escribe 'q'.
            if mensaje.lower() == "q":
                break

            # Enviar el mensaje y esperar una respuesta.
            enviar_mensaje(sock, mensaje)

    except KeyboardInterrupt:
        print("\nPrograma interrumpido por el usuario.")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        sock.close()
        print("Comunicación UDP finalizada.")


if __name__ == "__main__":
    ejecutar()
