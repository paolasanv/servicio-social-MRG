import socket

UDP_IP = "10.101.77.185"
UDP_PORT = 12345

# Crear el socket UDP.
sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

# Escuchar en el puerto configurado.
sock.bind((UDP_IP, UDP_PORT))

print(f"Esperando poses ArUco en el puerto {UDP_PORT}...")

try:
    while True:

        # Recibir un mensaje UDP.
        datos, direccion = sock.recvfrom(1024)

        # Convertir los bytes a texto.
        mensaje = datos.decode("utf-8")

        # Separar los campos recibidos.
        aruco_id, x, y, theta = mensaje.split(",")

        # Mostrar la información en la terminal.
        print(
            f"ArUco {aruco_id} | "
            f"x: {float(x):+.3f} m | "
            f"y: {float(y):+.3f} m | "
            f"theta: {float(theta):+.3f} rad"
        )

except KeyboardInterrupt:
    print("\nReceptor detenido.")

finally:
    sock.close()
