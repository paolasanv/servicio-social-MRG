# robot_movil.py


import socket

from src.config import UDP_PORT, R, L

class Robot:

    def __init__(self, ip):
        self.ip = ip
        self.port = UDP_PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def calcular_velocidades_ruedas(self, v, w):
        vr = (v + w * L / 2) / R
        vl = (v - w * L / 2) / R
        return vr, vl

    def enviar_velocidades(self, vr, vl):
        mensaje = f"{vr:.3f},{vl:.3f}"
        self.sock.sendto(mensaje.encode(), (self.ip, self.port))

    def detener(self):
        self.enviar_velocidades(0, 0)

    def cerrar(self):
        self.sock.close()