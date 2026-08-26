import csv
import json
import socket
import threading
import time
from datetime import datetime

# ======================================================
# CONFIGURACION UDP
# ======================================================
UDP_IP = "192.168.0.100"   # ESP32
UDP_PORT = 12345            # comandos -> ESP32
TELEMETRY_PORT = 12346      # telemetria <- ESP32

# ======================================================
# PARAMETROS DEL ROBOT DIFERENCIAL
# ======================================================
R = 0.05   # Radio de rueda [m]
L = 0.20   # Distancia entre ruedas [m]

# ======================================================
# SEGURIDAD / TIEMPOS
# ======================================================
PERIODO_ENVIO = 0.05       # 20 Hz
OMEGA_MAX = 12.0           # rad/s, igual al ESP32


def limitar(valor, minimo, maximo):
    return max(min(valor, maximo), minimo)


def cinematica_diferencial(v, w):
    omega_r = (2.0 * v + w * L) / (2.0 * R)
    omega_l = (2.0 * v - w * L) / (2.0 * R)

    omega_r = limitar(omega_r, -OMEGA_MAX, OMEGA_MAX)
    omega_l = limitar(omega_l, -OMEGA_MAX, OMEGA_MAX)
    return omega_r, omega_l


def enviar(sock, omega_r, omega_l):
    data = f"{omega_r:.3f},{omega_l:.3f}"
    sock.sendto(data.encode(), (UDP_IP, UDP_PORT))


def detener(sock):
    sock.sendto(b"0,0", (UDP_IP, UDP_PORT))


def receptor_telemetria(stop_event):
    sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_rx.bind(("", TELEMETRY_PORT))
    sock_rx.settimeout(0.25)

    nombre_csv = datetime.now().strftime("telemetria_robot_%Y%m%d_%H%M%S.csv")
    campos = [
        "host_time", "esp_ms", "rssi", "comm",
        "A_sp_rpm", "A_pv_rpm", "A_sp_rad", "A_pv_rad", "A_err_rpm",
        "A_ff", "A_pid", "A_pwm", "A_mode",
        "B_sp_rpm", "B_pv_rpm", "B_sp_rad", "B_pv_rad", "B_err_rpm",
        "B_ff", "B_pid", "B_pwm", "B_mode",
    ]

    print(f"Telemetria escuchando en UDP {TELEMETRY_PORT}")
    print(f"Guardando datos en: {nombre_csv}")
    print()

    with open(nombre_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=campos)
        writer.writeheader()

        while not stop_event.is_set():
            try:
                packet, _ = sock_rx.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                d = json.loads(packet.decode("utf-8"))
                A = d["A"]
                B = d["B"]
            except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError):
                continue

            fila = {
                "host_time": datetime.now().isoformat(timespec="milliseconds"),
                "esp_ms": d.get("t_ms"),
                "rssi": d.get("rssi"),
                "comm": d.get("comm"),
                "A_sp_rpm": A.get("sp_rpm"),
                "A_pv_rpm": A.get("pv_rpm"),
                "A_sp_rad": A.get("sp_rad"),
                "A_pv_rad": A.get("pv_rad"),
                "A_err_rpm": A.get("err_rpm"),
                "A_ff": A.get("ff"),
                "A_pid": A.get("pid"),
                "A_pwm": A.get("pwm"),
                "A_mode": A.get("mode"),
                "B_sp_rpm": B.get("sp_rpm"),
                "B_pv_rpm": B.get("pv_rpm"),
                "B_sp_rad": B.get("sp_rad"),
                "B_pv_rad": B.get("pv_rad"),
                "B_err_rpm": B.get("err_rpm"),
                "B_ff": B.get("ff"),
                "B_pid": B.get("pid"),
                "B_pwm": B.get("pwm"),
                "B_mode": B.get("mode"),
            }
            writer.writerow(fila)
            f.flush()

            print(
                f"A SP={A['sp_rpm']:7.2f} PV={A['pv_rpm']:7.2f} "
                f"ERR={A['err_rpm']:7.2f} FF={A['ff']:6.1f} "
                f"PID={A['pid']:6.1f} PWM={A['pwm']:6.1f} {A['mode']:>5} | "
                f"B SP={B['sp_rpm']:7.2f} PV={B['pv_rpm']:7.2f} "
                f"ERR={B['err_rpm']:7.2f} FF={B['ff']:6.1f} "
                f"PID={B['pid']:6.1f} PWM={B['pwm']:6.1f} {B['mode']:>5} | "
                f"RSSI={d.get('rssi')} dBm"
            )

    sock_rx.close()


if __name__ == "__main__":
    print("========================================")
    print(" Robot diferencial + telemetria UDP")
    print("========================================")
    print("Ejemplo inicial: v=0.20 m/s, w=0 rad/s")
    print()

    v = float(input("Ingresa velocidad lineal v [m/s]: "))
    w = float(input("Ingresa velocidad angular w [rad/s]: "))

    omega_r, omega_l = cinematica_diferencial(v, w)

    print()
    print(f"Rueda derecha:   {omega_r:.3f} rad/s")
    print(f"Rueda izquierda: {omega_l:.3f} rad/s")
    print("Ctrl+C para detener.")
    print()

    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop_event = threading.Event()
    hilo_rx = threading.Thread(
        target=receptor_telemetria,
        args=(stop_event,),
        daemon=True,
    )
    hilo_rx.start()

    try:
        while True:
            enviar(sock_tx, omega_r, omega_l)
            time.sleep(PERIODO_ENVIO)

    except KeyboardInterrupt:
        print("\nDeteniendo robot...")

    finally:
        detener(sock_tx)
        time.sleep(0.10)
        detener(sock_tx)
        stop_event.set()
        hilo_rx.join(timeout=0.5)
        sock_tx.close()
        print("Robot detenido y archivo CSV cerrado.")
