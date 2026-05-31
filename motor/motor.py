"""
CALIBRACION AUTOMATICA EMG30
Trabaja con: barrido_automatico_emg30_esp32.ino

Genera:
  - datos_crudos.csv
  - puntos_estables.csv
  - modelo_calibracion.txt
  - funcion_pwm_desde_omega.h
  - respuesta_temporal.png
  - calibracion_pwm_omega.png
  - calibracion_tension_equivalente_omega.png
"""

import re
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial
from serial.tools import list_ports

BAUDIOS = 115200
OMEGA_MIN_MOVIMIENTO = 0.30   # rad/s; umbral para detectar que ya giro
FASE_MEDICION = 1             # la ESP32 etiqueta como 1 los datos estables

COLS = [
    "tiempo_s", "step", "phase", "pwm", "pwm_pct", "dt_s",
    "delta_counts", "omega_raw", "omega_filt", "rpm", "total_counts"
]


def pedir_float(texto, default):
    valor = input(f"{texto} [{default}]: ").strip().replace(",", ".")
    return float(valor) if valor else default


def seguro(texto):
    return re.sub(r"[^A-Za-z0-9_-]+", "_", texto.strip()) or "ensayo"


def elegir_puerto():
    puertos = list(list_ports.comports())
    if not puertos:
        raise RuntimeError("No se encontraron puertos seriales.")
    print("\nPuertos encontrados:")
    for i, p in enumerate(puertos, 1):
        print(f"  [{i}] {p.device} - {p.description}")
    if len(puertos) == 1:
        print(f"Se usara automaticamente: {puertos[0].device}")
        return puertos[0].device
    while True:
        n = input("Selecciona el puerto: ").strip()
        if n.isdigit() and 1 <= int(n) <= len(puertos):
            return puertos[int(n) - 1].device


def adquirir(puerto, vfuente):
    filas = []
    print(f"\nAbriendo {puerto} a {BAUDIOS} baudios...")
    with serial.Serial(puerto, BAUDIOS, timeout=0.4) as ser:
        time.sleep(2.0)  # la ESP32 se reinicia al abrir el puerto
        ser.reset_input_buffer()

        print("\nAntes de iniciar:")
        print("  - Fija el motor o eleva el robot si tiene rueda.")
        print("  - Enciende la fuente del puente H.")
        print("  - No sujetes el eje ni la rueda.")
        input("\nPresiona ENTER para iniciar el barrido automatico...")
        ser.write(b"START\n")
        ser.flush()

        ultimo_step = None
        try:
            while True:
                linea = ser.readline().decode("utf-8", errors="ignore").strip()
                if not linea:
                    continue
                if linea.startswith("STATUS,STEP"):
                    p = linea.split(",")
                    step, pwm = int(p[2]), int(p[3])
                    if step != ultimo_step:
                        print(f"Punto {step + 1:02d}: PWM = {pwm:4d}")
                        ultimo_step = step
                    continue
                if linea.startswith("STATUS,END"):
                    print("\nBarrido terminado:", linea)
                    break
                if not linea.startswith("DATA,"):
                    continue

                p = linea.split(",")[1:]
                if len(p) != len(COLS):
                    continue
                nums = [float(x) for x in p]
                fila = dict(zip(COLS, nums))
                fila["step"] = int(fila["step"])
                fila["phase"] = int(fila["phase"])
                fila["pwm"] = int(fila["pwm"])
                fila["tension_equivalente_V"] = vfuente * fila["pwm"] / 255.0
                filas.append(fila)

        except KeyboardInterrupt:
            print("\nInterrupcion de emergencia: deteniendo motor.")
            ser.write(b"STOP\n")
            ser.flush()
        finally:
            try:
                ser.write(b"STOP\n")
                ser.flush()
            except serial.SerialException:
                pass

    return pd.DataFrame(filas)


def ajuste(puntos, sentido):
    if sentido > 0:
        uso = puntos[(puntos["pwm"] > 0) &
                     (puntos["omega_media"] >= OMEGA_MIN_MOVIMIENTO)]
        pwm_min = int(uso["pwm"].min()) if not uso.empty else None
    else:
        uso = puntos[(puntos["pwm"] < 0) &
                     (puntos["omega_media"] <= -OMEGA_MIN_MOVIMIENTO)]
        pwm_min = int(uso["pwm"].max()) if not uso.empty else None

    if len(uso) < 3:
        return None

    a, b = np.polyfit(uso["pwm"], uso["omega_media"], 1)
    pred = a * uso["pwm"].to_numpy() + b
    real = uso["omega_media"].to_numpy()
    ss_res = np.sum((real - pred) ** 2)
    ss_tot = np.sum((real - np.mean(real)) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return {"a": float(a), "b": float(b), "r2": float(r2), "pwm_min": pwm_min}


def guardar_modelo(carpeta, motor, condicion, vfuente, pos, neg):
    ruta = carpeta / "modelo_calibracion.txt"
    with ruta.open("w", encoding="utf-8") as f:
        f.write("CALIBRACION PWM - VELOCIDAD ANGULAR EMG30\n")
        f.write("=========================================\n")
        f.write(f"Motor/rueda: {motor}\nCondicion: {condicion}\n")
        f.write(f"Voltaje fuente puente H: {vfuente:.3f} V\n")
        f.write("PWM firmado en [-255,255]; omega en rad/s.\n\n")
        f.write("El voltaje equivalente Vfuente*PWM/255 no es una medicion "
                "del voltaje terminal del motor.\n\n")
        for nombre, modelo in [("POSITIVO", pos), ("NEGATIVO", neg)]:
            f.write(f"GIRO {nombre}\n")
            if modelo is None:
                f.write("No fue posible ajustar este sentido.\n\n")
                continue
            a, b = modelo["a"], modelo["b"]
            f.write(f"omega = {a:.10f} * PWM + ({b:.10f})\n")
            f.write(f"R2 = {modelo['r2']:.6f}\n")
            f.write(f"PWM minimo detectado = {modelo['pwm_min']}\n")
            f.write(f"PWM = (omega_d - ({b:.10f})) / {a:.10f}\n\n")

    if pos is None or neg is None:
        return

    h = f"""/*
 Funcion generada por la caracterizacion del EMG30.
 Entrada: omegaDeseada en rad/s. Salida: PWM firmado.
*/
const float A_POS = {pos['a']:.10f}f;
const float B_POS = {pos['b']:.10f}f;
const float A_NEG = {neg['a']:.10f}f;
const float B_NEG = {neg['b']:.10f}f;
const int PWM_MIN_POS = {pos['pwm_min']};
const int PWM_MIN_NEG = {neg['pwm_min']};

int16_t pwmDesdeOmega(float omegaDeseada) {{
  if (fabs(omegaDeseada) <= 0.05f) return 0;
  float pwm;
  if (omegaDeseada > 0.0f) {{
    pwm = (omegaDeseada - B_POS) / A_POS;
    if (pwm < PWM_MIN_POS) pwm = PWM_MIN_POS;
  }} else {{
    pwm = (omegaDeseada - B_NEG) / A_NEG;
    if (pwm > PWM_MIN_NEG) pwm = PWM_MIN_NEG;
  }}
  return (int16_t)round(constrain(pwm, -255.0f, 255.0f));
}}
"""
    (carpeta / "funcion_pwm_desde_omega.h").write_text(h, encoding="utf-8")


def graficas(df, puntos, pos, neg, carpeta):
    plt.figure(figsize=(10, 5))
    plt.plot(df["tiempo_s"], df["omega_filt"], label="Omega filtrada")
    usados = df[df["phase"] == FASE_MEDICION]
    plt.scatter(usados["tiempo_s"], usados["omega_filt"], s=7,
                label="Muestras de medicion")
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Velocidad angular (rad/s)")
    plt.title("Respuesta temporal del accionamiento EMG30")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(carpeta / "respuesta_temporal.png", dpi=300)
    plt.close()

    plt.figure(figsize=(9, 5))
    plt.errorbar(puntos["pwm"], puntos["omega_media"],
                 yerr=puntos["omega_std"], fmt="o", capsize=3,
                 label="Promedio medido")
    if pos:
        x = np.linspace(pos["pwm_min"], 255, 100)
        plt.plot(x, pos["a"] * x + pos["b"],
                 label=f"Ajuste positivo, R2={pos['r2']:.4f}")
    if neg:
        x = np.linspace(-255, neg["pwm_min"], 100)
        plt.plot(x, neg["a"] * x + neg["b"],
                 label=f"Ajuste negativo, R2={neg['r2']:.4f}")
    plt.xlabel("PWM firmado [-255, 255]")
    plt.ylabel("Velocidad angular (rad/s)")
    plt.title("Calibracion PWM - velocidad angular")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(carpeta / "calibracion_pwm_omega.png", dpi=300)
    plt.close()

    plt.figure(figsize=(9, 5))
    plt.errorbar(puntos["tension_equivalente_V"], puntos["omega_media"],
                 yerr=puntos["omega_std"], fmt="o", capsize=3)
    plt.xlabel("Vfuente * PWM / 255 (V), no voltaje terminal real")
    plt.ylabel("Velocidad angular (rad/s)")
    plt.title("Tension equivalente de mando - velocidad angular")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(carpeta / "calibracion_tension_equivalente_omega.png", dpi=300)
    plt.close()


def main():
    print("\nCALIBRACION AUTOMATICA EMG30: PWM -> OMEGA")
    print("Este ensayo no utiliza el potenciometro.\n")
    motor = input("Identificador del motor [M1]: ").strip() or "M1"
    condicion = input("Condicion [rueda_elevada]: ").strip() or "rueda_elevada"
    vfuente = pedir_float("Voltaje de la fuente del puente H (V)", 12.0)

    carpeta = Path(f"resultados_{seguro(motor)}_{seguro(condicion)}_"
                   f"{datetime.now():%Y%m%d_%H%M%S}")
    carpeta.mkdir()

    try:
        df = adquirir(elegir_puerto(), vfuente)
    except (serial.SerialException, PermissionError) as e:
        print("\nNo se pudo abrir el puerto serial:", e)
        print("Cierra Arduino IDE, Serial Plotter, Excel y otros procesos Python.")
        return

    if df.empty:
        print("No se recibieron datos.")
        return

    df.to_csv(carpeta / "datos_crudos.csv", index=False)

    med = df[df["phase"] == FASE_MEDICION]
    puntos = med.groupby("pwm", as_index=False).agg(
        pwm_pct=("pwm_pct", "mean"),
        tension_equivalente_V=("tension_equivalente_V", "mean"),
        muestras=("omega_raw", "size"),
        omega_media=("omega_raw", "mean"),
        omega_std=("omega_raw", "std"),
    )
    puntos["omega_std"] = puntos["omega_std"].fillna(0.0)
    puntos["rpm_media"] = puntos["omega_media"] * 60.0 / (2.0 * np.pi)
    puntos.to_csv(carpeta / "puntos_estables.csv", index=False)

    if (puntos[puntos["pwm"] >= 120]["omega_media"].mean() < 0 or
            puntos[puntos["pwm"] <= -120]["omega_media"].mean() > 0):
        print("\nEl signo del encoder esta invertido. Cambia SIGNO_ENCODER a -1")
        print("en Arduino y repite el ensayo.")
        return

    pos = ajuste(puntos, 1)
    neg = ajuste(puntos, -1)
    guardar_modelo(carpeta, motor, condicion, vfuente, pos, neg)
    graficas(df, puntos, pos, neg, carpeta)

    print(f"\nResultados guardados en: {carpeta.resolve()}")
    for nombre, m in [("positivo", pos), ("negativo", neg)]:
        if m:
            print(f"\nGiro {nombre}:")
            print(f"  omega = {m['a']:.8f} * PWM + ({m['b']:.8f})")
            print(f"  R2 = {m['r2']:.6f}; PWM minimo = {m['pwm_min']}")
            print(f"  PWM = (omega_d - ({m['b']:.8f})) / {m['a']:.8f}")
    if pos and neg:
        print("\nFuncion lista para copiar al controlador:")
        print("  funcion_pwm_desde_omega.h")


if __name__ == "__main__":
    main()
