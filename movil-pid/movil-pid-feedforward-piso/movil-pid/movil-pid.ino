/*
 * ESP32 + L298N + EMG30
 * Control de velocidad por rueda:
 *
 *     PWM = Feedforward_calibrado(omega_d) + PID(SP - PV)
 *
 * + PWM de arranque para vencer friccion estatica
 * + PWM minimo de marcha
 * + no inversion de sentido por correccion PID
 * + saturacion +/-255 y anti-windup
 * + timeout UDP
 *
 * UDP recibe rad/s:
 *   velA,velB
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "motor-PID.h"

// ======================================================
// WIFI
// ======================================================
const char* ssid = "TP-Link_8960";
const char* password = "53899736";

IPAddress local_IP(192, 168, 0, 100);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// ======================================================
// UDP / SEGURIDAD
// ======================================================
WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[256];

// Telemetria UDP hacia la computadora que envia comandos.
const unsigned int telemetryUdpPort = 12346;
IPAddress ultimoClienteIP;
bool clienteUDPConocido = false;
const unsigned long TELEMETRY_PERIOD_MS = 100;  // 10 Hz

unsigned long ultimoPaqueteUDP = 0;
const unsigned long TIMEOUT_UDP = 750;
bool comunicacionActiva = false;

const float MAX_WHEEL_RAD_S = 12.0f;

// ======================================================
// PINES L298N
// ======================================================
// Motor A
const int IN1 = 32;
const int IN2 = 33;
const int ENA = 25;

// Motor B
const int IN3 = 18;
const int IN4 = 19;
const int ENB = 5;

// ======================================================
// ENCODERS EMG30
// ======================================================
const int SENSOR_A_MA = 26;
const int SENSOR_B_MA = 27;

const int SENSOR_A_MB = 17;
const int SENSOR_B_MB = 16;

// ======================================================
// PID
// ======================================================
MotorPID motorA(
    SENSOR_A_MA, SENSOR_B_MA,
    IN2, IN1, ENA,
    0.5, 0.20, 0
);

MotorPID motorB(
    SENSOR_A_MB, SENSOR_B_MB,
    IN3, IN4, ENB,
    0.5, 0.20, 0
);

// ======================================================
// FEEDFORWARD EXPERIMENTAL EN PISO - POR RUEDA
// ======================================================
// Ajustado con telemetria en piso a referencias de 2, 4 y 6 rad/s.
// El modelo que usa MotorPID es:
//     omega = a*PWM + b
// y la clase despeja internamente:
//     PWM_FF = (omega_d - b)/a
//
// GIRO POSITIVO - MOTOR A
// omega = 0.1537745505*PWM - 12.5123396817
// R2 ~= 0.984884
const double A_A_POS = 0.1537745505;
const double A_B_POS = -12.5123396817;
//
// GIRO POSITIVO - MOTOR B
// omega = 0.1118491640*PWM - 9.6242878099
// R2 ~= 0.994849
const double B_A_POS = 0.1118491640;
const double B_B_POS = -9.6242878099;
//
// GIRO NEGATIVO - MODELO PROVISIONAL EN PISO, POR RUEDA
// Identificado con telemetria de -0.20 y -0.30 m/s.
// Se usaron muestras RUN posteriores a 5 s y se excluyo
// el tramo final de frenado de la prueba -0.20 m/s.
//
// MOTOR A:
// omega = 0.0955451842*PWM + 4.8928662071
// R2 ~= 0.7609
const double A_A_NEG = 0.0955451842;
const double A_B_NEG = 4.8928662071;
//
// MOTOR B:
// omega = 0.0844906226*PWM + 5.2012861454
// R2 ~= 0.7453
const double B_A_NEG = 0.0844906226;
const double B_B_NEG = 5.2012861454;

// ======================================================
// WIFI
// ======================================================
void conectarWiFi() {
    WiFi.mode(WIFI_STA);

    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("Error configurando IP fija.");
    }

    WiFi.begin(ssid, password);
    Serial.print("Conectando a WiFi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi conectado.");
    Serial.print("IP ESP32: ");
    Serial.println(WiFi.localIP());
}

// ======================================================
// SEGURIDAD
// ======================================================
void detenerRobot() {
    motorA.detener();
    motorB.detener();
}

float limitarReferencia(float value) {
    if (value > MAX_WHEEL_RAD_S) return MAX_WHEEL_RAD_S;
    if (value < -MAX_WHEEL_RAD_S) return -MAX_WHEEL_RAD_S;
    return value;
}

// ======================================================
// UDP
// ======================================================
void recibirUDP() {
    const int packetSize = Udp.parsePacket();
    if (!packetSize) return;

    // Guardamos la IP del equipo que esta mandando los comandos.
    // La telemetria se devolvera a esa misma computadora por el puerto 12346.
    ultimoClienteIP = Udp.remoteIP();
    clienteUDPConocido = true;

    const int len = Udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len <= 0) return;

    incomingPacket[len] = '\0';

    float velA = 0.0f;
    float velB = 0.0f;

    const int datos = sscanf(incomingPacket, " %f , %f", &velA, &velB);

    if (datos != 2 || !isfinite(velA) || !isfinite(velB)) {
        Serial.print("Formato UDP invalido: ");
        Serial.println(incomingPacket);
        return;
    }

    velA = limitarReferencia(velA);
    velB = limitarReferencia(velB);

    // rad/s -> RPM para el PID.
    const double rpmA = static_cast<double>(velA) * (60.0 / (2.0 * M_PI));
    const double rpmB = static_cast<double>(velB) * (60.0 / (2.0 * M_PI));

    motorA.setSetpoint(rpmA);
    motorB.setSetpoint(rpmB);

    ultimoPaqueteUDP = millis();
    comunicacionActiva = true;
}

// ======================================================
// TELEMETRIA UDP
// ======================================================
void enviarTelemetriaUDP() {
    static unsigned long tTelemetry = 0;
    const unsigned long now = millis();

    if (!clienteUDPConocido || now - tTelemetry < TELEMETRY_PERIOD_MS) {
        return;
    }
    tTelemetry = now;

    const double spA = motorA.getSetpoint();
    const double pvA = motorA.getPV();
    const double spB = motorB.getSetpoint();
    const double pvB = motorB.getPV();

    const double rpmToRad = 2.0 * M_PI / 60.0;

    const char* modeA = fabs(spA) <= 1e-3
        ? "STOP"
        : (motorA.isStarting() ? "START" : "RUN");
    const char* modeB = fabs(spB) <= 1e-3
        ? "STOP"
        : (motorB.isStarting() ? "START" : "RUN");

    // JSON sin ArduinoJson para no agregar dependencias.
    char telemetry[640];
    const int n = snprintf(
        telemetry, sizeof(telemetry),
        "{\"t_ms\":%lu,\"rssi\":%ld,\"comm\":%d,"
        "\"A\":{\"sp_rpm\":%.3f,\"pv_rpm\":%.3f,\"sp_rad\":%.3f,\"pv_rad\":%.3f,"
        "\"err_rpm\":%.3f,\"ff\":%.1f,\"pid\":%.1f,\"pwm\":%.1f,\"mode\":\"%s\"},"
        "\"B\":{\"sp_rpm\":%.3f,\"pv_rpm\":%.3f,\"sp_rad\":%.3f,\"pv_rad\":%.3f,"
        "\"err_rpm\":%.3f,\"ff\":%.1f,\"pid\":%.1f,\"pwm\":%.1f,\"mode\":\"%s\"}}",
        now,
        static_cast<long>(WiFi.RSSI()),
        comunicacionActiva ? 1 : 0,
        spA, pvA, spA * rpmToRad, pvA * rpmToRad,
        motorA.getErrorRPM(), motorA.getFeedforwardPWM(), motorA.getPIDPWM(), motorA.getOutput(), modeA,
        spB, pvB, spB * rpmToRad, pvB * rpmToRad,
        motorB.getErrorRPM(), motorB.getFeedforwardPWM(), motorB.getPIDPWM(), motorB.getOutput(), modeB
    );

    if (n <= 0 || n >= static_cast<int>(sizeof(telemetry))) {
        Serial.println("Error construyendo telemetria UDP.");
        return;
    }

    Udp.beginPacket(ultimoClienteIP, telemetryUdpPort);
    Udp.write(reinterpret_cast<const uint8_t*>(telemetry), n);
    Udp.endPacket();
}

// ======================================================
// DEBUG
// ======================================================
void imprimirDebug() {
    static unsigned long tPrint = 0;

    if (millis() - tPrint >= 200) {
        tPrint = millis();

        Serial.print("A -> ");
        motorA.imprimirDatos();

        Serial.print("B -> ");
        motorB.imprimirDatos();

        Serial.println();
    }
}

// ======================================================
// SETUP
// ======================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("===========================================");
    Serial.println(" EMG30: arranque + Feedforward + PID");
    Serial.println("===========================================");

    motorA.begin();
    motorB.begin();

    // --------------------------------------------------
    // FRICCION / ARRANQUE
    // --------------------------------------------------
    // Arranque: 155 PWM durante 150 ms para vencer friccion estatica.
    // Marcha: minimo de 60 PWM, pero sin invertir el sentido del SP.
    motorA.setStartupPWM(155.0, 150);
    motorB.setStartupPWM(155.0, 150);

    motorA.setRunMinPWM(60.0);
    motorB.setRunMinPWM(60.0);

    // Feedforward independiente por rueda.
    // Positivo: calibrado con el robot en piso.
    // Negativo: modelo provisional identificado con pruebas en reversa en piso.
    motorA.setFeedforwardModel(
        A_A_POS, A_B_POS,
        A_A_NEG, A_B_NEG
    );

    motorB.setFeedforwardModel(
        B_A_POS, B_B_POS,
        B_A_NEG, B_B_NEG
    );

    conectarWiFi();

    Udp.begin(localUdpPort);
    Serial.print("Escuchando comandos UDP en puerto ");
    Serial.println(localUdpPort);
    Serial.print("Telemetria UDP -> puerto ");
    Serial.println(telemetryUdpPort);

    detenerRobot();
}

// ======================================================
// LOOP
// ======================================================
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi desconectado. Deteniendo robot...");
        detenerRobot();
        comunicacionActiva = false;

        conectarWiFi();
        Udp.stop();
        Udp.begin(localUdpPort);
    }

    recibirUDP();

    if (comunicacionActiva &&
        (millis() - ultimoPaqueteUDP > TIMEOUT_UDP)) {

        Serial.println("Timeout UDP. Deteniendo robot.");
        detenerRobot();
        comunicacionActiva = false;
    }

    motorA.actualizar();
    motorB.actualizar();

    enviarTelemetriaUDP();
    imprimirDebug();
}
