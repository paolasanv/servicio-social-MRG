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

unsigned long ultimoPaqueteUDP = 0;
const unsigned long TIMEOUT_UDP = 350;
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

const int SENSOR_A_MB = 16;
const int SENSOR_B_MB = 17;

// ======================================================
// PID
// ======================================================
MotorPID motorA(
    SENSOR_A_MA, SENSOR_B_MA,
    IN2, IN1, ENA,
    0.5, 0.1, 0
);

MotorPID motorB(
    SENSOR_A_MB, SENSOR_B_MB,
    IN3, IN4, ENB,
 0.5, 0.1, 0
);

// ======================================================
// CALIBRACION EXPERIMENTAL (M1 aplicada temporalmente a M1 y M2)
// ======================================================
// Archivo experimental:
// POSITIVO: omega = 0.1757770768*PWM - 23.2700809430
// NEGATIVO: omega = 0.1797019906*PWM + 24.2606930126
// PWM minimo detectado: +/-140
//
// SUPOSICION EXPERIMENTAL:
// Se aplicara temporalmente esta misma calibracion a motorA y motorB,
// suponiendo que ambos EMG30 tienen comportamiento identico.
// Cuando se calibre M2, sustituir estos coeficientes por los propios de M2.
const double M1_A_POS = 0.1757770768;
const double M1_B_POS = -23.2700809430;
const double M1_A_NEG = 0.1797019906;
const double M1_B_NEG = 24.2606930126;

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
    // Marcha: minimo de 130 PWM, pero sin invertir el sentido del SP.
    motorA.setStartupPWM(155.0, 150);
    motorB.setStartupPWM(155.0, 150);

    motorA.setRunMinPWM(60.0);
    motorB.setRunMinPWM(60.0);

    // Feedforward experimental aplicado a AMBOS motores.
    // Motor A = modelo calibrado de M1.
    motorA.setFeedforwardModel(
        M1_A_POS, M1_B_POS,
        M1_A_NEG, M1_B_NEG
    );

    // Motor B = MISMO modelo de M1, por hipotesis experimental.
    motorB.setFeedforwardModel(
        M1_A_POS, M1_B_POS,
        M1_A_NEG, M1_B_NEG
    );

    conectarWiFi();

    Udp.begin(localUdpPort);
    Serial.print("Escuchando UDP en puerto ");
    Serial.println(localUdpPort);

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

    imprimirDebug();
}
