/*
 * ESP32 + L298N + EMG30
 * Control en lazo abierto: sin encoders ni PID.
 *
 * UDP recibe "velA,velB" en rad/s (velocidad angular de cada rueda):
 *   A = rueda derecha, B = rueda izquierda (como cine_dife_continuo.py).
 * Motor::moverRadS() convierte rad/s -> PWM con el modelo experimental
 * y Motor::moverMotor() es lo unico que actua sobre los pines.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include "motor.h"

// ======================================================
// WIFI
// ======================================================
const char* ssid = "TP-Link_8960";
const char* password = "53899736";

IPAddress local_IP(192, 168, 0, 102);
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

const float MAX_SPEED = 100.0f;          // Escala interna de moverMotor()
const float MAX_WHEEL_RAD_S = 20.0f;     // Límite de referencia (igual que OMEGA_MAX en Python)

// Modelo experimental (motor M1, aplicado a ambos): omega = a*PWM + b
const float M_A_POS = 0.1757770768f;
const float M_B_POS = -23.2700809430f;
const float M_A_NEG = 0.1797019906f;
const float M_B_NEG = 24.2606930126f;

// ======================================================
// PINES L298N
// ======================================================
const int IN1 = 32;
const int IN2 = 33;
const int ENA = 25;

const int IN3 = 18;
const int IN4 = 19;
const int ENB = 5;

// ======================================================
// MOTORES (mismo orden de pines que en la versión PID)
// ======================================================
Motor motorA(IN2, IN1, ENA, MAX_SPEED);
Motor motorB(IN3, IN4, ENB, MAX_SPEED);

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

    motorA.moverRadS(limitarReferencia(velA));
    motorB.moverRadS(limitarReferencia(velB));

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
        Serial.print("A: vel=");
        Serial.print(motorA.getVelocidad(), 1);
        Serial.print(" pwm=");
        Serial.print(motorA.getPWM());
        Serial.print(" | B: vel=");
        Serial.print(motorB.getVelocidad(), 1);
        Serial.print(" pwm=");
        Serial.println(motorB.getPWM());
    }
}

// ======================================================
// SETUP / LOOP
// ======================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    motorA.begin();
    motorB.begin();

    motorA.setModelo(M_A_POS, M_B_POS, M_A_NEG, M_B_NEG);
    motorB.setModelo(M_A_POS, M_B_POS, M_A_NEG, M_B_NEG);

    conectarWiFi();

    Udp.begin(localUdpPort);
    Serial.print("Escuchando UDP en puerto ");
    Serial.println(localUdpPort);

    detenerRobot();
}

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

    imprimirDebug();
}
