/*
 * Control de dos motores EMG30 con ESP32 + L298N + UDP
 *
 * Recibe velocidades angulares desde Python en formato:
 *
 * velA,velB
 *
 * Ejemplo:
 * 5.0,5.0
 * -5.0,5.0
 * 0.0,0.0
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "motor-PID.h"

// ======================================================
// CONFIGURACIÓN WIFI
// ======================================================
const char* ssid = "TP-Link_8960";
const char* password = "53899736";

// ======================================================
// CONFIGURACIÓN DE IP FIJA
// ======================================================
// IP fija que tendrá la ESP32.
// Debe ser la misma que se usa en Python.
IPAddress local_IP(192, 168, 0, 100);

// Puerta de enlace de tu red.
// Normalmente es la IP del router.
IPAddress gateway(192, 168, 0, 1);

// Máscara de red.
IPAddress subnet(255, 255, 255, 0);

// DNS. No es indispensable para UDP local, pero conviene definirlo.
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// ======================================================
// CONFIGURACIÓN UDP
// ======================================================
WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[256];

// ======================================================
// PINES DEL DRIVER L298N
// ======================================================

// Motor A
const int IN1 = 18;
const int IN2 = 19;
const int ENA = 5;

// Motor B
const int IN3 = 32;
const int IN4 = 33;
const int ENB = 25;

// ======================================================
// PINES DE ENCODERS EMG30
// ======================================================

// Motor A
const int SENSOR_A_MA = 16;
const int SENSOR_B_MA = 17;

// Motor B
const int SENSOR_A_MB = 26;
const int SENSOR_B_MB = 27;

// ======================================================
// CONFIGURACIÓN DE SENTIDOS
// ======================================================

const bool INVERTIR_ENCODER_A = false;
const bool INVERTIR_ENCODER_B = false;

const bool INVERTIR_SALIDA_A = false;
const bool INVERTIR_SALIDA_B = false;

// Si por montaje mecánico el motor B debe recibir el signo contrario,
// deja esto en true.
const bool INVERTIR_COMANDO_MOTOR_B = true;

// ======================================================
// OBJETOS PID
// ======================================================

MotorPID motorA(
  SENSOR_A_MA,
  SENSOR_B_MA,
  ENA,
  IN1,
  IN2,
  INVERTIR_ENCODER_A,
  INVERTIR_SALIDA_A,
  2.6,    // kP
  0.8,    // kI
  0.001   // kD
);

MotorPID motorB(
  SENSOR_A_MB,
  SENSOR_B_MB,
  ENB,
  IN3,
  IN4,
  INVERTIR_ENCODER_B,
  INVERTIR_SALIDA_B,
  2.8,    // kP
  0.9,    // kI
  0.001   // kD
);

void conectarWiFi() {
  WiFi.mode(WIFI_STA);

  // Configurar IP fija antes de iniciar WiFi
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Error: no se pudo configurar la IP fija.");
  }

  WiFi.begin(ssid, password);

  Serial.print("Conectando a WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi conectado correctamente.");

  Serial.print("IP de la ESP32: ");
  Serial.println(WiFi.localIP());

  Serial.print("Puerta de enlace: ");
  Serial.println(WiFi.gatewayIP());

  Serial.print("Máscara de red: ");
  Serial.println(WiFi.subnetMask());
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=====================================");
  Serial.println(" Control UDP para motores EMG30");
  Serial.println(" ESP32 + L298N + Encoder cuadratura");
  Serial.println(" IP fija habilitada");
  Serial.println("=====================================");

  motorA.begin();
  motorB.begin();

  conectarWiFi();

  Udp.begin(localUdpPort);

  Serial.print("Escuchando UDP en puerto: ");
  Serial.println(localUdpPort);
  Serial.println("Formato esperado: velA,velB");
  Serial.println("Ejemplo: 5.0,5.0");
}

void loop() {
  // Si se cae la conexión WiFi, intentar reconectar
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Deteniendo motores e intentando reconectar...");

    motorA.setSetPoint(0.0);
    motorB.setSetPoint(0.0);

    conectarWiFi();
    Udp.begin(localUdpPort);
  }

  int packetSize = Udp.parsePacket();

  if (packetSize) {
    int len = Udp.read(incomingPacket, 255);

    if (len > 0) {
      incomingPacket[len] = '\0';
    }

    String strPacket = String(incomingPacket);
    strPacket.trim();

    Serial.print("UDP recibido: ");
    Serial.println(strPacket);

    int sepIndex = strPacket.indexOf(',');

    if (sepIndex > 0) {
      String part1 = strPacket.substring(0, sepIndex);
      String part2 = strPacket.substring(sepIndex + 1);

      part1.trim();
      part2.trim();

      float velA = part1.toFloat();
      float velB = part2.toFloat();

      if (INVERTIR_COMANDO_MOTOR_B) {
        velB = -velB;
      }

      motorA.setSetPoint(velA);
      motorB.setSetPoint(velB);

      Serial.print("SetPoint Motor A: ");
      Serial.print(velA);
      Serial.print(" rad/s | SetPoint Motor B: ");
      Serial.print(velB);
      Serial.println(" rad/s");

    } else {
      Serial.println("Formato inválido. Usa: velA,velB");
    }
  }

  motorA.actualizar();
  motorB.actualizar();

  // Para depuración puedes activar:
  // motorA.imprimir();
  // motorB.imprimir();
}
