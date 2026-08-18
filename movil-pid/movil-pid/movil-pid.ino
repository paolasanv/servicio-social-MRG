/*
 * Control de robot diferencial con ESP32 + L298N + EMG30
 * Control de velocidad con:
 *
 * PWM = PWM_calibrado(SP) + PID(SP - PV)
 *
 * Entrada UDP:
 * velA,velB
 *
 * Ejemplos:
 * 5.0,5.0
 * 8.0,-8.0
 * 0.0,0.0
 */

#include <WiFi.h>
#include <WiFiUdp.h>
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
// UDP
// ======================================================

WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[256];

unsigned long ultimoPaqueteUDP = 0;
const unsigned long TIMEOUT_UDP = 350;
bool comunicacionActiva = false;

// ======================================================
// PINES L298N
// ======================================================

// Motor A (izquierda)
const int IN1 = 32; 
const int IN2 = 33; 
const int ENA = 25; 

// Motor B (derecha)
const int IN3 = 18; 
const int IN4 = 19; 
const int ENB = 5; 

// ======================================================
// PINES ENCODERS
// ======================================================

// Motor A (izquierda)
const int SENSOR_A_MA = 26;
const int SENSOR_B_MA = 27;

// Motor B (derecha)
const int SENSOR_A_MB = 16;
const int SENSOR_B_MB = 17; 

// ======================================================
// OBJETOS MOTOR
// ======================================================
MotorPID motorA(
  SENSOR_A_MA,
  SENSOR_B_MA,
  IN2,
  IN1,
  ENA,
  2,    // Kp
  2.95,    // Ki
  0.001  // Kd
);

MotorPID motorB(
  SENSOR_A_MB,
  SENSOR_B_MB,
  IN3,
  IN4,
  ENB,
  2,    // Kp
  2.95,    // Ki
  0.001  // Kd
);

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

void detenerRobot() {
  motorA.setSetpoint(0.0);
  motorB.setSetpoint(0.0);
  motorA.detener();
  motorB.detener();
}

void recibirUDP() {
  int packetSize = Udp.parsePacket();

  if (!packetSize) {
    return;
  }

  int len = Udp.read(incomingPacket, 255);

  if (len > 0) {
    incomingPacket[len] = '\0';
  } else {
    return;
  }

  float velA = 0.0;
  float velB = 0.0;

  int datos = sscanf(incomingPacket, " %f , %f", &velA, &velB);

  if (datos == 2) {
    motorA.setSetpoint(velA * (60.0 / (2.0 * M_PI)));
    motorB.setSetpoint(velB * (60.0 / (2.0 * M_PI)));

    ultimoPaqueteUDP = millis();
    comunicacionActiva = true;

    Serial.print("UDP recibido -> Motor A: ");
    Serial.print(velA, 2);
    Serial.print(" rad/s | Motor B: ");
    Serial.print(velB, 2);
    Serial.println(" rad/s");
  } else {
    Serial.print("Formato inválido: ");
    Serial.println(incomingPacket);
  }
}

void imprimirDebug() {
  static unsigned long tPrint = 0;

  if (millis() - tPrint >= 200) {
    tPrint = millis();

    Serial.println("****** Motor A ******");
    motorA.imprimirDatos();

    Serial.println("====== Motor B ======");
    motorB.imprimirDatos();

    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("=======================================");
  Serial.println(" Control EMG30 con Feedforward + PID");
  Serial.println(" ESP32 + L298N + UDP");
  Serial.println("=======================================");

  motorA.begin();
  motorB.begin();

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

    conectarWiFi();
    Udp.begin(localUdpPort);
  }

  recibirUDP();

  if (comunicacionActiva && (millis() - ultimoPaqueteUDP > TIMEOUT_UDP)) {
    Serial.println("Timeout UDP. Deteniendo robot.");
    detenerRobot();
    comunicacionActiva = false;
  }

  motorA.actualizar();
  motorB.actualizar();
  imprimirDebug();
}
