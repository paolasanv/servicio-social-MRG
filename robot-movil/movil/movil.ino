#include <WiFi.h>
#include <WiFiUdp.h>

// Configuración de red WiFi
const char* ssid = "TP-Link_8960";         // ← Cambia esto
const char* password = "53899736"; // ← Cambia esto

// Configuración de UDP
WiFiUDP Udp;
const unsigned int localUdpPort = 12345;
char incomingPacket[255];

// Pines de los motores
const int IN1 = 32;
const int IN2 = 33;
const int ENA = 25;
const int IN3 = 19;
const int IN4 = 18;
const int ENB = 5;

// Configuración PWM (LEDC)
const int PWM_FREQ = 1000;   // 1 kHz
const int PWM_RES  = 8;      // 8 bits -> 0..255
const int PWM_MAX  = (1 << PWM_RES) - 1;
const float maxSpeed = 100.0;

void setup() {
  Serial.begin(115200);

  // Pines de dirección
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Pines de enable con PWM por LEDC (ya no se usa pinMode para ENA/ENB)
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  // Conecta a Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Conectado a WiFi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localUdpPort);
  Serial.print("Esperando mensajes en el puerto UDP: ");
  Serial.println(localUdpPort);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = 0;
    }

    Serial.print("Mensaje UDP recibido: ");
    Serial.println(incomingPacket);

    String msg = String(incomingPacket);
    int sepIndex = msg.indexOf(',');
    if (sepIndex > 0) {
      float vel1 = msg.substring(0, sepIndex).toFloat();
      float vel2 = msg.substring(sepIndex + 1).toFloat();
      moverMotor(vel1, IN1, IN2, ENA);
      moverMotor(-vel2, IN3, IN4, ENB);
      Serial.println("Velocidades recibidas");
    }
  }
}

void moverMotor(float vel, int in1, int in2, int pwmPin) {
  int pwm = min((int)(abs(vel) / maxSpeed * PWM_MAX), PWM_MAX);

  if (vel > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (vel < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(pwmPin, pwm);
}