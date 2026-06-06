/*
 * Esta versión recibe velocidades angulares mediante UDP
 * */

#include <WiFi.h>
#include <WiFiUdp.h>
#include "motor-PID.h"

// Configuración de red WiFi
const char* ssid = "MiRedWifi"; // ← Cambia esto 
const char* password = "123456789"; // ← Cambia esto 

// Configuración de UDP
WiFiUDP Udp; 
const unsigned int localUdpPort = 12345; 
char incomingPacket[255];  // Buffer para almacenar los paquetes UDP 

// Pines de los motores del driver L298N
const int IN1 = 18; 
const int IN2 = 19; 
const int IN3 = 32; 
const int IN4 = 33; 

const int ENA = 5; 
const int ENB = 25; 

// Pines físicos asignados para los encoders (pendiente de revisar)
const int SENSOR_A_MA = 34; 
const int SENSOR_B_MA = 35;
const int SENSOR_A_MB = 36;
const int SENSOR_B_MB = 39;

// Objetos para calcular el PID de cada motor
MotorPID motorA(SENSOR_A_MA, SENSOR_B_MA, ENA, IN1, IN2); 
MotorPID motorB(SENSOR_A_MB, SENSOR_B_MB, ENB, IN3, IN4); 

void setup() {
  Serial.begin(115200); 

  // Inicializa los periféricos internos de los objetos motores
  motorA.begin();
  motorB.begin();

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

  // Inicia UDP
  Udp.begin(localUdpPort); 
  Serial.print("Esperando mensajes en el puerto UDP: ");
  Serial.println(localUdpPort); 
}

void loop() {
  // Procesamiento de los datos entrantes de red
  int packetSize = Udp.parsePacket(); 
  if (packetSize) { 
    int len = Udp.read(incomingPacket, 255); 
    if (len > 0) { 
      incomingPacket[len] = 0; // Termina la cadena de caracteres de forma segura 
    }

    Serial.print("Mensaje UDP recibido: ");
    Serial.println(incomingPacket); 

    // Transforma el buffer en un String manipulable
    String strPacket = String(incomingPacket);
    int sepIndex = strPacket.indexOf(','); 
    
    if (sepIndex > 0) { 
      // Divide el string en dos partes usando la posición de la coma
      String part1 = strPacket.substring(0, sepIndex);
      String part2 = strPacket.substring(sepIndex + 1);

      // Convierte los textos fragmentados a tipos float numéricos
      float vel1 = part1.toFloat();
      float vel2 = part2.toFloat();

      // Envía los nuevos SetPoints de velocidad a los controladores
      motorA.setSetPoint(vel1);
      motorB.setSetPoint(-vel2); 
    }
  }

  // Ejecución constante de las tareas temporizadas del motor (PID y Encoder)
  // Deben ejecutarse fuera del condicional del paquete UDP para mantener el lazo cerrado estable.
  motorA.actualizar();
  motorB.actualizar();
}