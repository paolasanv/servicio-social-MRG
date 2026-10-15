#include <WiFi.h>
#include <WiFiUdp.h>

// ======================================================
// WIFI
// ======================================================
const char* ssid = "TP-Link_8960";
const char* password = "53899736";

const int LED = 19; 

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

// ======================================================
// CONEXIÓN WIFI
// ======================================================
void conectarWiFi() {
    WiFi.mode(WIFI_STA);
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("Error configurando IP fija.");
    }

    WiFi.begin(ssid, password);
    Serial.print("Conectando a WiFi");

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    Serial.println();
    Serial.print("WiFi conectado. Dirección IP: ");
    Serial.println(WiFi.localIP());
    Udp.begin(localUdpPort);
    Serial.print("Escuchando UDP en el puerto ");
    Serial.println(localUdpPort);
}

// ======================================================
// RECEPCIÓN Y RESPUESTA UDP
// ======================================================
void recibirUDP() {

    int packetSize = Udp.parsePacket();

    if (packetSize > 0) {

        int n = Udp.read(incomingPacket, sizeof(incomingPacket) - 1);

        if (n >= 0) {

            // Identificar al emisor del paquete.
            IPAddress ipEmisor = Udp.remoteIP();
            unsigned int puertoEmisor = Udp.remotePort();

            // Convertir el contenido recibido en una cadena.
            incomingPacket[n] = '\0';

            // Mostrar información del paquete recibido.
            Serial.println();
            Serial.println("========== PAQUETE UDP RECIBIDO ==========");

            Serial.print("IP del emisor: ");
            Serial.println(ipEmisor);

            Serial.print("Puerto del emisor: ");
            Serial.println(puertoEmisor);

            Serial.print("Tamaño del paquete: ");
            Serial.print(packetSize);
            Serial.println(" bytes");

            Serial.print("Contenido: ");
            Serial.println(incomingPacket);

            Serial.println("==========================================");

            // Procesar el comando recibido.
            const char* respuesta;

            if (incomingPacket[0] == '1') {
                digitalWrite(LED, HIGH);
                respuesta = "OK: LED ENCENDIDO";
            } else if (incomingPacket[0] == '0') {
                digitalWrite(LED, LOW);
                respuesta = "OK: LED APAGADO";
            } else {
                respuesta = "Mensaje recibido";
            }

            // Enviar respuesta al emisor del paquete.
            Udp.beginPacket(ipEmisor, puertoEmisor);
            Udp.print(respuesta);
            Udp.endPacket();

            Serial.print("Respuesta enviada: ");
            Serial.println(respuesta);
        }
    }
}

// ======================================================
// SETUP
// ======================================================
void setup() {
    Serial.begin(115200);
    conectarWiFi();
    pinMode(LED, OUTPUT);
}

// ======================================================
// LOOP
// ======================================================
void loop() {
    // Verificar la conexión WiFi.
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi desconectado.");
        comunicacionActiva = false;
        Udp.stop();
        conectarWiFi();
        Serial.println("Recepción UDP reiniciada.");
    }

    // Recibir e imprimir paquetes UDP.
    recibirUDP();
}
