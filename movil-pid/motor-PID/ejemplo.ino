#include "motor-PID.h"

#define ADC 35 // Pin potenciometro

//******* Pines del motor A *******
#define SENSOR_A_MA 15
#define SENSOR_B_MA 4
#define ENA 19
#define IN1 26
#define IN2 27

//******* Pines del motor B*******
/*#define SENSOR_A_MB
#define SENSOR_B_MB
#define ENB
#define IN3
#define IN4 */

MotorPID motorA(ADC, SENSOR_A_MA, SENSOR_B_MA, ENA, IN1, IN2);
//MotorPID motorB(ADC, SENSOR_A_MB, SENSOR_B_MB, ENB, IN3, IN4);

void setup() {
    Serial.begin(115200);
    motorA.begin();  // Inicializar el objeto del motor A
}

void loop() {
    motorB.actualizar();
}
