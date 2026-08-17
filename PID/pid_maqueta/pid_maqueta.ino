#include <Arduino.h>
#include "motor-PID.h"

// Pines del ESP32
#define POT_PIN   35
#define ENCODER_A 4
#define ENCODER_B 15
#define MOTOR_IN1 26
#define MOTOR_IN2 27
#define MOTOR_PWM 19

// Instancia de la clase con los parámetros originales de pid.ino
MotorPID motor(ENCODER_A, ENCODER_B, MOTOR_IN1, MOTOR_IN2, MOTOR_PWM, 1.2, 2.85, 0.001);

void setup() {
    Serial.begin(115200);
    motor.begin();
}

void loop() {
    // 1. Leer el potenciómetro y mapearlo (-200 a 200 RPM)
    int potValue = analogRead(POT_PIN);
    double setpoint = map(potValue, 0, 4095, -200, 200);

    // 2. Asignar la referencia
    motor.setSetpoint(setpoint);
    // 3. Actualizar lectura de encoder y ejecutar el PID si pasaron 100 ms
    motor.actualizar();
    // 4. Salida por puerto serie para Plotter
    motor.imprimirDatos();

    delay(50);
}
