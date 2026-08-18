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
MotorPID motor(ENCODER_A, ENCODER_B, MOTOR_IN1, MOTOR_IN2, MOTOR_PWM, 3, 0.5, 0);

void setup() {
    Serial.begin(115200);
    motor.begin();
}

void loop() {
    // 1. Suponemos que queremos rad/s en lugar de rpm
    int potVal = analogRead(POT_PIN);
    double rad_s = map(potVal, 0, 4095, -210, 210) / 10.0; // Ej: -21.0 a 21.0 rad/s

    // 2. Asignar la referencia convirtiendo los rad/s en rpm
    motor.setSetpoint(rad_s * (60.0 / (2.0 * M_PI)));
    // 3. Actualizar lectura de encoder y ejecutar el PID si pasaron 100 ms
    motor.actualizar();
    // 4. Salida por puerto serie para Plotter
    motor.imprimirDatos();

    delay(50);
}