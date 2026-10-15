#include <Arduino.h>
#include "pid.h"

// Pines del ESP32
#define POT_PIN   35
#define ENCODER_A 4
#define ENCODER_B 15
#define MOTOR_IN1 26
#define MOTOR_IN2 27
#define MOTOR_PWM 19

const double A_POS = 0.1757770768;
const double B_POS = -23.2700809430;
const double A_NEG = 0.1797019906;
const double B_NEG = 24.2606930126;


// Instancia de la clase con los parámetros originales de pid.ino
MotorPID motor(ENCODER_A, ENCODER_B, MOTOR_IN1, MOTOR_IN2, MOTOR_PWM, 2, 1, 0.001);

void setup() {
    Serial.begin(115200);
    motor.begin();

    // Arranque: 155 PWM durante 150 ms para vencer friccion estatica.
    //motor.setStartupPWM(155.0, 150);

    // Marcha: minimo de 60 PWM, pero sin invertir el sentido del SP.
    //motor.setRunMinPWM(60.0);

    // Feedforward independiente por rueda mediante ecuación característica
     motor.setFeedforwardModel(
        A_POS, B_POS,
        A_NEG, B_NEG
    );
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
