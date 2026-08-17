#include <Arduino.h>
#include "motor-PID.h"

// Pines del ESP32
#define POT_PIN   35
#define ENCODER_A 4
#define ENCODER_B 15
#define MOTOR_IN1 26
#define MOTOR_IN2 27
#define MOTOR_PWM 19

CalibracionPWM cal = {
  0.1757770768,      
 -23.2700809430,    
  0.1797019906,      
  24.2606930126,     
  140.0,
  -140.0             
};

// Instancia con la calibración pasada como argumento correctamente
MotorPID motor(ENCODER_A, ENCODER_B, MOTOR_IN1, MOTOR_IN2, MOTOR_PWM, 1.8, 2.85, 0.001, cal);

void setup() {
    Serial.begin(115200);
    motor.begin();
}

void loop() {
    // 1. Leer potenciómetro y convertirlo a setpoint en rad/s (-20.94 a 20.94 rad/s)
    int potVal = analogRead(POT_PIN);
    double rad_deseados = map(potVal, 0, 4095, -210, 210) / 10.0; // Ej: -21.0 a 21.0 rad/s

    // Banda muerta manual en la entrada
    if (fabs(rad_deseados) < 0.1) {
        rad_deseados = 0.0;
    }
    motor.setSetpoint(rad_deseados);
    motor.actualizar();
    motor.imprimirDatos();
   // delay(50);
}