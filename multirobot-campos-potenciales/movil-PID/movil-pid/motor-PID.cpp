#include "motor-PID.h"

// ======================================================
// CONSTRUCTOR
// ======================================================

MotorPID::MotorPID(uint8_t encA, uint8_t encB, uint8_t motorIn1, uint8_t motorIn2,uint8_t motorPwm, double kp, double ki, double kd) {
    pinEncA = encA;
    pinEncB = encB;
    pinIn1 = motorIn1;
    pinIn2 = motorIn2;
    pinPwm = motorPwm;
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

// ======================================================
// BEGIN (Fiel a setup() de pid.ino)
// ======================================================

void MotorPID::begin() {
    // Configuración del encoder
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(pinEncA, pinEncB);
    encoder.clearCount();

    // Configuración de pines del motor
    pinMode(pinIn1, OUTPUT);
    pinMode(pinIn2, OUTPUT);

    // Configuración PWM en ESP32
    ledcAttach(pinPwm, 1000, 8);

    // Estado inicial
    detener();

}

void MotorPID::setPV(double PV){
    pv = PV;
}
// ======================================================
// ACTUALIZAR (Ejecuta la lectura del encoder y el PID)
// ======================================================

void MotorPID::actualizar() {
    unsigned long currentTime = millis();

    // Calcular RPM cada 100 ms exactamente como en pid.ino
    if (currentTime - lastTime >= 100) {
        int64_t count = encoder.getCount();
        setPV((count / 360.0) * 60.0 / 0.1); // 360 pulsos por revolución
        encoder.clearCount();
        lastTime = currentTime;
        // Ejecutar algoritmo de control PID
        pidControl();
    }
}

// ======================================================
// PID CONTROL 
// ======================================================

void MotorPID::pidControl() {
    double error = getSetpoint() - getPV();
    output = CV_Anterior + (Kp + Kd / Ts) * error 
             + (-Kp + Ki * Ts - 2 * Kd / Ts) * error1 
             + (Kd / Ts) * error2;

    // Actualizar errores previos
    CV_Anterior = output;
    error2 = error1;
    error1 = error;

    // Compensación de zona muerta
    if (abs(output) > 0 && abs(output) < dead_zone) {
        output = (output > 0) ? dead_zone : -dead_zone;
    }

    // Saturar la señal PWM
    output = constrain(output, -255.0, 255.0);

    // Control del motor en ambos sentidos
    if (getSetpoint() == 0) {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, LOW);
        ledcWrite(pinPwm, 0);
    } else if (output > 0) {
        digitalWrite(pinIn1, HIGH);
        digitalWrite(pinIn2, LOW);
        ledcWrite(pinPwm, abs(output));
    } else {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, HIGH);
        ledcWrite(pinPwm, abs(output));
    }
}

// ======================================================
// MÉTODOS AUXILIARES
// ======================================================

void MotorPID::setSetpoint(double sp) {
    this->setpoint = sp;
}

void MotorPID::detener() {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    ledcWrite(pinPwm, 0);
    output = 0.0;
    CV_Anterior = 0.0;
    error1 = 0.0;
    error2 = 0.0;
}

void MotorPID::imprimirDatos() {
    Serial.print("SP:"); Serial.print(setpoint);  // Setpoint (Referencia)
    Serial.print(", PV:"); Serial.print(pv);    // PV (Variable de Proceso)
    Serial.print(", Min:"); Serial.print(-200);  // Valor mínimo para gráfica
    Serial.print(", Max:"); Serial.print(200);   // Valor máximo para gráfica
    Serial.println();  // Salto de línea para el Serial Plotter
}