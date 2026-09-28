#include "motor-PID.h"

// ======================================================
// CONSTRUCTOR
// ======================================================

MotorPID::MotorPID(
    uint8_t encA, 
    uint8_t encB, 
    uint8_t motorIn1, 
    uint8_t motorIn2, 
    uint8_t motorPwm, 
    double kp, 
    double ki, 
    double kd, 
    CalibracionPWM calibracion
) {
    pinEncA = encA;
    pinEncB = encB;
    pinIn1 = motorIn1;
    pinIn2 = motorIn2;
    pinPwm = motorPwm;
    Kp = kp;
    Ki = ki;
    Kd = kd;
    cal = calibracion;
}

// ======================================================
// BEGIN
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

void MotorPID::setPV(double PV) {
    pv = PV;
}

// ======================================================
// ACTUALIZAR (Ejecuta la lectura del encoder y el PID)
// ======================================================

void MotorPID::actualizar() {
    unsigned long currentTime = millis();

    // Convertir Ts a milisegundos para el temporizador (0.02 s * 1000 = 20 ms)
    if (currentTime - lastTime >= (unsigned long)(Ts * 1000.0)) {
        int64_t count = encoder.getCount();

        // Conversión usando la variable Ts de la clase
        setPV((count / 360.0) * (2.0 * M_PI) / Ts); //

        encoder.clearCount(); 
        lastTime = currentTime; 

        // Ejecutar algoritmo de control PID
        pidControl(); 
    }
}


void MotorPID::pidControl() {

    double error = getSetpoint() - getPV();

    // ==========================================
    // MOTOR REALMENTE DETENIDO
    // ==========================================

    if (fabs(getSetpoint()) < 0.1 && fabs(getPV()) < 0.5) {

        detener();

        return;
    }

    // ==========================================
    // PID
    // ==========================================

    output = CV_Anterior
           + (Kp + Kd / Ts) * error
           + (-Kp + Ki * Ts - 2.0 * Kd / Ts) * error1
           + (Kd / Ts) * error2;

    // ==========================================
    // MEMORIA DEL PID
    // ==========================================

    error2 = error1;
    error1 = error;

    // ==========================================
    // LIMITAR OUTPUT
    // ==========================================

    output = constrain(output, -255.0, 255.0);

    CV_Anterior = output;

    if (output > 0.0) {

        // Adelante
        digitalWrite(pinIn1, HIGH);
        digitalWrite(pinIn2, LOW);

        ledcWrite(
            pinPwm,
            (uint32_t)fabs(output)
        );

    }
    else if (output < 0.0) {

        // Atrás / frenado
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, HIGH);

        ledcWrite(
            pinPwm,
            (uint32_t)fabs(output)
        );

    }
    else {

        detener();
    }
}

// ======================================================
// MÉTODOS AUXILIARES
// ======================================================

void MotorPID::setSetpoint(double sp) {
    setpoint = sp;
}

void MotorPID::detener() {
    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    ledcWrite(pinPwm, 0);

    CV_Anterior = 0.0;
    error1 = 0.0;
    error2 = 0.0;
}

void MotorPID::imprimirDatos() {
    Serial.print("SP:"); Serial.print(setpoint);
    Serial.print(", PV:"); Serial.print(pv);
    Serial.print(", PWM:"); Serial.print(output);
    Serial.print(", Min: -255");
    Serial.print(", Max: 255"); 
    Serial.println();
}
