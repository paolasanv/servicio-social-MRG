#include "motor.h"

Motor::Motor(uint8_t motorIn1, uint8_t motorIn2, uint8_t motorPwm, float maxSpd) {
    pinIn1 = motorIn1;
    pinIn2 = motorIn2;
    pinPwm = motorPwm;
    maxSpeed = maxSpd;
}

void Motor::begin() {
    pinMode(pinIn1, OUTPUT);
    pinMode(pinIn2, OUTPUT);
    ledcAttach(pinPwm, PWM_FREQ, PWM_RES);
    detener();
}

void Motor::moverMotor(float vel) {
    velocidad = vel;

    int pwm = (int)(fabsf(vel) / maxSpeed * PWM_MAX);
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    pwmActual = pwm;

    if (vel > 0) {
        digitalWrite(pinIn1, HIGH);
        digitalWrite(pinIn2, LOW);
    } else if (vel < 0) {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, HIGH);
    } else {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, LOW);
    }

    ledcWrite(pinPwm, pwm);
}

void Motor::detener() {
    moverMotor(0.0f);
}

void Motor::setModelo(float aPos, float bPos, float aNeg, float bNeg) {
    ffAPos = aPos; ffBPos = bPos;
    ffANeg = aNeg; ffBNeg = bNeg;
    modeloActivo = (fabsf(aPos) > 1e-9f && fabsf(aNeg) > 1e-9f);
}

void Motor::moverRadS(float omega) {
    // Sin modelo o por debajo de la banda muerta: parar.
    if (!modeloActivo || fabsf(omega) < 0.5f) {
        moverMotor(0.0f);
        return;
    }

    float pwm = (omega > 0.0f) ? (omega - ffBPos) / ffAPos
                               : (omega - ffBNeg) / ffANeg;

    // PWM (-255..255) -> escala de moverMotor (-maxSpeed..maxSpeed)
    moverMotor(pwm / PWM_MAX * maxSpeed);
}
