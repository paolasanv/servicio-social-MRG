#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(
        uint8_t motorIn1,
        uint8_t motorIn2,
        uint8_t motorPwm,
        float maxSpeed = 100.0f
    );

    void begin();

    // vel en [-maxSpeed, maxSpeed]. Signo = sentido, magnitud = PWM.
    void moverMotor(float vel);
    void detener();

    // Modelo experimental (lazo abierto), omega [rad/s] = a*PWM + b
    //   giro positivo: aPos, bPos   |   giro negativo: aNeg, bNeg
    void setModelo(float aPos, float bPos, float aNeg, float bNeg);

    // Traduce rad/s -> PWM con el modelo y llama a moverMotor().
    void moverRadS(float omega);

    float getVelocidad() const { return velocidad; }
    int getPWM() const { return pwmActual; }

private:
    uint8_t pinIn1;
    uint8_t pinIn2;
    uint8_t pinPwm;

    float maxSpeed;
    float velocidad = 0.0f;
    int pwmActual = 0;

    float ffAPos = 0.0f, ffBPos = 0.0f;
    float ffANeg = 0.0f, ffBNeg = 0.0f;
    bool modeloActivo = false;

    // ESP32 Arduino Core 3.x: PWM 1 kHz, 8 bits.
    static constexpr uint32_t PWM_FREQ = 1000;
    static constexpr uint8_t  PWM_RES  = 8;
    static constexpr int      PWM_MAX  = (1 << PWM_RES) - 1;  // 255
};

#endif
