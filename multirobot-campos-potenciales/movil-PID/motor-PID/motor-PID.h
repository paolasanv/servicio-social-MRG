#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>

struct CalibracionPWM {
    float mPos;
    float bPos;
    float mNeg;
    float bNeg;
    float pwmMinAbs;
};

class MotorPID {
private:
    uint8_t pinSensorA;
    uint8_t pinSensorB;
    uint8_t pinENA;
    uint8_t pinIN1;
    uint8_t pinIN2;

    bool invertirEncoder = false;
    bool invertirSalida = false;

    CalibracionPWM cal;

    unsigned long intervaloVelocidad = 30;
    unsigned long tiempoVelocidad = 0;

    unsigned long tiempoPID = 0;
    float ts = 0.100;

    int frecuencia = 1000;
    int resolucion = 8;

    // EMG30: 360 cuentas por vuelta del eje de salida
    float COUNTS_PER_REV = 360.0;

    float setPoint = 0.0;
    float pv = 0.0;
    float pvRaw = 0.0;
    float pvAnteriorPID = 0.0;

    // Ahora CV será PWM firmado, no velocidad angular
    float cv = 0.0;
    float pwmFF = 0.0;
    float pwmPID = 0.0;

    float error = 0.0;
    float integral = 0.0;

    float kp;
    float ki;
    float kd;

    float alphaVel = 0.35;
    bool pvInicializada = false;

    float maxSetPointAbs = 18.0;
    float omegaStop = 0.05;

    volatile long conteoEncoder = 0;
    volatile uint8_t estadoAnterior = 0;

    static void IRAM_ATTR encoderISR(void* arg);
    void IRAM_ATTR actualizarEncoder();

    void calcularVelocidad();
    void ejecutarPID();

    float pwmFeedforward(float omegaRef);
    void aplicarPWM(float pwmFirmado);

public:
    MotorPID(
        uint8_t sA,
        uint8_t sB,
        uint8_t ena,
        uint8_t in1,
        uint8_t in2,
        CalibracionPWM calibracion,
        bool invEnc = false,
        bool invOut = false,
        float kP = 0.0,
        float kI = 0.0,
        float kD = 0.0
    );

    void begin();
    void actualizar();

    void setSetPoint(float sp);
    void setGains(float kP, float kI, float kD);
    void setMaxSetPoint(float maxAbs);
    void detener();

    void imprimir();
};

#endif
