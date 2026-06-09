#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>

class MotorPID {
private:
    uint8_t pinSensorA;
    uint8_t pinSensorB;
    uint8_t pinENA;
    uint8_t pinIN1;
    uint8_t pinIN2;

    bool invertirEncoder = false;
    bool invertirSalida = false;

    unsigned long intervalo = 100;
    unsigned long tiempoInicial = 0;
    unsigned long tiempoActual = 0;

    unsigned long ti = 0;
    float ts = 0.100;

    int frecuencia = 1000;
    int resolucion = 8;

    // EMG30: 360 conteos por vuelta del eje de salida
    float COUNTS_PER_REV = 360.0;

    float setPoint = 0.0;
    float pv = 0.0;
    float cv = 0.0;
    float cvAnterior = 0.0;
    float error = 0.0;
    float error1 = 0.0;
    float error2 = 0.0;

    float kp; 
    float ki; 
    float kd;

    volatile long conteoEncoder = 0;
    volatile uint8_t estadoAnterior = 0;

    static void IRAM_ATTR encoderISR(void* arg);
    void IRAM_ATTR actualizarEncoder();

    void calcularVelocidad();
    void ejecutarPID();
    int16_t ajustarDireccionYEscalar(float omega);
    void imprimir();

public:
    MotorPID(
        uint8_t sA,
        uint8_t sB,
        uint8_t ena,
        uint8_t in1,
        uint8_t in2,
        bool invEnc = false,
        bool invOut = false,
        float kP = 0.0,
        float kI = 0.0,
        float kD = 0.0
    );

    void begin();
    void actualizar();
    void setSetPoint(float sp);
};

#endif
