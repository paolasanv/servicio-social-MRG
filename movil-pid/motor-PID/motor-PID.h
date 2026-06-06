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

    long intervalo = 100; 
    unsigned long tiempoInicial = 0;
    unsigned long tiempoActual = 0;

    unsigned long ti = 0;
    float ts = 0.100; 

    int frecuencia = 1000;
    int resolucion = 8;
    float maxRPM = 188.0;
    float maxOmega;

    float setPoint = 0.0;
    float pv = 0.0;          
    float cv = 0.0;          
    float cvAnterior = 0.0;  
    float error = 0.0;       
    float error1 = 0.0;      
    float error2 = 0.0;      

    float kp = 1.200;
    float ki = 2.850;
    float kd = 0.001;

    static volatile int cambios;
    static void IRAM_ATTR encoderISR();

    void calcularVelocidad();
    void ejecutarPID();
    int16_t ajustarDireccionYEscalar(float omega); // Se renombró para manejar el sentido físico de IN1 e IN2
    void imprimir();

public:
    MotorPID(uint8_t sA, uint8_t sB, uint8_t ena, uint8_t in1, uint8_t in2);
    void begin();
    void actualizar();
    void setSetPoint(float sp); 
};

#endif