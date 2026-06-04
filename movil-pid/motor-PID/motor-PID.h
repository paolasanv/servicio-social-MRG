#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>

class MotorPID {
private:
    // Pines de hardware
    uint8_t pinADC;
    uint8_t pinSensorA;
    uint8_t pinSensorB;
    uint8_t pinENA;
    uint8_t pinIN1;
    uint8_t pinIN2;

    // Control de tiempo para el cálculo de velocidad (PV)
    long intervalo = 100; // Milisegundos
    unsigned long tiempoInicial = 0;
    unsigned long tiempoActual = 0;

    // Control de tiempo para el bucle PID
    unsigned long ti = 0;
    float ts = 0.100; // Tiempo de muestreo en segundos (100ms)

    // Variables de control de PWM
    int frecuencia = 1000;
    int resolucion = 8;
    float maxRPM = 188.0;
    float maxOmega;

    // Variables del algoritmo PID
    float setPoint = 0.0;
    float pv = 0.0;          // Variable de proceso (rad/s)
    float cv = 0.0;          // Valor correctivo actual
    float cvAnterior = 0.0;  // cv(n-1)
    float error = 0.0;       // e(n)
    float error1 = 0.0;      // e(n-1)
    float error2 = 0.0;      // e(n-2)

    // Constantes de sintonización PID
    float kp = 1.200;
    float ki = 2.850;
    float kd = 0.001;

    // Miembros estáticos necesarios para las interrupciones del encoder
    static volatile int cambios;
    static void IRAM_ATTR encoderISR();

    // Métodos privados internos
    void calcularVelocidad();
    void ejecutarPID();
    int16_t escalarPWM(float omega);
    void imprimir();

public:
    // Constructor
    MotorPID(uint8_t adc, uint8_t sA, uint8_t sB, uint8_t ena, uint8_t in1, uint8_t in2);

    // Inicializa los periféricos (Equivalente al setup())
    void begin();

    // Tarea principal que debe llamarse en el loop()
    void actualizar();
};

#endif
