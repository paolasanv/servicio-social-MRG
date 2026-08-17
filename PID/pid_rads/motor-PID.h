#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>

struct CalibracionPWM {
    float A_POS;
    float B_POS;
    float A_NEG;
    float B_NEG;
    float PWM_MIN_POS;
    float PWM_MIN_NEG;
};

class MotorPID {
public:
    // --------------------------------------------------
    // VARIABLES PÚBLICAS
    // --------------------------------------------------
    double setpoint = 0.0;
    double pv = 0.0;     
    double output = 0.0;
    unsigned long lastTime = 0;
    CalibracionPWM cal;

    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    double CV_Anterior = 0.0;
    double error1 = 0.0;
    double error2 = 0.0;

    // --------------------------------------------------
    // CONSTRUCTOR Y MÉTODOS PÚBLICOS
    // --------------------------------------------------
    MotorPID(
        uint8_t encA,
        uint8_t encB,
        uint8_t motorIn1,
        uint8_t motorIn2,
        uint8_t motorPwm,
        double kp,
        double ki,
        double kd,
        CalibracionPWM calibracion
    );

    void begin();
    void actualizar();
    void pidControl();
    
    void setSetpoint(double sp);
    void setPV(double PV);
    void detener();
    
    double getPV() const { return pv; }
    double getSetpoint() const { return setpoint; }
    double pwm(double omegaDeseada);
    void imprimirDatos();

private:
    // Configuración de Hardware
    uint8_t pinEncA;
    uint8_t pinEncB;
    uint8_t pinIn1;
    uint8_t pinIn2;
    uint8_t pinPwm;

    // Encoder de la librería ESP32Encoder
    ESP32Encoder encoder;

    // Control de tiempo y parámetros de control
    double Ts = 0.06;               // Tiempo de muestreo
    const int dead_zone = 75;      // Zona muerta
};

#endif