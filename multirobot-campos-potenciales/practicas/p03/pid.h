#ifndef MOTOR_PID_H
#define MOTOR_PID_H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>

class MotorPID {
public:
    double setpoint = 0.0;   // RPM
    double pv = 0.0;         // RPM medidas
    double output = 0.0;     // PWM aplicado [-255,255]

    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    // Estado del PID incremental: componente de realimentacion.
    double CV_Anterior = 0.0;
    double error1 = 0.0;
    double error2 = 0.0;

    MotorPID(
        uint8_t encA,
        uint8_t encB,
        uint8_t motorIn1,
        uint8_t motorIn2,
        uint8_t motorPwm,
        double kp = 1.2,
        double ki = 2.85,
        double kd = 0.001
    );

    void begin();
    void actualizar();
    void pidControl(double dt);

    void setSetpoint(double spRPM);
    void setPV(double PV);
    void detener();

    // Logica de friccion:
    // - Al arrancar desde cero o cambiar de sentido se aplica pwmStart
    //   durante startTimeMs.
    // - En marcha, si el controlador pide un PWM no nulo menor que
    //   pwmRun, se eleva a pwmRun.
    // - Nunca se permite invertir el sentido respecto al setpoint.
    void setStartupPWM(double pwmStart, unsigned long startTimeMs);
    void setRunMinPWM(double pwmRun);

    // Feedforward experimental asimetrico:
    //   omega = aPos*PWM + bPos  (giro positivo)
    //   omega = aNeg*PWM + bNeg  (giro negativo)
    void setFeedforwardModel(
        double aPos, double bPos,
        double aNeg, double bNeg
    );
    void disableFeedforward();

    double getPV() const { return pv; }
    double getSetpoint() const { return setpoint; }
    double getOutput() const { return output; }
    double getFeedforwardPWM() const { return lastFeedforwardPWM; }
    double getPIDPWM() const { return CV_Anterior; }
    double getErrorRPM() const { return setpoint - pv; }
    double getStartupPWM() const { return startupPWM; }
    double getRunMinPWM() const { return runMinPWM; }
    bool isStarting() const { return startupActive; }

    void imprimirDatos();

private:
    uint8_t pinEncA;
    uint8_t pinEncB;
    uint8_t pinIn1;
    uint8_t pinIn2;
    uint8_t pinPwm;

    ESP32Encoder encoder;

    unsigned long lastTime = 0;
    int64_t lastEncoderCount = 0;

    const unsigned long SAMPLE_PERIOD_MS = 100;
    const double COUNTS_PER_REVOLUTION = 360.0;
    const double PWM_MAX = 255.0;
    const double ZERO_SETPOINT_EPS = 1e-3; // RPM
    const double ZERO_PWM_EPS = 1e-9;

    // Arranque y marcha.
    double startupPWM = 155.0;
    double runMinPWM = 130.0;
    unsigned long startupDurationMs = 150;
    unsigned long startupStartMs = 0;
    bool startupActive = false;

    // Feedforward.
    bool feedforwardEnabled = false;
    double ffAPos = 0.0;
    double ffBPos = 0.0;
    double ffANeg = 0.0;
    double ffBNeg = 0.0;
    double lastFeedforwardPWM = 0.0;

    void aplicarPWM(double pwm);
    void resetControlState();
    void iniciarArranque();
    bool arranqueVigente();
    double calcularFeedforward();
    double limitarPWMEnMarcha(double pwm) const;
    static double limitar(double value, double minValue, double maxValue);
};

#endif
