#include "motor-PID.h"

volatile int MotorPID::cambios = 0;

MotorPID::MotorPID(uint8_t sA, uint8_t sB, uint8_t ena, uint8_t in1, uint8_t in2) {
    pinSensorA = sA;
    pinSensorB = sB;
    pinENA = ena;
    pinIN1 = in1;
    pinIN2 = in2;
    maxOmega = maxRPM * PI / 30.0;
}

void MotorPID::begin() {
    pinMode(pinSensorA, INPUT);
    pinMode(pinSensorB, INPUT);
    pinMode(pinENA, OUTPUT);
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(pinSensorA), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinSensorB), encoderISR, CHANGE);

    ledcAttach(pinENA, frecuencia, resolucion);

    digitalWrite(pinENA, 0);
    digitalWrite(pinIN1, 0);
    digitalWrite(pinIN2, 1); 

    tiempoInicial = millis();
    ti = millis();
}

void IRAM_ATTR MotorPID::encoderISR() {
    cambios++;
}

void MotorPID::actualizar() {
    tiempoActual = millis();

    if ((tiempoActual - tiempoInicial) >= intervalo) {
        calcularVelocidad();
    }

    if ((millis() - ti) >= (ts * 1000)) {
        ejecutarPID();
        imprimir();
    }
}

void MotorPID::calcularVelocidad() {
    tiempoInicial = tiempoActual;
    pv = (1000.0 / intervalo) * cambios * (2.0 * PI / 360.00);
    cambios = 0;
}

void MotorPID::ejecutarPID() {
    ti = millis();

    error = setPoint - pv;

    // Ecuación de diferencias del PID
    cv = cvAnterior + (kp + kd / ts) * error + (-kp + ki * ts - 2 * kd / ts) * error1 + (kd / ts) * error2;

    cvAnterior = cv;
    error2 = error1;
    error1 = error;

    // Límites de saturación del control
    if (cv > 500.00) cv = 500.00;
    if (cv < -500.00) cv = -500.00; // Se permite valor negativo si el motor cambia de sentido

    // Control de apagado total si la referencia es cero
    if (setPoint == 0.0f) {
        ledcWrite(pinENA, 0);
        digitalWrite(pinIN1, 0);
        digitalWrite(pinIN2, 0);
    } else {
        int16_t wc = ajustarDireccionYEscalar(cv);
        ledcWrite(pinENA, wc);
    }
}

int16_t MotorPID::ajustarDireccionYEscalar(float omega) {
    float pwm;
    
    // Cambia el giro del puente H basándose en el signo del valor correctivo omega
    if (omega >= 0.0f) {
        digitalWrite(pinIN1, 1);
        digitalWrite(pinIN2, 0);
        pwm = (omega - (-23.2700809430)) / 0.1757770768;
        if (pwm < 140) pwm = 140;
    } else {
        digitalWrite(pinIN1, 0);
        digitalWrite(pinIN2, 1);
        omega = abs(omega); // Se extrae la magnitud para el mapeo matemático
        pwm = (omega - 24.2606930126) / 0.1797019906;
        if (pwm < 140) pwm = 140;
    }
    
    return (int16_t) round(constrain(pwm, 0, 255));
}

void MotorPID::imprimir() {
    Serial.print("SP:");
    Serial.print(setPoint);
    Serial.print(", PV:");
    Serial.print(pv);
    Serial.print(", m:");
    Serial.print(-200);      
    Serial.print(", M:");
    Serial.print(200);   
    Serial.println();
}

// Implementación corregida usando el operador de resolución de ámbito de la clase
void MotorPID::setSetPoint(float sp) {
    setPoint = sp;
}