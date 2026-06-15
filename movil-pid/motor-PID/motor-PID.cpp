#include "motor-PID.h"

MotorPID::MotorPID(
    uint8_t sA,
    uint8_t sB,
    uint8_t ena,
    uint8_t in1,
    uint8_t in2,
    CalibracionPWM calibracion,
    bool invEnc,
    bool invOut,
    float kP,
    float kI,
    float kD
) {
    pinSensorA = sA;
    pinSensorB = sB;
    pinENA = ena;
    pinIN1 = in1;
    pinIN2 = in2;

    cal = calibracion;

    invertirEncoder = invEnc;
    invertirSalida = invOut;

    kp = kP;
    ki = kI;
    kd = kD;
}

void MotorPID::begin() {
    pinMode(pinSensorA, INPUT_PULLUP);
    pinMode(pinSensorB, INPUT_PULLUP);

    pinMode(pinENA, OUTPUT);
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);

    estadoAnterior = (digitalRead(pinSensorA) << 1) | digitalRead(pinSensorB);

    attachInterruptArg(digitalPinToInterrupt(pinSensorA), encoderISR, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pinSensorB), encoderISR, this, CHANGE);

    ledcAttach(pinENA, frecuencia, resolucion);

    detener();

    tiempoVelocidad = millis();
    tiempoPID = millis();
}

void IRAM_ATTR MotorPID::encoderISR(void* arg) {
    MotorPID* motor = static_cast<MotorPID*>(arg);
    motor->actualizarEncoder();
}

void IRAM_ATTR MotorPID::actualizarEncoder() {
    uint8_t estadoActual = (digitalRead(pinSensorA) << 1) | digitalRead(pinSensorB);
    uint8_t transicion = (estadoAnterior << 2) | estadoActual;

    int direccion = 0;

    if (
        transicion == 0b1101 ||
        transicion == 0b0100 ||
        transicion == 0b0010 ||
        transicion == 0b1011
    ) {
        direccion = 1;
    }
    else if (
        transicion == 0b1110 ||
        transicion == 0b0111 ||
        transicion == 0b0001 ||
        transicion == 0b1000
    ) {
        direccion = -1;
    }

    if (invertirEncoder) {
        direccion = -direccion;
    }

    conteoEncoder += direccion;
    estadoAnterior = estadoActual;
}

void MotorPID::actualizar() {
    unsigned long ahora = millis();

    if ((ahora - tiempoVelocidad) >= intervaloVelocidad) {
        calcularVelocidad();
    }

    if ((ahora - tiempoPID) >= (unsigned long)(ts * 1000.0)) {
        ejecutarPID();
    }
}

void MotorPID::calcularVelocidad() {
    unsigned long ahora = millis();
    float dt = (ahora - tiempoVelocidad) / 1000.0;

    if (dt <= 0.0) return;

    tiempoVelocidad = ahora;

    long deltaConteos = 0;

    noInterrupts();
    deltaConteos = conteoEncoder;
    conteoEncoder = 0;
    interrupts();

    pvRaw = ((float)deltaConteos / COUNTS_PER_REV) * (2.0 * PI) / dt;

    if (!pvInicializada) {
        pv = pvRaw;
        pvInicializada = true;
    } else {
        pv = alphaVel * pvRaw + (1.0 - alphaVel) * pv;
    }
}

float MotorPID::pwmFeedforward(float omegaRef) {
    if (fabsf(omegaRef) < omegaStop) {
        return 0.0;
    }

    float pwm = 0.0;

    if (omegaRef > 0.0) {
        // omega = mPos * PWM + bPos
        pwm = (omegaRef - cal.bPos) / cal.mPos;

        if (pwm > 0.0 && pwm < cal.pwmMinAbs) {
            pwm = cal.pwmMinAbs;
        }
    } else {
        // omega = mNeg * PWM + bNeg
        // Aquí omegaRef es negativa y PWM también debe salir negativo
        pwm = (omegaRef - cal.bNeg) / cal.mNeg;

        if (pwm < 0.0 && fabsf(pwm) < cal.pwmMinAbs) {
            pwm = -cal.pwmMinAbs;
        }
    }

    return constrain(pwm, -255.0, 255.0);
}

void MotorPID::ejecutarPID() {
    tiempoPID = millis();

    if (fabsf(setPoint) < omegaStop) {
        detener();
        return;
    }

    error = setPoint - pv;

    pwmFF = pwmFeedforward(setPoint);

    float derivadaPV = (pv - pvAnteriorPID) / ts;

    float uSinSaturar = pwmFF
                      + kp * error
                      + ki * integral
                      - kd * derivadaPV;

    float uSaturado = constrain(uSinSaturar, -255.0, 255.0);

    bool saturadoAlto = uSinSaturar > 255.0;
    bool saturadoBajo = uSinSaturar < -255.0;

    // Anti-windup por integración condicional
    if (
        (!saturadoAlto && !saturadoBajo) ||
        (saturadoAlto && error < 0.0) ||
        (saturadoBajo && error > 0.0)
    ) {
        integral += error * ts;
    }

    if (ki > 0.0001) {
        float limiteIntegral = 120.0 / ki;
        integral = constrain(integral, -limiteIntegral, limiteIntegral);
    }

    pwmPID = kp * error + ki * integral - kd * derivadaPV;

    cv = pwmFF + pwmPID;
    cv = constrain(cv, -255.0, 255.0);

    pvAnteriorPID = pv;

    aplicarPWM(cv);
}

void MotorPID::aplicarPWM(float pwmFirmado) {
    int pwm = (int)round(fabsf(pwmFirmado));

    if (pwm <= 0) {
        ledcWrite(pinENA, 0);
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);
        return;
    }

    bool sentidoPositivo = pwmFirmado >= 0.0;

    if (invertirSalida) {
        sentidoPositivo = !sentidoPositivo;
    }

    if (sentidoPositivo) {
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
    } else {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
    }

    pwm = constrain(pwm, 0, 255);
    ledcWrite(pinENA, pwm);
}

void MotorPID::setSetPoint(float sp) {
    sp = constrain(sp, -maxSetPointAbs, maxSetPointAbs);

    if ((sp * setPoint) < 0.0) {
        integral = 0.0;
    }

    if (fabsf(sp) < omegaStop) {
        setPoint = 0.0;
        detener();
        return;
    }

    setPoint = sp;
}

void MotorPID::setGains(float kP, float kI, float kD) {
    kp = kP;
    ki = kI;
    kd = kD;
    integral = 0.0;
}

void MotorPID::setMaxSetPoint(float maxAbs) {
    maxSetPointAbs = fabsf(maxAbs);
}

void MotorPID::detener() {
    ledcWrite(pinENA, 0);

    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);

    cv = 0.0;
    pwmFF = 0.0;
    pwmPID = 0.0;

    error = 0.0;
    integral = 0.0;

    pvAnteriorPID = pv;
}

void MotorPID::imprimir() {
    Serial.print(" SP:");
    Serial.print(setPoint, 2);

    Serial.print(", PV:");
    Serial.print(pv, 2);

    Serial.print(", PVraw:");
    Serial.print(pvRaw, 2);

    Serial.print(", Error:");
    Serial.print(error, 2);

    Serial.print(", PWMff:");
    Serial.print(pwmFF, 1);

    Serial.print(", PWMpid:");
    Serial.print(pwmPID, 1);

    Serial.print(", PWM:");
    Serial.print(cv, 1);

    Serial.println();
}
