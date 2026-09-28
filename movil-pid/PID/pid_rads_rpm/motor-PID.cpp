#include "motor-PID.h"

MotorPID::MotorPID(
    uint8_t encA,
    uint8_t encB,
    uint8_t motorIn1,
    uint8_t motorIn2,
    uint8_t motorPwm,
    double kp,
    double ki,
    double kd
) {
    pinEncA = encA;
    pinEncB = encB;
    pinIn1 = motorIn1;
    pinIn2 = motorIn2;
    pinPwm = motorPwm;

    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void MotorPID::begin() {
    // EMG30: encoder open collector.
    // Recomendado: pull-up externo de 4.7 kOhm a 3.3 V.
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(pinEncA, pinEncB);
    encoder.clearCount();

    pinMode(pinIn1, OUTPUT);
    pinMode(pinIn2, OUTPUT);

    // ESP32 Arduino Core 3.x: PWM 1 kHz, 8 bits.
    ledcAttach(pinPwm, 1000, 8);

    lastEncoderCount = encoder.getCount();
    lastTime = millis();
    detener();
}

void MotorPID::setPV(double PV) {
    pv = PV;
}

void MotorPID::actualizar() {
    const unsigned long currentTime = millis();
    const unsigned long elapsedMs = currentTime - lastTime;

    if (elapsedMs < SAMPLE_PERIOD_MS) {
        return;
    }

    const int64_t currentCount = encoder.getCount();
    const int64_t deltaCount = currentCount - lastEncoderCount;
    const double dt = elapsedMs / 1000.0;

    lastEncoderCount = currentCount;
    lastTime = currentTime;

    if (dt <= 0.0) {
        return;
    }

    // EMG30: 360 cuentas por revolucion del eje de salida.
    setPV((static_cast<double>(deltaCount) / COUNTS_PER_REVOLUTION) * (60.0 / dt));

    pidControl(dt);
}

void MotorPID::pidControl(double dt) {
    if (dt <= 0.0) {
        return;
    }

    // SP = 0: paro inmediato y memoria limpia.
    if (fabs(setpoint) <= ZERO_SETPOINT_EPS) {
        resetControlState();
        aplicarPWM(0.0);
        return;
    }

    // ------------------------------------------------------
    // FASE DE ARRANQUE
    // ------------------------------------------------------
    // Durante el arranque NO usamos PID. Se aplica un PWM fijo
    // para vencer friccion estatica sin dar un golpe de PWM=255
    // por el gran error inicial del PID.
    if (arranqueVigente()) {
        lastFeedforwardPWM = calcularFeedforward(); // solo para debug

        output = (setpoint > 0.0) ? startupPWM : -startupPWM;
        output = limitar(output, -PWM_MAX, PWM_MAX);

        // El PID empieza limpio cuando termine el arranque.
        CV_Anterior = 0.0;
        error1 = 0.0;
        error2 = 0.0;

        aplicarPWM(output);
        return;
    }

    // ------------------------------------------------------
    // FASE DE MARCHA: Feedforward + PID
    // ------------------------------------------------------
    const double error = setpoint - pv;

    // PID incremental (velocity form).
    const double pidCandidate =
        CV_Anterior
        + (Kp + Kd / dt) * error
        + (-Kp + Ki * dt - 2.0 * Kd / dt) * error1
        + (Kd / dt) * error2;

    const double feedforward = calcularFeedforward();
    const double totalCandidate = feedforward + pidCandidate;

    // Primero: no permitir inversion respecto al setpoint y aplicar
    // el PWM minimo de marcha.
    const double actuatorCandidate = limitarPWMEnMarcha(totalCandidate);

    // Saturacion fisica final.
    output = limitar(actuatorCandidate, -PWM_MAX, PWM_MAX);

    // Anti-windup por seguimiento del actuador realmente aplicado:
    // FF + estadoPID = output.
    CV_Anterior = output - feedforward;
    CV_Anterior = limitar(CV_Anterior, -PWM_MAX, PWM_MAX);

    error2 = error1;
    error1 = error;

    aplicarPWM(output);
}

void MotorPID::setSetpoint(double spRPM) {
    const double oldSetpoint = setpoint;

    // Cero: detener de inmediato.
    if (fabs(spRPM) <= ZERO_SETPOINT_EPS) {
        setpoint = 0.0;
        startupActive = false;
        resetControlState();
        aplicarPWM(0.0);
        return;
    }

    const bool estabaDetenido = fabs(oldSetpoint) <= ZERO_SETPOINT_EPS;
    const bool cambioSentido =
        (oldSetpoint > ZERO_SETPOINT_EPS && spRPM < -ZERO_SETPOINT_EPS) ||
        (oldSetpoint < -ZERO_SETPOINT_EPS && spRPM > ZERO_SETPOINT_EPS);

    setpoint = spRPM;

    // Solo reiniciar la fase de arranque cuando realmente partimos
    // desde cero o cambiamos el sentido. Los paquetes UDP repetidos
    // NO vuelven a iniciar el temporizador.
    if (estabaDetenido || cambioSentido) {
        resetControlState();
        iniciarArranque();
    }
}

void MotorPID::setStartupPWM(double pwmStart, unsigned long startTimeMs) {
    startupPWM = limitar(fabs(pwmStart), 0.0, PWM_MAX);
    startupDurationMs = startTimeMs;
}

void MotorPID::setRunMinPWM(double pwmRun) {
    runMinPWM = limitar(fabs(pwmRun), 0.0, PWM_MAX);
}

void MotorPID::setFeedforwardModel(
    double aPos, double bPos,
    double aNeg, double bNeg
) {
    if (fabs(aPos) < 1e-12 || fabs(aNeg) < 1e-12) {
        disableFeedforward();
        return;
    }

    ffAPos = aPos;
    ffBPos = bPos;
    ffANeg = aNeg;
    ffBNeg = bNeg;
    feedforwardEnabled = true;
}

void MotorPID::disableFeedforward() {
    feedforwardEnabled = false;
    ffAPos = 0.0;
    ffBPos = 0.0;
    ffANeg = 0.0;
    ffBNeg = 0.0;
    lastFeedforwardPWM = 0.0;
}

void MotorPID::detener() {
    setpoint = 0.0;
    startupActive = false;
    resetControlState();
    aplicarPWM(0.0);

    lastEncoderCount = encoder.getCount();
    lastTime = millis();
    pv = 0.0;
}

void MotorPID::aplicarPWM(double pwm) {
    const double pwmLimited = limitar(pwm, -PWM_MAX, PWM_MAX);
    const uint32_t duty = static_cast<uint32_t>(round(fabs(pwmLimited)));

    if (duty == 0) {
        // COAST, no frenado activo.
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, LOW);
        ledcWrite(pinPwm, 0);
        return;
    }

    if (pwmLimited > 0.0) {
        digitalWrite(pinIn1, HIGH);
        digitalWrite(pinIn2, LOW);
    } else {
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, HIGH);
    }

    ledcWrite(pinPwm, duty);
}

void MotorPID::resetControlState() {
    output = 0.0;
    CV_Anterior = 0.0;
    error1 = 0.0;
    error2 = 0.0;
    lastFeedforwardPWM = 0.0;
}

void MotorPID::iniciarArranque() {
    startupStartMs = millis();
    startupActive = startupDurationMs > 0 && startupPWM > 0.0;
}

bool MotorPID::arranqueVigente() {
    if (!startupActive) {
        return false;
    }

    if (millis() - startupStartMs >= startupDurationMs) {
        startupActive = false;
        return false;
    }

    return true;
}

double MotorPID::calcularFeedforward() {
    if (!feedforwardEnabled || fabs(setpoint) <= ZERO_SETPOINT_EPS) {
        lastFeedforwardPWM = 0.0;
        return 0.0;
    }

    // El PID trabaja en RPM; la calibracion se hizo en rad/s.
    const double omegaDeseada = setpoint * (2.0 * M_PI / 60.0);

    double pwmFF = 0.0;

    if (omegaDeseada > 0.0) {
        pwmFF = (omegaDeseada - ffBPos) / ffAPos;
    } else {
        pwmFF = (omegaDeseada - ffBNeg) / ffANeg;
    }

    pwmFF = limitar(pwmFF, -PWM_MAX, PWM_MAX);
    lastFeedforwardPWM = pwmFF;
    return pwmFF;
}

double MotorPID::limitarPWMEnMarcha(double pwm) const {
    // No permitimos que el PID invierta el motor por un pequeno
    // sobreimpulso. Puede reducir hasta el PWM minimo de marcha.
    if (setpoint > ZERO_SETPOINT_EPS) {
        if (pwm <= 0.0) {
            // Mejor dejar rodar que invertir el motor.
            return 0.0;
        }

        if (pwm < runMinPWM) {
            return runMinPWM;
        }

        return limitar(pwm, 0.0, PWM_MAX);
    }

    if (setpoint < -ZERO_SETPOINT_EPS) {
        if (pwm >= 0.0) {
            return 0.0;
        }

        if (fabs(pwm) < runMinPWM) {
            return -runMinPWM;
        }

        return limitar(pwm, -PWM_MAX, 0.0);
    }

    return 0.0;
}

double MotorPID::limitar(double value, double minValue, double maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

void MotorPID::imprimirDatos() {
    Serial.print("SP:"); Serial.print(setpoint, 3);
    Serial.print(", PV:"); Serial.print(pv, 3);
    Serial.print(", FF:"); Serial.print(lastFeedforwardPWM, 1);
    Serial.print(", PWM:"); Serial.print(output, 1);
    Serial.print(", MODE:"); Serial.print(startupActive ? "START" : "RUN");
    Serial.print(", PSTART:"); Serial.print(startupPWM, 0);
    Serial.print(", PRUN:"); Serial.print(runMinPWM, 0);
    Serial.println();
}
