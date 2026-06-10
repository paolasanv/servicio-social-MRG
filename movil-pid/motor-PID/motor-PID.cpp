#include "motor-PID.h"

MotorPID::MotorPID(
    uint8_t sA,
    uint8_t sB,
    uint8_t ena,
    uint8_t in1,
    uint8_t in2,
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
    invertirEncoder = invEnc;
    invertirSalida = invOut;
    kp = kP;
    ki = kI;
    kd = kD;
}

void MotorPID::begin() {
    // Los EMG30 tienen salidas Hall tipo open collector.
    // En GPIO 26 y 27 sí podemos usar pull-up interno.
    pinMode(pinSensorA, INPUT_PULLUP);
    pinMode(pinSensorB, INPUT_PULLUP);

    pinMode(pinENA, OUTPUT);
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);

    estadoAnterior = (digitalRead(pinSensorA) << 1) | digitalRead(pinSensorB);

    attachInterruptArg(digitalPinToInterrupt(pinSensorA), encoderISR, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pinSensorB), encoderISR, this, CHANGE);

    ledcAttach(pinENA, frecuencia, resolucion);

    ledcWrite(pinENA, 0);
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);

    tiempoInicial = millis();
    ti = millis();
}

void IRAM_ATTR MotorPID::encoderISR(void* arg) {
    MotorPID* motor = static_cast<MotorPID*>(arg);
    motor->actualizarEncoder();
}

void IRAM_ATTR MotorPID::actualizarEncoder() {
    uint8_t estadoActual = (digitalRead(pinSensorA) << 1) | digitalRead(pinSensorB);
    uint8_t transicion = (estadoAnterior << 2) | estadoActual;

    int direccion = 0;

    // Decodificación cuadratura
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
    tiempoActual = millis();

    if ((tiempoActual - tiempoInicial) >= intervalo) {
        calcularVelocidad();
    }

    if ((millis() - ti) >= (ts * 1000)) {
        ejecutarPID();
       // imprimir();
    }
}

void MotorPID::calcularVelocidad() {
    unsigned long ahora = millis();
    float dt = (ahora - tiempoInicial) / 1000.0;

    if (dt <= 0.0) return;

    tiempoInicial = ahora;

    long deltaConteos = 0;

    noInterrupts();
    deltaConteos = conteoEncoder;
    conteoEncoder = 0;
    interrupts();

    // Velocidad angular en rad/s
    pv = ((float)deltaConteos / COUNTS_PER_REV) * (2.0 * PI) / dt;

    Serial.print("deltaConteos = ");
    Serial.println(deltaConteos);
}

void MotorPID::ejecutarPID() {
    ti = millis();

    // Si la referencia es cero, apagamos el motor y limpiamos memoria del PID.
    if (abs(setPoint) < 0.01) {
        ledcWrite(pinENA, 0);
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);

        cv = 0.0;
        cvAnterior = 0.0;
        error = 0.0;
        error1 = 0.0;
        error2 = 0.0;

        return;
    }

    error = setPoint - pv;

    cv = cvAnterior
       + (kp + kd / ts) * error
       + (-kp + ki * ts - 2.0 * kd / ts) * error1
       + (kd / ts) * error2;

    if (cv > 500.0) cv = 500.0;
    if (cv < -500.0) cv = -500.0;

    cvAnterior = cv;
    error2 = error1;
    error1 = error;

    int16_t pwm = ajustarDireccionYEscalar(cv);
    ledcWrite(pinENA, pwm);
}

int16_t MotorPID::ajustarDireccionYEscalar(float omega) {
    float pwm = 0.0;
    float magnitud = abs(omega);

    if (magnitud < 1.0) {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);
        return 0;
    }

    bool sentidoPositivo = omega >= 0.0;

    if (invertirSalida) {
        sentidoPositivo = !sentidoPositivo;
    }

    if (sentidoPositivo) {
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);

        // Modelo positivo que ya tenías
        pwm = (magnitud - (-23.2700809430)) / 0.1757770768;

        if (pwm < 140) pwm = 140;
    } else {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);

        // Modelo negativo que ya tenías
        pwm = (magnitud - 24.2606930126) / 0.1797019906;

        if (pwm < 140) pwm = 140;
    }

    return (int16_t)round(constrain(pwm, 0, 255));
}

void MotorPID::imprimir() {
    Serial.print("SP:");
    Serial.print(setPoint);
    Serial.print(", PV:");
    Serial.print(pv);
    Serial.print(", CV:");
    Serial.print(cv);
    Serial.print(", m:");
    Serial.print(-200);
    Serial.print(", M:");
    Serial.print(200);
    Serial.println();
}

void MotorPID::setSetPoint(float sp) {
    setPoint = sp;
}
