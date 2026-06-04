#include "motor-PID.h"

// Inicialización de la variable estática miembro de la clase
volatile int MotorPID::cambios = 0;

// Constructor: Asigna los pines pasados como argumentos a los atributos de la clase
MotorPID::MotorPID(uint8_t adc, uint8_t sA, uint8_t sB, uint8_t ena, uint8_t in1, uint8_t in2) {
    pinADC = adc;
    pinSensorA = sA;
    pinSensorB = sB;
    pinENA = ena;
    pinIN1 = in1;
    pinIN2 = in2;

    // Calcula la velocidad angular máxima en rad/s basada en las RPM máximas
    maxOmega = maxRPM * PI / 30.0;
}

// Método de configuración inicial (Equivalente al setup original)
void MotorPID::begin() {
    pinMode(pinADC, INPUT);
    pinMode(pinSensorA, INPUT);
    pinMode(pinSensorB, INPUT);
    pinMode(pinENA, OUTPUT);
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);

    // Adjunta las interrupciones externas a la función estática de la clase
    attachInterrupt(digitalPinToInterrupt(pinSensorA), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinSensorB), encoderISR, CHANGE);

    // Configuración inicial del driver de motor (L298N)
    // Nota: Si usas la librería ledc de ESP32 actualizada v3+, ledcAttach maneja los parámetros directo.
    ledcAttach(pinENA, frecuencia, resolucion);

    digitalWrite(pinENA, 0);
    digitalWrite(pinIN1, 0);
    digitalWrite(pinIN2, 1); // Sentido de giro inicial

    // Inicializa los temporizadores
    tiempoInicial = millis();
    ti = millis();
}

// Función de interrupción (ISR) estática: Cuenta los flancos del encoder
void IRAM_ATTR MotorPID::encoderISR() {
    cambios++;
}

// Método principal que gestiona los tiempos de ejecución (Reemplaza la lógica del loop)
void MotorPID::actualizar() {
    tiempoActual = millis();

    // 1. Condición de intervalo de tiempo para calcular la velocidad (PV)
    if ((tiempoActual - tiempoInicial) >= intervalo) {
        calcularVelocidad();
    }

    // 2. Condición de tiempo de muestreo para ejecutar el algoritmo PID y monitorizar
    if ((millis() - ti) >= (ts * 1000)) {
        ejecutarPID();
        imprimir();
    }
}

// Calcula la velocidad angular actual (Proces Variable - PV) en rad/s
void MotorPID::calcularVelocidad() {
    tiempoInicial = tiempoActual; // Reinicia el tiempo inicial de la ventana

    // Calcula rad/s en función de los pulsos contados
    pv = (1000.0 / intervalo) * cambios * (2.0 * PI / 360.00);

    cambios = 0; // Reinicia el contador de cambios para el siguiente intervalo
}

// Implementación del algoritmo de control PID (Ecuación en diferencias)
void MotorPID::ejecutarPID() {
    ti = millis(); // Reinicia el temporizador del PID

    // Lee el potenciómetro y mapea el valor al SetPoint (0 a maxOmega)
    setPoint = analogRead(pinADC) * (maxOmega / 4095.00);

    // Calcula el error actual e(n)
    error = setPoint - pv;

    // Ecuación de diferencias de posición del controlador PID
    cv = cvAnterior + (kp + kd / ts) * error + (-kp + ki * ts - 2 * kd / ts) * error1 + (kd / ts) * error2;

    // Almacena el historial de variables para el próximo ciclo de control
    cvAnterior = cv;
    error2 = error1;
    error1 = error;

    // Restricciones y saturación del valor correctivo (Límites de control)
    if (cv > 500.00) cv = 500.00;
    if (cv < 30.00)  cv = 0.00;

    // Escala la acción de control a un valor PWM de 8 bits válido
    int16_t wc = abs(escalarPWM(cv));

    // Condición de apagado o activación del motor
    if (setPoint == 0) {
        ledcWrite(pinENA, 0);
    } else {
        ledcWrite(pinENA, wc);
    }
}

// Mapea la velocidad angular calculada por el PID a un ciclo de trabajo PWM (-255 a 255)
int16_t MotorPID::escalarPWM(float omega) {
    float pwm;
    if (omega > 0.0f) {
        pwm = (omega - (-23.2700809430)) / 0.1757770768;
        if (pwm < 140) pwm = 140;
    } else {
        pwm = (omega - 24.2606930126) / 0.1797019906;
        if (pwm > -140) pwm = -140;
    }

    // Restringe el resultado entre -255 y 255 y lo redondea a entero
    return (int16_t) round(constrain(pwm, -255, 255));
}

// Envía los datos al puerto serial (Ideal para usar con el Serial Plotter)
void MotorPID::imprimir() {
    Serial.print("SP:");
    Serial.print(setPoint);
    Serial.print(", PV:");
    Serial.print(pv);
    Serial.print(", m:");
    Serial.print(0);      // Límite mínimo de la gráfica
    Serial.print(", M:");
    Serial.print(200);    // Límite máximo de la gráfica
    Serial.println();
}
