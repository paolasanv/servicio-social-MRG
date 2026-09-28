/*
 * DIAGNOSTICO Y CALIBRACION EN LAZO ABIERTO - ESP32 + L298N + EMG30
 * ------------------------------------------------------------------
 * Este sketch NO usa PID ni MotorPID. Aplica PWM crudo directamente
 * al puente H y mide RPM real con el encoder, para:
 *
 *   1) FASE TEST: confirmar que el motor gira y el encoder cuenta,
 *      antes de perder tiempo calibrando.
 *   2) FASE SWEEP: barrer niveles de PWM (positivos y negativos) y
 *      registrar la RPM/rad-s en estado estable de cada motor, para
 *      poder ajustar por minimos cuadrados las rectas:
 *          omega = a*PWM + b
 *      que luego se usan en setFeedforwardModel() del firmware final.
 *
 * Ajusta los pines si son distintos en el robot 2.
 * Abre el Monitor Serie a 115200 baud.
 */

#include <ESP32Encoder.h>
#include <math.h>

// ======================================================
// PINES - AJUSTAR SEGUN EL ROBOT 2 SI ES NECESARIO
// ======================================================
// Motor A
const int A_IN1 = 33;
const int A_IN2 = 32;
const int A_ENA = 25;
const int A_ENC_A = 26;
const int A_ENC_B = 27;

// Motor B
const int B_IN1 = 18;
const int B_IN2 = 19;
const int B_ENA = 5;
const int B_ENC_A = 17;
const int B_ENC_B = 16;

const double COUNTS_PER_REV = 360.0; // EMG30

ESP32Encoder encA;
ESP32Encoder encB;

// ======================================================
// CONFIGURACION DEL BARRIDO
// ======================================================
// Valores de PWM a probar (0-255). Incluye niveles bajos para
// encontrar el PWM minimo de arranque/marcha, y altos para
// cubrir el rango que realmente vas a usar.
const int PWM_LEVELS[] = {60, 80, 100, 120, 140, 160, 180, 200, 230, 255};
const int NUM_LEVELS = sizeof(PWM_LEVELS) / sizeof(PWM_LEVELS[0]);

const unsigned long SETTLE_MS = 800;   // tiempo para que el motor llegue a velocidad estable antes de medir
const unsigned long MEASURE_MS = 1200; // ventana de medicion en estado estable
const unsigned long SAMPLE_MS = 100;   // periodo de muestreo del encoder dentro de la ventana
const unsigned long PAUSE_MS = 600;    // pausa con motor detenido entre niveles

// ======================================================
// BAJO NIVEL: PWM crudo
// ======================================================
void setupMotor(int in1, int in2, int ena) {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    ledcAttach(ena, 1000, 8); // 1 kHz, 8 bits, igual que en motor-PID.cpp
}

// signedPwm: positivo = un sentido, negativo = el otro, 0 = coast
void aplicarPWMCrudo(int in1, int in2, int ena, int signedPwm) {
    int duty = abs(signedPwm);
    if (duty > 255) duty = 255;

    if (duty == 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        ledcWrite(ena, 0);
        return;
    }

    if (signedPwm > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    ledcWrite(ena, duty);
}

void detenerTodo() {
    aplicarPWMCrudo(A_IN1, A_IN2, A_ENA, 0);
    aplicarPWMCrudo(B_IN1, B_IN2, B_ENA, 0);
}

// ======================================================
// MEDICION DE RPM INSTANTANEA A PARTIR DEL ENCODER
// ======================================================
double medirRPM(ESP32Encoder &enc, int64_t &lastCount, unsigned long &lastMillis) {
    const unsigned long now = millis();
    const unsigned long dtMs = now - lastMillis;
    if (dtMs == 0) return 0.0;

    const int64_t count = enc.getCount();
    const int64_t delta = count - lastCount;

    lastCount = count;
    lastMillis = now;

    const double dtMin = dtMs / 60000.0; // ms -> minutos
    return (static_cast<double>(delta) / COUNTS_PER_REV) / dtMin;
}

// ======================================================
// FASE 1: TEST BASICO DE MOVIMIENTO
// ------------------------------------------------------
// Aplica PWM=200 a cada motor, en cada sentido, 1.5 s, e
// imprime si el encoder detecto movimiento. Si aqui ya falla,
// el problema es de cableado/driver/encoder, NO de calibracion.
// ======================================================
void testBasico() {
    Serial.println();
    Serial.println("=== FASE 1: TEST BASICO DE MOVIMIENTO ===");

    struct TestCase {
        const char* nombre;
        int in1, in2, ena;
        ESP32Encoder* enc;
    };

    TestCase casos[] = {
        {"Motor A", A_IN1, A_IN2, A_ENA, &encA},
        {"Motor B", B_IN1, B_IN2, B_ENA, &encB},
    };

    for (auto &c : casos) {
        for (int sentido = 0; sentido < 2; sentido++) {
            int pwm = (sentido == 0) ? 200 : -200;
            Serial.print(c.nombre);
            Serial.print(sentido == 0 ? " adelante (PWM=200): " : " reversa (PWM=-200): ");

            int64_t antes = c.enc->getCount();
            aplicarPWMCrudo(c.in1, c.in2, c.ena, pwm);
            delay(1500);
            int64_t despues = c.enc->getCount();
            aplicarPWMCrudo(c.in1, c.in2, c.ena, 0);
            delay(400);

            int64_t delta = despues - antes;
            Serial.print("delta_encoder=");
            Serial.print((long)delta);

            if (delta == 0) {
                Serial.println("  -> SIN MOVIMIENTO. Revisa cableado IN1/IN2/ENA, alimentacion del L298N, "
                                "o si el encoder esta bien conectado (pull-up).");
            } else {
                Serial.println("  -> OK, se detecto giro.");
            }
        }
    }

    Serial.println("=== FIN FASE 1 ===");
    Serial.println();
}

// ======================================================
// FASE 2: BARRIDO PWM -> RPM (calibracion feedforward)
// ======================================================
void medirNivelEstableUnMotor(const char* nombre, int in1, int in2, int ena,
                               ESP32Encoder &enc, int pwmSigned) {
    aplicarPWMCrudo(in1, in2, ena, pwmSigned);
    delay(SETTLE_MS);

    int64_t lastCount = enc.getCount();
    unsigned long lastMillis = millis();

    double sumaRPM = 0.0;
    int nMuestras = 0;

    unsigned long tFin = millis() + MEASURE_MS;
    while (millis() < tFin) {
        delay(SAMPLE_MS);
        double rpm = medirRPM(enc, lastCount, lastMillis);
        sumaRPM += rpm;
        nMuestras++;
    }

    aplicarPWMCrudo(in1, in2, ena, 0);
    delay(PAUSE_MS);

    if (nMuestras == 0) return;

    double rpmProm = sumaRPM / nMuestras;
    double radS = rpmProm * (2.0 * M_PI / 60.0);

    // Formato CSV facil de copiar/pegar: nombre,pwm,rpm,rad_s
    Serial.print(nombre);
    Serial.print(",");
    Serial.print(pwmSigned);
    Serial.print(",");
    Serial.print(rpmProm, 4);
    Serial.print(",");
    Serial.println(radS, 4);
}

void barridoCalibracion() {
    Serial.println();
    Serial.println("=== FASE 2: BARRIDO PWM -> RPM ===");
    Serial.println("Copia estas lineas para ajustar omega = a*PWM + b por minimos cuadrados.");
    Serial.println("nombre,pwm,rpm,rad_s");

    // Sentido positivo, ambos motores
    for (int i = 0; i < NUM_LEVELS; i++) {
        medirNivelEstableUnMotor("A", A_IN1, A_IN2, A_ENA, encA, PWM_LEVELS[i]);
        medirNivelEstableUnMotor("B", B_IN1, B_IN2, B_ENA, encB, PWM_LEVELS[i]);
    }

    // Sentido negativo, ambos motores
    for (int i = 0; i < NUM_LEVELS; i++) {
        medirNivelEstableUnMotor("A", A_IN1, A_IN2, A_ENA, encA, -PWM_LEVELS[i]);
        medirNivelEstableUnMotor("B", B_IN1, B_IN2, B_ENA, encB, -PWM_LEVELS[i]);
    }

    Serial.println("=== FIN FASE 2 ===");
}

// ======================================================
// SETUP / LOOP
// ======================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("===========================================");
    Serial.println(" DIAGNOSTICO Y CALIBRACION EN LAZO ABIERTO ");
    Serial.println("===========================================");

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encA.attachFullQuad(A_ENC_A, A_ENC_B);
    encB.attachFullQuad(B_ENC_A, B_ENC_B);
    encA.clearCount();
    encB.clearCount();

    setupMotor(A_IN1, A_IN2, A_ENA);
    setupMotor(B_IN1, B_IN2, B_ENA);

    detenerTodo();

    // Paso 1: confirmar que hay movimiento real antes de calibrar.
    //testBasico();

    // Si el test basico mostro movimiento en ambos motores/sentidos,
    // comenta la linea de arriba y descomenta esta para correr el
    // barrido completo (tarda varios minutos):
     barridoCalibracion();
}

void loop() {
    // Nada en loop: todo corre una vez en setup().
    // Para repetir, resetea la placa.
}
