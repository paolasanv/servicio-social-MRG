/*
  BARRIDO AUTOMATICO EMG30 - ESP32 + L298N + ENCODER
  Cargar en Arduino IDE. El ensayo inicia cuando Python envia START.

  L298N: ENA->25 (retirar jumper), IN1->26, IN2->27
  Encoder: A azul->32, B morado->33, ambos con pull-up 4.7k a 3.3 V
  GND de ESP32, puente y fuente en comun.
*/

#include <Arduino.h>
#if __has_include("esp_arduino_version.h")
  #include "esp_arduino_version.h"
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ---------------- Pines y parametros ----------------
const uint8_t ENA = 19, IN1 = 26, IN2 = 27, ENC_A = 15, ENC_B = 4;
const float CPR = 360.0f;       // verificar con una vuelta manual
const int8_t SIGNO_ENCODER = -1; // cambiar a -1 si el signo sale invertido
      
const uint32_t T_EST_MS = 2500; // transitorio: Python lo descarta
const uint32_t T_MED_MS = 2000; // datos usados para el ajuste
const uint32_t T_SAMPLE_MS = 100;
const uint32_t T_INV_MS = 250;
        
const uint32_t PWM_FREQ = 10000;
const uint8_t PWM_BITS = 8;
#if ESP_ARDUINO_VERSION_MAJOR < 3
const uint8_t PWM_CH = 0;
#endif

// 0=estabilizacion; 1=medicion. Incluye subida y bajada (histeresis).
const int16_t barrido[] = {
   0, 20,40,60,80,100,120,140,160,180,200,220,240,255,
   240,220,200,180,160,140,120,100,80,60,40,20,0,
   -20,-40,-60,-80,-100,-120,-140,-160,-180,-200,-220,-240,-255,
   -240,-220,-200,-180,-160,-140,-120,-100,-80,-60,-40,-20,0
};
const uint16_t N_PUNTOS = sizeof(barrido) / sizeof(barrido[0]);

// ---------------- Encoder ----------------
volatile int32_t cuentas = 0;
volatile uint8_t abAnterior = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrEncoder() {
  portENTER_CRITICAL_ISR(&mux);
  uint8_t ab = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  uint8_t tr = (abAnterior << 2) | ab;
  switch (tr) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: cuentas++; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: cuentas--; break;
  }
  abAnterior = ab;
  portEXIT_CRITICAL_ISR(&mux);
}

int32_t leerCuentas() {
  portENTER_CRITICAL(&mux);
  int32_t n = cuentas;
  portEXIT_CRITICAL(&mux);
  return n;
}

void reiniciarCuentas() {
  portENTER_CRITICAL(&mux);
  cuentas = 0;
  portEXIT_CRITICAL(&mux);
}

// ---------------- Motor ----------------
int16_t pwmActual = 0;
int8_t dirActual = 0;

void pwmWrite(uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ENA, duty);
#else
  ledcWrite(PWM_CH, duty);
#endif
}

void detener() {
  pwmWrite(0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  pwmActual = 0;
  dirActual = 0;
}

void mandarPWM(int16_t pwm) {
  pwm = constrain(pwm, -255, 255);
  int8_t dirNueva = (pwm > 0) ? 1 : ((pwm < 0) ? -1 : 0);

  if (dirNueva == 0) {
    detener();
    return;
  }

  if (dirActual != 0 && dirNueva != dirActual) {
    detener();
    delay(T_INV_MS);
  }

  digitalWrite(IN1, dirNueva > 0 ? HIGH : LOW);
  digitalWrite(IN2, dirNueva > 0 ? LOW : HIGH);
  pwmWrite((uint8_t)abs(pwm));
  pwmActual = pwm;
  dirActual = dirNueva;
  /*if(pwm == 0){
    detener();
    dirActual = 0;
    return;
  }

  if (dirActual != 0 && dirNueva != dirActual) {
    detener();
    delay(50);  // o T_INV_MS
  }

  if(pwm > 0){
      digitalWrite(IN1,  HIGH );
      digitalWrite(IN2,  LOW );
  }

  if(pwm < 0){
     digitalWrite(IN1,  LOW );
     digitalWrite(IN2,  HIGH );
  }

  pwmWrite((uint8_t)abs(pwm));
  pwmActual = pwm;
  dirActual = dirNueva;*/

}

// ---------------- Ensayo ----------------
bool activo = false;
uint16_t punto = 0;
uint8_t fase = 0;
uint32_t tInicio = 0, tFase = 0, tSample = 0;
int32_t cuentasPrev = 0;
float omegaFilt = 0.0f;
bool primerFiltro = true;
const float ALPHA = 0.25f;

void anunciarPunto() {
  Serial.printf("STATUS,STEP,%u,%d,%.3f\n",
                punto, barrido[punto], 100.0f * barrido[punto] / 255.0f);
}

void iniciar() {
  detener();
  reiniciarCuentas();
  cuentasPrev = 0;
  omegaFilt = 0.0f;
  primerFiltro = true;
  punto = 0;
  fase = 0;
  tInicio = tFase = tSample = millis();
  activo = true;
  Serial.println("STATUS,BEGIN");
  Serial.println("HEADER,t_s,step,phase,pwm,pwm_pct,dt_s,delta_counts,omega_raw,omega_filt,rpm,total_counts");
  Serial.print("PWM:"); 
  Serial.println(barrido[punto]);
  mandarPWM(barrido[punto]);
  anunciarPunto();
}

void finalizar(const char* causa) {
  detener();
  activo = false;
  Serial.print("STATUS,END,");
  Serial.println(causa);
}

void actualizarFase() {
  uint32_t ahora = millis();
  if (fase == 0 && ahora - tFase >= T_EST_MS) {
    fase = 1;
    tFase = ahora;
    Serial.printf("STATUS,MEASURE,%u,%d\n", punto, pwmActual);
     Serial.print("PWM:"); 
    Serial.println(pwmActual);
  } else if (fase == 1 && ahora - tFase >= T_MED_MS) {
    punto++;
    if (punto >= N_PUNTOS) {
      finalizar("COMPLETE");
    } else {
      fase = 0;
      tFase = ahora;
      mandarPWM(barrido[punto]);
      anunciarPunto();
    }
  }
}

void muestrear() {
  uint32_t ahora = millis();
  if (ahora - tSample < T_SAMPLE_MS) return;

  float dt = (ahora - tSample) / 1000.0f;
  tSample = ahora;
  int32_t n = leerCuentas();
  int32_t dn = n - cuentasPrev;
  cuentasPrev = n;

  float omega = SIGNO_ENCODER * dn * 2.0f * PI / (CPR * dt);
  if (primerFiltro) {
    omegaFilt = omega;
    primerFiltro = false;
  } else {
    omegaFilt = ALPHA * omega + (1.0f - ALPHA) * omegaFilt;
  }

  Serial.printf("DATA,%.3f,%u,%u,%d,%.3f,%.4f,%ld,%.6f,%.6f,%.4f,%ld\n",
    (ahora - tInicio) / 1000.0f, punto, fase, pwmActual,
    100.0f * pwmActual / 255.0f, dt, (long)dn, omega, omegaFilt,
    omegaFilt * 60.0f / (2.0f * PI), (long)n);
}

void leerComando() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  if ((cmd == "START" || cmd == "S") && !activo) iniciar();
  if (cmd == "STOP" || cmd == "X") finalizar("STOP");
}


// ---------------- Función de Prueba de Giro ----------------
void probarAmbosSentidos() {
  Serial.println("FORZANDO SENTIDO POSITIVO DIRECTO...");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  pwmWrite(180);
  delay(3000);

  Serial.println("PARADA...");
  detener();
  delay(1000);

  Serial.println("FORZANDO SENTIDO NEGATIVO DIRECTO...");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  pwmWrite(180);
  delay(3000);

  detener();
}

// ---------------- Inicio ----------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(30);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  abAnterior = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrEncoder, CHANGE);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if (!ledcAttach(ENA, PWM_FREQ, PWM_BITS)) {
    Serial.println("ERROR,PWM"); while (true) delay(1000);
  }
#else
  ledcSetup(PWM_CH, PWM_FREQ, PWM_BITS);
  ledcAttachPin(ENA, PWM_CH);
#endif

  detener();
  Serial.println("STATUS,READY");
}

void loop() {
  leerComando();
  if (activo) {
    actualizarFase();
    if (activo) muestrear();
  }
}
