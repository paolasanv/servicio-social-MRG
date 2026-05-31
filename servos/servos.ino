/*

Servo 4 -> const float SERVO_MIN_US = 197.0; // 800 us -> 0.8 ms
          const float SERVO_MAX_US = 540.0; //2200 us -> 2.2 ms

Servo 3 y servo 1 -> SERVOMIN_US = 500 y SERVOMAX_US -> 2500
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// =====================================================
// ========== CONFIGURACIÓN I2C / PCA9685 ===============
// =====================================================

#define SDA_PIN 21
#define SCL_PIN 22

#define PCA9685_ADDR 0x40
#define SERVO_FREQ 60

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// =====================================================
// ========== CANALES DEL PCA9685 =======================
// =====================================================
//
// J1  -> Base
// J2A -> Hombro, servo 1
// J2B -> Hombro, servo 2
// J3  -> Codo
// J4  -> Muñeca pitch
// J5  -> Muñeca giro
// J6  -> Gripper opcional

const uint8_t J1_CH  = 0;
const uint8_t J2A_CH = 1;
const uint8_t J2B_CH = 2;
const uint8_t J3_CH  = 3;
const uint8_t J4_CH  = 4;
const uint8_t J5_CH  = 5;

const uint8_t GRIPPER_CH = 6;


// =====================================================
// ========== PARÁMETROS GEOMÉTRICOS DEL BRAZO ==========
// =====================================================
// Unidades: cm

const float h = 9.3;    // altura de la base
const float k = 15.0;   // eslabón 1
const float g = 15.0;   // eslabón 2
const float f = 11.7;   // longitud muñeca / efector

// Orientación por defecto del efector
const float DEFAULT_PHI_DEG = 0.0;
const float DEFAULT_CSI_DEG = 0.0;

// =====================================================
// ========== CALIBRACIÓN DE SERVOS =====================
// =====================================================
//
// SERVO_ZERO_DEG:
// Ángulo físico del servo cuando la junta teórica vale 0 grados.
//
// SERVO_DIR:
//  1.0 si el servo gira en el mismo sentido que la junta.
// -1.0 si el servo gira en sentido contrario.
//
// IMPORTANTE:
// La junta 2 usa dos servos, pero ambos reciben el mismo ángulo calculado.
// Si están conectados eléctricamente al mismo cable de señal, físicamente
// ya recibirán el mismo PWM. Si están en dos canales del PCA9685, este
// código les manda el mismo valor.

const uint8_t N_JOINTS = 5;

float SERVO_ZERO_DEG[N_JOINTS] = {
  90.0,  // J1 base
  90.0,  // J2 hombro -> 90° es el cero mecanico
  125.0,  // J3 codo -> 125° es el cero mecanico
  110.0,  // J4 muñeca pitch -> 110° es el cero mecanico
  90.0   // J5 muñeca giro
};

float SERVO_DIR[N_JOINTS] = {
  1.0,   // J1
  1.0,   // J2
  1.0,   // J3
  1.0,   // J4
  1.0    // J5
};

// Límites seguros de comando para servos.
// Empieza con límites conservadores para evitar golpes mecánicos.

float SERVO_MIN_DEG[N_JOINTS] = {
  10.0, 10.0, 10.0, 10.0, 10.0
};

float SERVO_MAX_DEG[N_JOINTS] = {
  170.0, 170.0, 170.0, 170.0, 170.0
};

// Pulsos típicos para servos estándar.
// Ajusta si el servo no llega correctamente a 0° o 180°.

//const float SERVO_MIN_US = 500.0;
//const float SERVO_MAX_US = 2500.0; /

float SERVO_MIN_US[N_JOINTS] = {
  600, 600, 500, 500, 500
};

float SERVO_MAX_US[N_JOINTS] = {
  2000, 2400, 2500, 2500, 2500
};


// Estado actual de las 5 juntas en grados de servo
float currentServoDeg[N_JOINTS] = {
  90.0, 90.0, 90.0, 90.0, 90.0
};


// =====================================================
// ========== FUNCIONES AUXILIARES ======================
// =====================================================

float degToRad(float deg) { return deg * PI / 180.0; }

float radToDeg(float rad) { return rad * 180.0 / PI; }


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}


void writeServoAngle(uint8_t channel, float angleDeg) {
  angleDeg = clampFloat(angleDeg, 0.0, 180.0);

  float pulseUs = mapFloat(
    angleDeg,
    0.0,
    180.0,
    SERVO_MIN_US[channel],
    SERVO_MAX_US[channel]
  );

  uint16_t ticks = microsecondsToTicks(pulseUs);
  pwm.setPWM(channel, 0, ticks);
}

uint16_t microsecondsToTicks(float us) {
  float ticks = us * 4096.0 * SERVO_FREQ / 1000000.0;
  ticks = clampFloat(ticks, 0.0, 4095.0);
  return (uint16_t)(ticks + 0.5);
}

float jointRadToServoDeg(uint8_t jointIndex, float thetaRad) {
  float thetaDeg = radToDeg(thetaRad);
  float servoDeg = SERVO_ZERO_DEG[jointIndex] + SERVO_DIR[jointIndex] * thetaDeg;
  return servoDeg;
}

// =====================================================
// ========== ENVÍO A SERVOS ============================
// =====================================================

void sendJointAnglesToServos(float servoDeg[N_JOINTS]) {
  float cmdJ1 = servoDeg[0];
  float cmdJ2 = servoDeg[1];
  float cmdJ3 = servoDeg[2];
  float cmdJ4 = servoDeg[3];
  float cmdJ5 = servoDeg[4];

  writeServoAngle(J1_CH, cmdJ1);

  // Junta 2 con DOS servos:
  // ambos reciben el MISMO ángulo.
  writeServoAngle(J2A_CH, cmdJ2);
  writeServoAngle(J2B_CH, 180 - cmdJ2);

  writeServoAngle(J3_CH, cmdJ3);
  writeServoAngle(J4_CH, cmdJ4);
  writeServoAngle(J5_CH, cmdJ5);
}

void moveServosSmooth(float targetDeg[N_JOINTS], int steps = 50, int dtMs = 20) {
  for (int s = 1; s <= steps; s++) {
    float alpha = (float)s / (float)steps;

    float intermediate[N_JOINTS];

    for (uint8_t i = 0; i < N_JOINTS; i++) {
      intermediate[i] = currentServoDeg[i] + alpha * (targetDeg[i] - currentServoDeg[i]);
    }

    sendJointAnglesToServos(intermediate);
    delay(dtMs);
  }

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    currentServoDeg[i] = targetDeg[i]; //actualiza la posicion actual de los servos
  }
}

void goHome() {
  float home[N_JOINTS] = {
    90.0, 90.0, 90.0, 90.0, 90.0
  };

  Serial.println("Moviendo a HOME...");
  moveServosSmooth(home, 50, 20);
  Serial.println("HOME listo.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Iniciando ESP32 + PCA9685...");

  Wire.begin(SDA_PIN, SCL_PIN);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(100);

   /*float home[N_JOINTS] = {
    90.0, 90.0, 90.0, 90.0, 90.0
  };

  Serial.println("Moviendo a HOME...");
  moveServosSmooth(home, 50, 20);*/

 
  writeServoAngle(4,110);

  

   Serial.println("Movimiento de prueba");

  //writeServoAngle(0, 90);




}

void loop() {

  /*if (Serial.available()) {

    int ang = Serial.parseInt();

    writeServoAngle(J1_CH, ang);

    Serial.print("Moviendo a: ");
    Serial.println(ang);
  }*/
}







