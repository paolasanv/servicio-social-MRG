/*
  ESP32 + PCA9685 + Servos Power HD 1501MG
  Cinemática inversa para manipulador serial de 5 GDL.

  CORRECCIÓN IMPORTANTE:
  - La junta 2 usa DOS servos.
  - Ambos servos de la junta 2 reciben el MISMO ángulo.

  Entrada por Monitor Serial:
    x y z
    x y z phi_deg csi_deg

  Unidades:
    x, y, z en cm
    phi_deg y csi_deg en grados

  Ejemplos:
    20 0 10
    20 0 10 0 0

  Comandos:
    home
    open
    close
    help
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
  90.0,  // J2 hombro a 90° es el cero mecanico
  125.0,  // J3 codo a 40° es el cero mecanico
  110.0,  // J4 muñeca pitch a 110° es el cero mecanico
  90.0   // J5 muñeca giro
};

float SERVO_DIR[N_JOINTS] = {
  1.0,
  1.0,
  1.0,
  1.0,
  1.0
};

// Límites seguros de comando para servos.
// Empieza con límites conservadores para evitar golpes mecánicos.

float SERVO_MIN_DEG[N_JOINTS] = {
  10.0, 10.0, 10.0, 10.0, 10.0
};

float SERVO_MAX_DEG[N_JOINTS] = {
  170.0, 170.0, 150.0, 170.0, 170.0
};

// Pulsos típicos para servos estándar.
// Ajusta si el servo no llega correctamente a 0° o 180°.

//const float SERVO_MIN_US = 500.0;
//const float SERVO_MAX_US = 2500.0;

float SERVO_MIN_US[N_JOINTS] = {
  600, 600, 500, 500, 500
};

float SERVO_MAX_US[N_JOINTS] = {
  2000, 2400, 2500, 2500, 2500
};

// Estado actual de las 5 juntas en grados de servo
float currentServoDeg[N_JOINTS] = {
   90.0, 90.0, 125.0, 110.0, 90.0
};

// =====================================================
// ========== GRIPPER OPCIONAL ==========================
// =====================================================

const float GRIPPER_CLOSE_DEG = 0.0;
const float GRIPPER_OPEN_DEG  = 55.0;

// =====================================================
// ========== FUNCIONES AUXILIARES ======================
// =====================================================

float degToRad(float deg) {
  return deg * PI / 180.0;
}

float radToDeg(float rad) {
  return rad * 180.0 / PI;
}

float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t microsecondsToTicks(float us) {
  float ticks = us * 4096.0 * SERVO_FREQ / 1000000.0;
  ticks = clampFloat(ticks, 0.0, 4095.0);
  return (uint16_t)(ticks + 0.5);
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

float jointRadToServoDeg(uint8_t jointIndex, float thetaRad) {
  float thetaDeg = radToDeg(thetaRad);
  float servoDeg = SERVO_ZERO_DEG[jointIndex] + SERVO_DIR[jointIndex] * thetaDeg;
  return servoDeg;
}

bool isServoCommandSafe(uint8_t jointIndex, float servoDeg) {
  return servoDeg >= SERVO_MIN_DEG[jointIndex] &&
         servoDeg <= SERVO_MAX_DEG[jointIndex];
}

// =====================================================
// ========== ENVÍO A SERVOS ============================
// =====================================================
//
// Esta función es donde se corrige lo de la junta 2:
// J2A y J2B reciben exactamente el mismo comando cmdJ2.
//
// Si J2A tiene angulo X entonces J2B tiene angulo (180 - X)
//

void sendJointAnglesToServos(float servoDeg[N_JOINTS]) {
  float cmdJ1 = servoDeg[0];
  float cmdJ2 = servoDeg[1];
  float cmdJ3 = servoDeg[2];
  float cmdJ4 = servoDeg[3];
  float cmdJ5 = servoDeg[4];

  writeServoAngle(J1_CH, cmdJ1);

  // Junta 2 con DOS servos:
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
      intermediate[i] = currentServoDeg[i] +
                        alpha * (targetDeg[i] - currentServoDeg[i]);
    }

    sendJointAnglesToServos(intermediate);
    delay(dtMs);
  }

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    currentServoDeg[i] = targetDeg[i];
  }
}

void goHome() {
  float home[N_JOINTS] = {
    90.0, 90.0, 125.0, 110.0, 90.0
  };

  Serial.println("Moviendo a HOME...");
  moveServosSmooth(home, 50, 20);
  Serial.println("HOME listo.");
}

void openGripper() {
  Serial.println("Abriendo gripper...");
  writeServoAngle(GRIPPER_CH, GRIPPER_OPEN_DEG);
}

void closeGripper() {
  Serial.println("Cerrando gripper...");
  writeServoAngle(GRIPPER_CH, GRIPPER_CLOSE_DEG);
}

// =====================================================
// ========== CINEMÁTICA INVERSA ========================
// =====================================================
//
// Entrada:
//   px, py, pz en cm
//   phi, csi en radianes
//
// Salida:
//   theta[0] = theta1
//   theta[1] = theta2
//   theta[2] = theta3
//   theta[3] = theta4
//   theta[4] = theta5

bool inverseKinematics(
  float px,
  float py,
  float pz,
  float phi,
  float csi,
  float theta[N_JOINTS],
  String &errorMsg
) {

  if (px < 8 || px > 24) return false;
  if (abs(py) > 18) return false;
  if (pz < 0 || pz > 22) return false;

  float a = cos(phi);
  float b = sin(phi);

  float rr = sqrt(px * px + py * py);

  // Punto de muñeca
  float x = rr - f * a;
  float z = pz + f * b;

  float dz = z - h;
  float R = sqrt(x * x + dz * dz);

  if (R < 0.0001) {
    errorMsg = "Error: configuracion singular. R demasiado pequeno.";
    return false;
  }

  if (R > (k + g) || R < fabs(k - g)) {
    errorMsg = "Error: punto fuera del espacio de trabajo.";
    return false;
  }

  float sin_a = dz / R;
  float cos_a = x / R;

  float cos_b = (R * R + g * g - k * k) / (2.0 * g * R);
  float cos_g = (k * k + g * g - R * R) / (2.0 * g * k);
  

  if (cos_b < -1.0001 || cos_b > 1.0001 ||
      cos_g < -1.0001 || cos_g > 1.0001) {
    errorMsg = "Error: solucion trigonometrica invalida.";
    return false;
  }

  cos_b = clampFloat(cos_b, -1.0, 1.0);
  cos_g = clampFloat(cos_g, -1.0, 1.0);

  float sin_b = sqrt(1.0 - cos_b * cos_b); 
  float sin_g = sqrt(1.0 - cos_g * cos_g);

  float th1 = atan2(py, px);

  float th2 = atan2(
    sin_a * cos_b + cos_a * sin_b,
    -(cos_a * cos_b - sin_a * sin_b)
  );

  float th3 = atan2(
    sin_g,
    -cos_g
  );

  float c = cos(th2);
  float d = sin(th2);

  float th4 = atan2(
    a * (sin_g * c - cos_g * d) + b * (cos_g * c + sin_g * d),
    a * (cos_g * c + sin_g * d) - b * (sin_g * c - cos_g * d)
  );

  float th5 = csi;

  theta[0] = th1;
  theta[1] = th2;
  theta[2] = th3;
  theta[3] = th4;
  theta[4] = th5;

  errorMsg = "";
  return true;
}

// =====================================================
// ========== EJECUCIÓN DE POSICIÓN XYZ =================
// =====================================================

void executeXYZ(float px, float py, float pz, float phiDeg, float csiDeg) {
  float phi = degToRad(phiDeg);
  float csi = degToRad(csiDeg);

  float theta[N_JOINTS];
  String errorMsg;

  Serial.println();
  Serial.println("Calculando cinematica inversa...");
  Serial.print("Objetivo [cm]: x = ");
  Serial.print(px, 3);
  Serial.print(" , y = ");
  Serial.print(py, 3);
  Serial.print(" , z = ");
  Serial.print(pz, 3);
  Serial.print(" , phi = ");
  Serial.print(phiDeg, 2);
  Serial.print(" deg , csi = ");
  Serial.print(csiDeg, 2);
  Serial.println(" deg");

  bool ok = inverseKinematics(px, py, pz, phi, csi, theta, errorMsg);

  if (!ok) {
    Serial.println(errorMsg);
    return;
  }

  float servoTarget[N_JOINTS];

  Serial.println();
  Serial.println("Angulos articulares calculados:");

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    Serial.print("theta");
    Serial.print(i + 1);
    Serial.print(" = ");
    Serial.print(radToDeg(theta[i]), 3);
    Serial.println(" deg");

    servoTarget[i] = jointRadToServoDeg(i, theta[i]);
  }

  Serial.println();
  Serial.println("Comandos enviados a servos:");

  Serial.print("J1 canal ");
  Serial.print(J1_CH);
  Serial.print(" = ");
  Serial.print(servoTarget[0], 3);
  Serial.println(" deg");

  Serial.print("J2A canal ");
  Serial.print(J2A_CH);
  Serial.print(" = ");
  Serial.print(servoTarget[1], 3);
  Serial.println(" deg");

  Serial.print("J2B canal ");
  Serial.print(J2B_CH);
  Serial.print(" = ");
  Serial.print(servoTarget[1], 3);
  Serial.println(" deg ");

  Serial.print("J3 canal ");
  Serial.print(J3_CH);
  Serial.print(" = ");
  Serial.print(servoTarget[2], 3);
  Serial.println(" deg");

  Serial.print("J4 canal ");
  Serial.print(J4_CH);
  Serial.print(" = ");
  Serial.print(servoTarget[3], 3);
  Serial.println(" deg");

  Serial.print("J5 canal ");
  Serial.print(J5_CH);
  Serial.print(" = ");
  Serial.print(servoTarget[4], 3);
  Serial.println(" deg");

  for (uint8_t i = 0; i < N_JOINTS; i++) {
    if (!isServoCommandSafe(i, servoTarget[i])) {
      Serial.println();
      Serial.print("ERROR: J");
      Serial.print(i + 1);
      Serial.println(" fuera de limites seguros.");
      Serial.println("Ajusta SERVO_ZERO_DEG, SERVO_DIR o los limites mecanicos.");
      return;
    }
  }

  Serial.println();
  Serial.println("Moviendo brazo...");
  moveServosSmooth(servoTarget, 50, 20);
  Serial.println("Movimiento terminado.");
}

// =====================================================
// ========== MONITOR SERIAL ============================
// =====================================================

void printHelp() {
  Serial.println();
  Serial.println("==============================================");
  Serial.println("CONTROL IK MANIPULADOR 5 GDL");
  Serial.println("ESP32 + PCA9685 + Power HD 1501MG");
  Serial.println();
  Serial.println("Entrada en cm:");
  Serial.println("  x y z");
  Serial.println("Ejemplo:");
  Serial.println("  20 0 10");
  Serial.println();
  Serial.println("Entrada con orientacion:");
  Serial.println("  x y z phi_deg csi_deg");
  Serial.println("Ejemplo:");
  Serial.println("  20 0 10 0 0");
  Serial.println();
  Serial.println("Comandos:");
  Serial.println("  home");
  Serial.println("  open");
  Serial.println("  close");
  Serial.println("  help");
  Serial.println();
  Serial.println("Nota:");
  Serial.println("  La junta 2 usa dos servos:");
  Serial.println("  J2A y J2B reciben el mismo angulo.");
  Serial.println("==============================================");
  Serial.println();
}

void processSerialLine(String line) {
  line.trim();

  if (line.length() == 0) {
    return;
  }

  String command = line;
  command.toLowerCase();

  if (command == "help") {
    printHelp();
    return;
  }

  if (command == "home") {
    goHome();
    return;
  }

  if (command == "open") {
    openGripper();
    return;
  }

  if (command == "close") {
    closeGripper();
    return;
  }

  float px = 0.0;
  float py = 0.0;
  float pz = 0.0;
  float phiDeg = DEFAULT_PHI_DEG;
  float csiDeg = DEFAULT_CSI_DEG;

  int n = sscanf(
    line.c_str(),
    "%f %f %f %f %f",
    &px,
    &py,
    &pz,
    &phiDeg,
    &csiDeg
  );

  if (n == 3) {
    executeXYZ(px, py, pz, DEFAULT_PHI_DEG, DEFAULT_CSI_DEG);
  }
  else if (n == 5) {
    executeXYZ(px, py, pz, phiDeg, csiDeg);
  }
  else {
    Serial.println("Entrada no valida.");
    Serial.println("Usa:");
    Serial.println("  x y z");
    Serial.println("o:");
    Serial.println("  x y z phi_deg csi_deg");
    Serial.println("Ejemplo:");
    Serial.println("  20 0 10");
  }
}

// =====================================================
// ========== SETUP / LOOP ==============================
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Iniciando ESP32 + PCA9685...");

  Wire.begin(SDA_PIN, SCL_PIN);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  delay(500);

  goHome();
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    processSerialLine(line);
  }
}
