## Código ESP32

Código base compartido por los robots móviles para controlar su desplazamiento mediante un controlador PID. 


### Movil 1 (USB-C)
MotorPID motorA(
  SENSOR_A_MA,
  SENSOR_B_MA,
  ENA,
  IN1,
  IN2,
  CAL_EMG30_M1,
  INVERTIR_ENCODER_A,
  INVERTIR_SALIDA_A,
  2.8,    // Kp
  1.8,    // Ki
  0.02  // Kd
);

MotorPID motorB(
  SENSOR_A_MB,
  SENSOR_B_MB,
  ENB,
  IN3,
  IN4,
  CAL_EMG30_M2,
  INVERTIR_ENCODER_B,
  INVERTIR_SALIDA_B,
  2,    // Kp
  1,    // Ki
  0.001  // Kd
);

### Movil 2 (USB-A)

MotorPID motorA(
  SENSOR_A_MA,
  SENSOR_B_MA,
  ENA,
  IN1,
  IN2,
  CAL_EMG30_M1,
  INVERTIR_ENCODER_A,
  INVERTIR_SALIDA_A,
  2,    // Kp
  1,    // Ki
  0.001  // Kd
);

MotorPID motorB(
  SENSOR_A_MB,
  SENSOR_B_MB,
  ENB,
  IN3,
  IN4,
  CAL_EMG30_M2,
  INVERTIR_ENCODER_B,
  INVERTIR_SALIDA_B,
  3.3,    // Kp
  2,    // Ki
  0.03  // Kd
);

