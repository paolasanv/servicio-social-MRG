/*
 Funcion generada por la caracterizacion del EMG30.
 Entrada: omegaDeseada en rad/s. Salida: PWM firmado.
*/
const float A_POS = 0.1757770768f;
const float B_POS = -23.2700809430f;
const float A_NEG = 0.1797019906f;
const float B_NEG = 24.2606930126f;
const int PWM_MIN_POS = 140;
const int PWM_MIN_NEG = -140;

int16_t pwmDesdeOmega(float omegaDeseada) {
  if (fabs(omegaDeseada) <= 0.05f) return 0;
  float pwm;
  if (omegaDeseada > 0.0f) {
    pwm = (omegaDeseada - B_POS) / A_POS;
    if (pwm < PWM_MIN_POS) pwm = PWM_MIN_POS;
  } else {
    pwm = (omegaDeseada - B_NEG) / A_NEG;
    if (pwm > PWM_MIN_NEG) pwm = PWM_MIN_NEG;
  }
  return (int16_t)round(constrain(pwm, -255.0f, 255.0f));
}
