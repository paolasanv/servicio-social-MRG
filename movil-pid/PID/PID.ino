//******Variables de control de tiempo**************************************
#define ADC 35 // Entrada de la señal analógica del potenciómetro
#define SensorA 15 // Entrada de la señal A del encoder
#define SensorB 4 // Entrada de la señal B del encoder
#define ENA 19 // Salida PWM del habilitador de MOTOR A de L298N (ENA)
#define IN1 26 // Salida de control de MOTOR A de L298N (IN1)
#define IN2 27 // Salida de control de MOTOR A de L298N (IN2)

//******Variables de control de tiempo**************************************
volatile int Cambios = 0; // Contador de cambios del encoder
long Intervalo = 100.00; // Intervalo del tiempo de muestreo(miliseg.)
unsigned long Tiempo_inicial = 0; // Tiempo inicial de conteo de RPM (miliseg.)
unsigned long Tiempo_actual; // Tiempo relativo al tiempo de ejecución actual
unsigned long Ti = millis(); // Tiempo inicial de muestreo de control PID

//******Variables de control de PWM*****************************************
int canal = 0; // Canal de salida de PWM
int frecuencia = 1000; // Frecuencia de salida de PWM (Hz)
int resolucion = 8; // Tamaño de resolución de PWM (8 bits[255])
float MaxRPM = 188; // Valor de velocidad máxima (RPM)
float MaxOmega = MaxRPM * PI / 30.0; // Valor de velocidad angular máxima (rad/s)

//******Variables de control de PID*****************************************
float SetPoint; // Valor de referencia: SP
float PV; // Variable de proceso: PV
float CV; // Valor correctivo: cv(n)
float CV_Anterior; // Variable de corrección anterior cv(n-1)
float error; // Variable de error: e(n)
float error1; // Variable de error pasado: e(n-1)
float error2; // Variable de error antepasado: e(n-2)
float Kp = 1.200; // Constante de Proporcionalidad: Kp
float Ki = 2.850; // Constante de Integración: Ki
float Kd = 0.001; // Constante derivativa: Kd
float Ts = 0.100; // Tiempo de muestreo: Ts


//****** Subrutina de interrupciones globales ******************************
void IRAM_ATTR Encoder(){Cambios++;} //Función de interrupción externa


//****** Configuración**************************************
void setup() {
    Serial.begin(115200); //Inicializador del puerto serial a 115200 baudios
    pinMode(ADC,INPUT); // Pin de entrada señal analógica del potenciómetro
    pinMode(SensorA, INPUT); // Pin de entrada de la señal del sensor A
    pinMode(SensorB, INPUT); // Pin de entrada de la señal del sensor B
    pinMode(ENA,OUTPUT); // Pin de salida PWM para el motor
    pinMode(IN1,OUTPUT); // Pin de salida para control de giro del driver L298
    pinMode(IN2,OUTPUT); // Pin de salida para control de giro del driver L298
    attachInterrupt(digitalPinToInterrupt(SensorA),Encoder,CHANGE); //ISR del sensor A
    attachInterrupt(digitalPinToInterrupt(SensorB),Encoder,CHANGE); //ISR del sensor B
    ledcAttach(ENA, frecuencia, resolucion);
    digitalWrite(ENA,0); digitalWrite(IN1,0); digitalWrite(IN2,1); //Estado inicial
}

//****** Rutina de ciclo principal **************************************
void loop() {
Tiempo_actual = millis(); //Tiempo actual de muestreo
    if((Tiempo_actual-Tiempo_inicial)>=Intervalo){ //Condición de intervalo de tiempo
        Tiempo_inicial = Tiempo_actual; //Reinicia el tiempo inicial
        PV = (1000/Intervalo)*(Cambios)*(2.0*PI/360.00);  //Calcula rad/s en función de cambios
        Cambios = 0; //Reinicia el conteo de cambios
    }
    if ((millis()-Ti)>= (Ts*1000)){ //Calcula el tiempo de muestreo PID
        PID(); //Ejecuta subrutina de control PID
        Imprime(); //Ejecuta subrutina de impresión
    }
}

//****** Subrutina del controlador PID **************************************
void PID(){
    Ti=millis(); //Tiempo actual de muestreo
    SetPoint = analogRead(ADC)*(MaxOmega/4095.00); //Lee el pin analógico(SP)
    error = SetPoint-PV; //Calcula error e(n)
    //Ecuación de diferencias
    CV=CV_Anterior+(Kp + Kd/Ts)*error+(-Kp+Ki*Ts-2*Kd/Ts)*error1+(Kd/Ts)*error2;
    CV_Anterior = CV; //Registra corrección anterior CV(n-1)
    error2 = error1; //Registra error antepasado e(n-2)
    error1 = error; //Registra error pasado e(n-1)

    if(CV > 500.00) CV = 500.00; //Condición máxima de valor correctivo
    if(CV < 30.00) CV = 0.00; //Condición mínima de valor correctivo
   
    int16_t WC = abs(escalar_pwm(CV));

    if(SetPoint == 0){  //Condición de apagado
        ledcWrite(ENA, 0);
    }
    else{ 
        ledcWrite(ENA, WC); //Activación de motor
    }
}

// **********
int16_t escalar_pwm(float omega) { //<--- nuevo 
  //if (fabs(omega) <= 0.05f) return 0;
  float pwm;
  if (omega > 0.0f) {
    pwm = (omega - (-23.2700809430)) / 0.1757770768;
    if (pwm < 140) pwm = 140;
  } else {
    pwm = (omega - 24.2606930126) / 0.1797019906;
    if (pwm > -140) pwm = -140;
  }
  return (int16_t) round(constrain(pwm, -255, 255));
}

//****** Subrutina de monitoreo **************************************
    void Imprime(){
    Serial.print("SP:"); // Etiqueta de variable de referencia (Set Point)
    Serial.print(SetPoint); // Valor de la variable Set Point
    Serial.print(", PV:"); // Etiqueta de variable de Proceso
    Serial.print(PV); // Valor de la variable de proceso (PV)
    Serial.print(", m:"); // Etiqueta de ajuste mínimo para gráfica.
    Serial.print(0); // Valor mínimo para grafica de serial plotter
    Serial.print(", M:"); // Etiqueta de ajuste máximo para gráfica.
    Serial.print(200); // Valor máximo para grafica de serial plotter
    Serial.println(); // Salto de línea
}

