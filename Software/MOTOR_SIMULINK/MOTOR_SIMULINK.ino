// ==== Pines según PCB ====
#define ENC_A 3    // Encoder canal A
#define ENC_B 4    // Encoder canal B
#define M1_IN1 0   // TB6612 AIN1
#define M1_IN2 1   // TB6612 AIN2
#define PWM_PIN 2  // TB6612 PWMA

// ==== Encoder ====
volatile int32_t contador_pulsos = 0;
volatile int sentido_dir = 1;
float resolution = 550;
float rpm = 0;

// ==== COM ====
float cmd = 0;
int pwm_cmd = 0;

//Estructura Union
typedef union {
  float number;
  uint8_t bytes[4];
} valor;

//Variable Union
valor velocidad;

//Variable global de pulsos compartida con la interrupción
unsigned long timeold;

// ==== Interrupción ====
void IRAM_ATTR isr_encoderA() {
  int b = digitalRead(ENC_B);
  sentido_dir = (b ? -1 : 1);
  contador_pulsos++;
}

// ==== Motor ====
void motor_drive(int pwm_val, bool sentido) {
  digitalWrite(M1_IN1, !sentido);
  digitalWrite(M1_IN2, sentido);
  analogWrite(PWM_PIN, pwm_val);
}

void motor_brake() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  analogWrite(PWM_PIN, 0);
}

void setup() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), isr_encoderA, RISING);

  Serial.begin(115200);
  //Configurar Interrupción
  timeold = 0;
}

void loop() {
  //Pregunta si tenemos un dato a recibir
  if (Serial.available() > 0) {
    cmd = recepcion();
  }

  //Transforma el valor
  pwm_cmd = map(cmd, 0, 100, 0, 255);

  //Comando a motor
  if (pwm_cmd == 0) motor_brake();
  else motor_drive(pwm_cmd, cmd > 0);

  //Calculo de las RPM
  if (millis() - timeold >= 50) {
    noInterrupts();
    rpm = float(sentido_dir * (60.0 * 1000.0 / resolution) / (millis() - timeold) * contador_pulsos);
    timeold = millis();
    contador_pulsos = 0;
    velocidad.number = rpm;
    Serial.write('V');
    for (int i = 0; i < 4; i++) {
      Serial.write(velocidad.bytes[i]);
    }
    Serial.write('\n');
    interrupts();
  }
}

//Recibir Flotante
float recepcion() {
  int i;
  valor buf;
  for (i = 0; i < 4; i++)
    buf.bytes[i] = Serial.read();
  return buf.number;
}