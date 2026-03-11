#include "Wire.h"

// ==== Pines según PCB ====
#define ENC_A     3   // Encoder canal A
#define ENC_B     4   // Encoder canal B
#define M1_IN1    0   // TB6612 AIN1
#define M1_IN2    1   // TB6612 AIN2
#define PWM_PIN   2   // TB6612 PWMA

#define I2C_ADDR 0x21 // Dirección I2C (0x20, 0x21, 0x22)

// ==== Motor JGA25-370 6V 130RPM ====
const int ENCODER_PULSOS = 11;       // Pulsos por vuelta del motor
const int RELACION = 50;             // Reducción 50:1
const int PULSOS_POR_VUELTA = ENCODER_PULSOS * RELACION;  // 550
const uint16_t PERIODO_MS = 50;      // Tiempo de muestreo
float rpm_medida = 0.0f;
float rpm_msg = 0.0f;

// ==== PID ====
float Kp = 11.0, Ki = 5.3, Kd = 0.006;
float sp_rpm = 0.0;
float sp_rpmm = 0.0;
float err = 0, err_prev = 0, integ = 0;
float dt = PERIODO_MS / 1000.0f;

const int PWM_MAX = 255;
const int PWM_MIN_ACTIVO = 80;

// ==== Encoder ====
volatile int32_t contador_pulsos = 0;
volatile int sentido_dir = 1;  

void IRAM_ATTR isr_encoderA() {
  int b = digitalRead(ENC_B);
  sentido_dir = (b ? 1 : -1);
  contador_pulsos++;
}

// ==== Motor ====
void motor_drive(int pwm_val, bool sentido) {
  pwm_val = constrain(pwm_val, 0, PWM_MAX);
  digitalWrite(M1_IN1, !sentido);
  digitalWrite(M1_IN2, sentido);
  analogWrite(PWM_PIN, pwm_val);
}

void motor_brake() {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  analogWrite(PWM_PIN, 0);
}

// ==== Setup ====
void setup() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), isr_encoderA, RISING);

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.begin(115200);
}

// ==== Loop ====
unsigned long prevMillis = 0;
void loop() {
  unsigned long now = millis();
  if (now - prevMillis >= PERIODO_MS) {
    prevMillis = now;

    // Leer pulsos en la ventana
    noInterrupts();
    int pulsos = contador_pulsos;
    contador_pulsos = 0;
    interrupts();

    float rpm_instant = (pulsos * (60000.0f / PERIODO_MS)) / PULSOS_POR_VUELTA;
    rpm_medida = 0.3 * rpm_instant + 0.7 * rpm_medida;
    rpm_msg = rpm_medida * sentido_dir;

    // === PID ===
    err = sp_rpm - rpm_medida;
    integ += err * dt;
    float deriv = (err - err_prev) / dt;
    err_prev = err;

    float u = Kp * err + Ki * integ + Kd * deriv;

    int pwm_cmd = constrain((int)u, 0, PWM_MAX);
    if (pwm_cmd > 0 && pwm_cmd < PWM_MIN_ACTIVO) pwm_cmd = PWM_MIN_ACTIVO;

    if (sp_rpm == 0) motor_brake();
    else motor_drive(pwm_cmd, sp_rpmm > 0);

    Serial.print("sp_rpm: ");   Serial.print(sp_rpm);   Serial.print("\t");
    Serial.print("rpm_medida: ");   Serial.print(rpm_medida); Serial.print("\t");
    Serial.print("pwm_cmd: ");   Serial.println(pwm_cmd);
  }
}

void onRequest() {
  Wire.write((uint8_t *)&rpm_msg, sizeof(rpm_msg)); // enviar float
}

void onReceive(int len) {
  if (len == sizeof(float)) {
    Wire.readBytes((uint8_t *)&sp_rpmm, sizeof(float)); // recibir float
    sp_rpm = abs(sp_rpmm);
  } else {
    while (Wire.available()) Wire.read();
  }
}