/*  
Este código integra un MPU6050 con dos multiplexores (MUX) para leer sensores analógicos.
Basado en el ejemplo de la librería MPU6050: MPU6050_DMP6_ImuData_for_ROS

ESP32S3 Dev Module
USB CDC On Boot : "Enabled"               Para usar USB OTG

https://mischianti.org/esp32-s3-devkitc-1-high-resolution-pinout-and-specs
*/

#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ====== MPU6050 ======
MPU6050 mpu;

#define EARTH_GRAVITY_MS2 9.80665
#define DEG_TO_RAD        0.017453292519943295
#define RAD_TO_DEG        57.29577951308232
#define RGB_BRILLO        10  //0-255

bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

Quaternion q;
VectorInt16 aa, gg, aaWorld, ggWorld;
VectorFloat gravity;
float ypr[3];
float ref[3] = {0.0, 0.0, 0.0};
float vel[3];

// ====== Pines MUX1 (línea) ======
#define s1_0 46
#define s1_1 3
#define s1_2 18
#define s1_3 17
#define in1 16

// ====== Pines MUX2 (distancia) ======
#define s2_0 4
#define s2_1 5
#define s2_2 6
#define s2_3 7
#define in2 15

int linea[9];     
float dist_cm[9];

const int states[16][4] = {
  { LOW,  LOW,  LOW,  LOW }, 
  { HIGH, LOW,  LOW,  LOW }, 
  { LOW,  HIGH, LOW,  LOW }, 
  { HIGH, HIGH, LOW,  LOW }, 
  { LOW,  LOW,  HIGH, LOW }, 
  { HIGH, LOW,  HIGH, LOW }, 
  { LOW,  HIGH, HIGH, LOW }, 
  { HIGH, HIGH, HIGH, LOW }, 
  { LOW,  LOW,  LOW,  HIGH }, 
  { HIGH, LOW,  LOW,  HIGH }, 
  { LOW,  HIGH, LOW,  HIGH }, 
  { HIGH, HIGH, LOW,  HIGH }, 
  { LOW,  LOW,  HIGH, HIGH }, 
  { HIGH, LOW,  HIGH, HIGH }, 
  { LOW,  HIGH, HIGH, HIGH }, 
  { HIGH, HIGH, HIGH, HIGH }
};

// ====== Tiempo ======
unsigned long lastIMU = 0;
const unsigned long intervalIMU = 30; // 30Hz

unsigned long lastMUX = 0;
const unsigned long intervalMUX = 50; // 20Hz para MUX y sensores

unsigned long lastI2C = 0;
const unsigned long intervalI2C = 20; // 50Hz para enviar/recibir datos I2C

// ====== PROCESAMIENTO SHARP ======
float distanciaSharp(int pin) {
  int valorADC = analogRead(pin);
  float distancia_cm = 0;
  float a = 390.7143;
  float K = 14902.8564;
  const float MAX_CM = 80.0;

  if (valorADC > (a + 1.0)) { 
    distancia_cm = 0.01 * K / (valorADC - a);
  } else {
    distancia_cm = MAX_CM;
  }
  return distancia_cm;
}

// ====== LED de error ======
void error_led(){
  while(true){
    rgbLedWrite(RGB_BUILTIN, RGB_BRILLO, 0, 0);  // Red
    delay(500);
    rgbLedWrite(RGB_BUILTIN, 0, 0, 0);  // Off / black
    delay(500);
  }
}

// ====== I2C no bloqueante ======
void Send_data(float ref_val, int add) {
  Wire.beginTransmission(add);
  Wire.write((uint8_t*)&ref_val, sizeof(ref_val));
  Wire.endTransmission();
}

void Req_data(float &vel_val, int add) {
  Wire.requestFrom(add, sizeof(vel_val));
  if (Wire.available() >= sizeof(vel_val)) {
    Wire.readBytes((uint8_t*)&vel_val, sizeof(vel_val));
  }
}

// ================== SETUP ==================
void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Configurar MUX1
  pinMode(s1_0, OUTPUT); pinMode(s1_1, OUTPUT); pinMode(s1_2, OUTPUT); pinMode(s1_3, OUTPUT);
  pinMode(in1, INPUT);

  // Configurar MUX2
  pinMode(s2_0, OUTPUT); pinMode(s2_1, OUTPUT); pinMode(s2_2, OUTPUT); pinMode(s2_3, OUTPUT);
  pinMode(in2, INPUT);

  // Inicializar MPU
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    error_led();
  }
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int c1 = input.indexOf(',');
    int c2 = input.indexOf(',', c1 + 1);

    if (c1 != -1 && c2 != -1) {
      ref[0] = input.substring(0, c1).toFloat();
      ref[1] = input.substring(c1 + 1, c2).toFloat();
      ref[2] = input.substring(c2 + 1).toFloat();
    }
  }


  // ====== IMU 30Hz ======
  if (DMPReady && (now - lastIMU >= intervalIMU)) {
    lastIMU = now;

    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetAccel(&aa, FIFOBuffer);
      mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
      mpu.dmpGetGyro(&gg, FIFOBuffer);
      mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      char buffer[512];
      snprintf(buffer, sizeof(buffer),
        "{\"quat\":[%.6f,%.6f,%.6f,%.6f],"
        "\"ypr\":[%.2f,%.2f,%.2f],"
        "\"accel\":[%.2f,%.2f,%.2f],"
        "\"gyro\":[%.4f,%.4f,%.4f],"
        "\"linea\":[%d,%d,%d,%d,%d,%d,%d,%d,%d],"
        "\"dist_cm\":[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],"
        "\"vel\":[%.2f,%.2f,%.2f]}",
        q.w, q.x, q.y, q.z,
        ypr[0]*RAD_TO_DEG, ypr[1]*RAD_TO_DEG, ypr[2]*RAD_TO_DEG,
        aaWorld.x*mpu.get_acce_resolution()*EARTH_GRAVITY_MS2,
        aaWorld.y*mpu.get_acce_resolution()*EARTH_GRAVITY_MS2,
        aaWorld.z*mpu.get_acce_resolution()*EARTH_GRAVITY_MS2,
        ggWorld.x*mpu.get_gyro_resolution()*DEG_TO_RAD,
        ggWorld.y*mpu.get_gyro_resolution()*DEG_TO_RAD,
        ggWorld.z*mpu.get_gyro_resolution()*DEG_TO_RAD,
        linea[0], linea[1], linea[2], linea[3], linea[4], linea[5], linea[6], linea[7], linea[8],
        dist_cm[0], dist_cm[1], dist_cm[2], dist_cm[3], dist_cm[4], dist_cm[5], dist_cm[6], dist_cm[7], dist_cm[8],
        vel[0], vel[1], vel[2]
      );
      Serial.println(buffer);
    }
  }

  // ====== MUX 20Hz ======
  if (now - lastMUX >= intervalMUX) {
    lastMUX = now;

    for (int i = 0; i < 9; i++) {
      // MUX línea
      digitalWrite(s1_0, states[i][0]);
      digitalWrite(s1_1, states[i][1]);
      digitalWrite(s1_2, states[i][2]);
      digitalWrite(s1_3, states[i][3]);
      linea[i] = analogRead(in1);

      // MUX distancia
      digitalWrite(s2_0, states[i][0]);
      digitalWrite(s2_1, states[i][1]);
      digitalWrite(s2_2, states[i][2]);
      digitalWrite(s2_3, states[i][3]);
      dist_cm[i] = distanciaSharp(in2);
    }
  }

  // ====== I2C 50Hz ======
  if (now - lastI2C >= intervalI2C) {
    lastI2C = now;

    // Enviar referencias
    Send_data(ref[0], 0x20);
    Send_data(ref[1], 0x21);
    Send_data(ref[2], 0x22);

    // Pedir velocidades
    Req_data(vel[0], 0x20);
    Req_data(vel[1], 0x21);
    Req_data(vel[2], 0x22);
  }
}
