#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Inisialisasi objek
Adafruit_MPU6050 mpu;
Servo servo1, servo2, servo3, servo4, servo5;

// Pin definitions
#define SERVO1_PIN 16
#define SERVO2_PIN 17
#define SERVO3_PIN 25
#define SERVO4_PIN 26
#define SERVO5_PIN 27
#define PIR_PIN 23

// Variabel global
const int INITIAL_POS = 90;  
const int DETECTION_POS = 45; 
float prevYaw = 0;
bool yawRotating = false;
unsigned long yawStopTime = 0;
bool pirDetected = false;
unsigned long pirDetectTime = 0;

// Variabel untuk menyimpan posisi servo terakhir
int lastPos1 = INITIAL_POS;
int lastPos2 = INITIAL_POS;
int lastPos3 = INITIAL_POS;
int lastPos4 = INITIAL_POS;
int lastPos5 = INITIAL_POS;

void setup() {
  Serial.begin(115200);
  
  // Inisialisasi I2C untuk MPU6050
  Wire.begin();
  
  // Inisialisasi MPU6050
  if (!mpu.begin()) {
    Serial.println("Gagal mendeteksi MPU6050!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 berhasil diinisialisasi!");
  
  // Konfigurasi MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Inisialisasi servo
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);
  
  // Set semua servo ke posisi initial
  resetAllServos();
  
  // Inisialisasi PIR sensor
  pinMode(PIR_PIN, INPUT);
  
  delay(100);
}

void loop() {
  // Baca sensor PIR
  int pirState = digitalRead(PIR_PIN);
  
  // Deteksi gerakan eksternal (kondisi 4)
  if (pirState == HIGH && !pirDetected) {
    pirDetected = true;
    pirDetectTime = millis();
    handlePIRDetection();
    return; 
  }
  
  // Reset flag PIR setelah servo kembali ke posisi awal
  if (pirDetected && millis() - pirDetectTime > 2000) {
    pirDetected = false;
  }
  
  // Jika sedang dalam mode PIR detection, skip MPU processing
  if (pirDetected) {
    return;
  }
  
  // Baca data dari MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Hitung sudut Roll dan Pitch dari accelerometer
  float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + 
                      a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  
  // Hitung Yaw dari gyroscope
  static float yaw = 0;
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  yaw += g.gyro.z * 180.0 / PI * dt;
  lastTime = currentTime;
  
  // Batasi range -90 hingga 90 derajat
  roll = constrain(roll, -90, 90);
  pitch = constrain(pitch, -90, 90);
  yaw = constrain(yaw, -90, 90);
  
  // Proses kondisi 1: Roll control untuk servo 1 dan 2
  handleRoll(roll);
  
  // Proses kondisi 2: Pitch control untuk servo 3 dan 4
  handlePitch(pitch);
  
  // Proses kondisi 3: Yaw control untuk servo 5
  handleYaw(yaw);
  
  // Debug output
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" | Pitch: "); Serial.print(pitch);
  Serial.print(" | Yaw: "); Serial.print(yaw);
  Serial.print(" | PIR: "); Serial.println(pirState);
  
  delay(50);
}

// Fungsi untuk menangani rotasi Roll (kondisi 1)
void handleRoll(float roll) {
  
  // Konversi roll ke posisi servo (berlawanan arah)
  int servoPos = INITIAL_POS - roll; 
  servoPos = constrain(servoPos, 0, 180);
  
  servo1.write(servoPos);
  servo2.write(servoPos);
  
  lastPos1 = servoPos;
  lastPos2 = servoPos;
}

// Fungsi untuk menangani rotasi Pitch (kondisi 2)
void handlePitch(float pitch) {
  
  // Konversi pitch ke posisi servo (searah)
  int servoPos = INITIAL_POS + pitch; 
  servoPos = constrain(servoPos, 0, 180);
  
  servo3.write(servoPos);
  servo4.write(servoPos);
  
  lastPos3 = servoPos;
  lastPos4 = servoPos;
}

// Fungsi untuk menangani rotasi Yaw (kondisi 3)
void handleYaw(float yaw) {
  // Deteksi apakah ada perubahan yaw
  float yawChange = abs(yaw - prevYaw);
  
  if (yawChange > 2) { 
    yawRotating = true;
    
    // Servo 5 mengikuti arah rotasi yaw
    int servoPos = INITIAL_POS + yaw;
    servoPos = constrain(servoPos, 0, 180);
    servo5.write(servoPos);
    lastPos5 = servoPos;
    
    yawStopTime = 0; // Reset timer
  } else if (yawRotating && yawStopTime == 0) {
    // Rotasi berhenti, mulai timer 1 detik
    yawStopTime = millis();
  }
  
  // Setelah 1 detik diam, kembali ke posisi initial
  if (yawStopTime > 0 && millis() - yawStopTime >= 1000) {
    servo5.write(INITIAL_POS);
    lastPos5 = INITIAL_POS;
    yawRotating = false;
    yawStopTime = 0;
    yaw = 0; // Reset yaw
  }
  
  prevYaw = yaw;
}

// Fungsi untuk menangani deteksi PIR 
void handlePIRDetection() {
  Serial.println("Gerakan eksternal terdeteksi!");
  
  // Semua servo bergerak bersamaan
  servo1.write(DETECTION_POS);
  servo2.write(DETECTION_POS);
  servo3.write(DETECTION_POS);
  servo4.write(DETECTION_POS);
  servo5.write(DETECTION_POS);
  
  delay(1000); 
  
  // Kembali ke posisi initial secara bersamaan
  resetAllServos();
  
  Serial.println("Kembali ke posisi initial");
}

// Fungsi untuk mereset semua servo ke posisi initial
void resetAllServos() {
  servo1.write(INITIAL_POS);
  servo2.write(INITIAL_POS);
  servo3.write(INITIAL_POS);
  servo4.write(INITIAL_POS);
  servo5.write(INITIAL_POS);
  
  lastPos1 = INITIAL_POS;
  lastPos2 = INITIAL_POS;
  lastPos3 = INITIAL_POS;
  lastPos4 = INITIAL_POS;
  lastPos5 = INITIAL_POS;
}


