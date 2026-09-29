#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// ==========================================
// 1. NETWORK CONFIGURATION
// ==========================================
const char* ssid = "QuadraConnect";
const char* password = "Da32!stob$17orsMovin2Ch!";

const char* receiverIP = "192.168.0.147"; 
const int udpPort = 5005;

WiFiUDP udp;
Adafruit_MPU6050 mpu;

// ==========================================
// 2. TIMING & HARDWARE
// ==========================================
const float SENSOR_RATE = 500.0; 
const float NETWORK_RATE = 60.0; 

unsigned long microsPerSensor, microsPreviousSensor;
unsigned long microsPerNetwork, microsPreviousNetwork;
unsigned long startTime;

float X_BIAS = 0.0;
float Y_BIAS = 0.0;
float Z_BIAS = 0.0;

// ==========================================
// 3. 6DOF QUATERNION FILTER
// ==========================================
// mq0 = W, mq1 = X, mq2 = Y, mq3 = Z
float mq0 = 1.0f, mq1 = 0.0f, mq2 = 0.0f, mq3 = 0.0f;
float twoKp = 2.0f * 0.5f; 
float twoKi = 0.0f; // Set to 0.0 to prevent high-G drift (Matches Madgwick)

void updateMahony(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = mq1 * mq3 - mq0 * mq2;
        halfvy = mq0 * mq1 + mq2 * mq3;
        halfvz = mq0 * mq0 - 0.5f + mq3 * mq3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    qa = mq0; qb = mq1; qc = mq2;
    mq0 += (-qb * gx - qc * gy - mq3 * gz);
    mq1 += (qa * gx + qc * gz - mq3 * gy);
    mq2 += (qa * gy - qb * gz + mq3 * gx);
    mq3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = 1.0f / sqrt(mq0 * mq0 + mq1 * mq1 + mq2 * mq2 + mq3 * mq3);
    mq0 *= recipNorm; mq1 *= recipNorm; mq2 *= recipNorm; mq3 *= recipNorm;
}

void setup() {
  Wire.begin(21, 22);
  Wire.setClock(400000); 

  Serial.begin(115200);
  delay(1000);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  if (!mpu.begin()) { while (1) { delay(10); } }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  // --- 3-AXIS HARDWARE CALIBRATION ---
  Serial.println("\n[SYSTEM] IMU Booting. Hands off the sensor!");
  for(int i = 3; i > 0; i--) {
      Serial.print("Calibrating in "); Serial.print(i); Serial.println("...");
      delay(1000);
  }
  
  Serial.println("[SYSTEM] Calibrating 3-Axis Gyro... DO NOT MOVE.");
  for (int i = 0; i < 10; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      delay(10);
  }

  float bx = 0, by = 0, bz = 0;
  int calib_samples = 500;
  for (int i = 0; i < calib_samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    bx += (g.gyro.x * 57.2958);
    by += (g.gyro.y * 57.2958);
    bz += (g.gyro.z * 57.2958);
    delay(5); 
  }
  X_BIAS = bx / calib_samples; Y_BIAS = by / calib_samples; Z_BIAS = bz / calib_samples;
  Serial.println("[SYSTEM] Calibration Complete.");

  microsPerSensor = 1000000 / SENSOR_RATE;
  microsPerNetwork = 1000000 / NETWORK_RATE;
  
  unsigned long now = micros();
  microsPreviousSensor = now;
  microsPreviousNetwork = now;
  startTime = millis();
}

void loop() {
  unsigned long microsNow = micros();
  
  if (microsNow - microsPreviousSensor >= microsPerSensor) {
    float dt = (microsNow - microsPreviousSensor) / 1000000.0;
    microsPreviousSensor = microsNow;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gx = (g.gyro.x * 57.2958) - X_BIAS;
    float gy = (g.gyro.y * 57.2958) - Y_BIAS;
    float gz = (g.gyro.z * 57.2958) - Z_BIAS;

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    updateMahony(gx, gy, gz, ax, ay, az, dt);
  }

  if (microsNow - microsPreviousNetwork >= microsPerNetwork) {
    microsPreviousNetwork = microsNow;

    char payload[128];
    // Payload: ms, W, X, Y, Z
    snprintf(payload, sizeof(payload), "%lu,%.4f,%.4f,%.4f,%.4f",
             millis() - startTime, mq0, mq1, mq2, mq3);

    udp.beginPacket(receiverIP, udpPort);
    udp.print(payload);
    udp.endPacket();
  }
}