// YawDriftSocket.ino -- ESP32 + MPU6050 magnetometer-free orientation node.
//
// Architecture
//   * The MPU6050 samples itself at exactly 500 Hz into its on-chip FIFO, so no
//     sample is ever lost or integrated with the wrong dt, even if the ESP32
//     stalls (the FIFO holds ~170 ms of data).
//   * loop() (core 1) drains the FIFO and runs the fusion core (imu_fusion.h) on
//     every sample with a fixed dt measured against the ESP32 crystal.
//   * A separate FreeRTOS task on core 0 (the Wi-Fi core) sends UDP packets at
//     ~60 Hz, so network hiccups can never block the math.
//
// Setup
//   1. Copy secrets.example.h to secrets.h and fill in Wi-Fi + receiver IP.
//   2. (Recommended) run tools/gyro_scale_cal.py once per board and paste the
//      matrix into calibration.h.
//
// Packet (CSV, one line):
//   ms,W,X,Y,Z,still,tempC,bias_x,bias_y,bias_z,integ_x,integ_y,integ_z,fifo_overflows
// The first five fields are unchanged from the original firmware.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp_timer.h>

#include "calibration.h"
#include "imu_fusion.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "secrets.h not found: copy secrets.example.h to secrets.h and fill it in"
#include "secrets.example.h"
#endif

// ==========================================
// 1. SETTINGS
// ==========================================
const int I2C_SDA = 21, I2C_SCL = 22;
const int SENSOR_HZ = 500;            // MPU6050 output data rate
const int NETWORK_HZ = 60;            // UDP packet rate
const uint8_t DLPF_CFG = 2;           // 98 Hz gyro bandwidth: anti-aliased for 500 Hz, lower noise
const float CAL_SECONDS = 1.0f;       // boot calibration length
const float CAL_MAX_GYRO_STD = 0.5f;  // deg/s; recalibrate if the sensor moved during boot

// ==========================================
// 2. MPU6050 DRIVER (register level, FIFO based)
// ==========================================
namespace mpu {
const uint8_t ADDR = 0x68;
const uint8_t SMPLRT_DIV = 0x19, CONFIG = 0x1A, GYRO_CONFIG = 0x1B, ACCEL_CONFIG = 0x1C, FIFO_EN = 0x23,
              TEMP_OUT_H = 0x41, SIGNAL_PATH_RESET = 0x68, USER_CTRL = 0x6A, PWR_MGMT_1 = 0x6B,
              FIFO_COUNTH = 0x72, FIFO_R_W = 0x74, WHO_AM_I = 0x75;
const float GYRO_LSB_PER_DPS = 16.4f;  // +-2000 deg/s
const float ACCEL_LSB_PER_G = 4096.0f; // +-8 g
const int SAMPLE_BYTES = 12;           // accel xyz + gyro xyz, big-endian int16
const int FIFO_BYTES = 1024;
const int MAX_BURST = 10;              // samples per I2C read (120 B < 128 B Wire buffer)

bool write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool read(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void resetFifo() {
  write(FIFO_EN, 0x00);
  write(USER_CTRL, 0x04);  // FIFO_RESET
  write(USER_CTRL, 0x40);  // FIFO_EN
  write(FIFO_EN, 0x78);    // XG, YG, ZG, ACCEL
}

bool begin() {
  if (!write(PWR_MGMT_1, 0x80)) return false;  // device reset
  delay(100);
  write(PWR_MGMT_1, 0x01);  // wake, clock = PLL on X gyro (more stable than internal RC)
  delay(10);
  write(SIGNAL_PATH_RESET, 0x07);
  delay(10);
  uint8_t who = 0;
  if (!read(WHO_AM_I, &who, 1)) return false;
  Serial.printf("[MPU] WHO_AM_I = 0x%02X%s\n", who, who == 0x68 ? "" : " (not 0x68: clone chip?)");
  write(CONFIG, DLPF_CFG);                              // gyro rate 1 kHz with DLPF enabled
  write(SMPLRT_DIV, (uint8_t)(1000 / SENSOR_HZ - 1));   // 1 kHz / (1 + div)
  write(GYRO_CONFIG, 0x18);                             // +-2000 deg/s
  write(ACCEL_CONFIG, 0x10);                            // +-8 g
  resetFifo();
  return true;
}

bool readTemperature(float& c) {
  uint8_t b[2];
  if (!read(TEMP_OUT_H, b, 2)) return false;
  c = (int16_t)((b[0] << 8) | b[1]) / 340.0f + 36.53f;
  return true;
}
}  // namespace mpu

// ==========================================
// 3. STATE
// ==========================================
imu::Config makeFusionConfig() {
  imu::Config c;
  for (int i = 0; i < 9; i++) c.gyro_M[i] = GYRO_CAL_M[i];
  for (int i = 0; i < 3; i++) c.temp_slope_dps_per_c[i] = GYRO_TEMP_SLOPE[i];
  return c;
}

imu::Fusion fusion(makeFusionConfig());
WiFiUDP udp;

struct Snapshot {
  bool valid;
  uint32_t ms;
  imu::Quat q;
  bool still;
  float temp;
  imu::Vec3 bias, integ;
  uint32_t overflows;
};
Snapshot snap = {};
portMUX_TYPE snapMux = portMUX_INITIALIZER_UNLOCKED;

float dt = 1.0f / SENSOR_HZ;
int64_t odrT0 = 0;
uint64_t odrSamples = 0;
uint32_t overflows = 0, i2cErrors = 0;
uint32_t startMs = 0;
float temperature = 25.0f;

// Boot-calibration accumulator.
struct CalAccum {
  double gs[3], gss[3], as[3];
  int n, need;
} cal;

void restartOdrClock() {
  odrT0 = esp_timer_get_time();
  odrSamples = 0;
}

typedef void (*SampleSink)(imu::Vec3 gyro_dps, imu::Vec3 accel_g);

void fuseSample(imu::Vec3 g, imu::Vec3 a) { fusion.update(g, a, dt); }

void discardSample(imu::Vec3, imu::Vec3) {}

void calSample(imu::Vec3 g, imu::Vec3 a) {
  if (cal.n >= cal.need) return;
  const float gv[3] = {g.x, g.y, g.z}, av[3] = {a.x, a.y, a.z};
  for (int i = 0; i < 3; i++) {
    cal.gs[i] += gv[i];
    cal.gss[i] += gv[i] * gv[i];
    cal.as[i] += av[i];
  }
  cal.n++;
}

// Drain every complete sample from the FIFO into `sink`. Returns samples read.
int drainFifo(SampleSink sink) {
  uint8_t cnt[2];
  if (!mpu::read(mpu::FIFO_COUNTH, cnt, 2)) {
    i2cErrors++;
    return 0;
  }
  int count = (cnt[0] << 8) | cnt[1];
  if (count >= mpu::FIFO_BYTES - mpu::SAMPLE_BYTES) {
    // Overflowed: the oldest bytes were overwritten and alignment is lost.
    mpu::resetFifo();
    overflows++;
    restartOdrClock();
    return 0;
  }
  int n = count / mpu::SAMPLE_BYTES, done = 0;
  uint8_t buf[mpu::MAX_BURST * mpu::SAMPLE_BYTES];
  while (done < n) {
    int k = min(n - done, mpu::MAX_BURST);
    if (!mpu::read(mpu::FIFO_R_W, buf, k * mpu::SAMPLE_BYTES)) {
      i2cErrors++;
      mpu::resetFifo();  // re-align after a failed burst
      restartOdrClock();
      return done;
    }
    for (int s = 0; s < k; s++) {
      const uint8_t* p = buf + s * mpu::SAMPLE_BYTES;
      int16_t raw[6];
      for (int j = 0; j < 6; j++) raw[j] = (int16_t)((p[2 * j] << 8) | p[2 * j + 1]);
      imu::Vec3 a = {raw[0] / mpu::ACCEL_LSB_PER_G, raw[1] / mpu::ACCEL_LSB_PER_G, raw[2] / mpu::ACCEL_LSB_PER_G};
      imu::Vec3 g = {raw[3] / mpu::GYRO_LSB_PER_DPS, raw[4] / mpu::GYRO_LSB_PER_DPS, raw[5] / mpu::GYRO_LSB_PER_DPS};
      sink(g, a);
    }
    done += k;
  }
  odrSamples += done;
  return done;
}

// The MPU's sample clock is only accurate to a few percent; any error there is
// a direct heading scale error. Measure it against the ESP32 crystal instead.
void updateDt() {
  int64_t elapsed = esp_timer_get_time() - odrT0;
  if (elapsed < 5000000 || odrSamples < 1000) return;
  float odr = odrSamples * 1e6f / (float)elapsed;
  if (fabsf(odr - SENSOR_HZ) < 0.05f * SENSOR_HZ) dt = 1.0f / odr;
}

// ==========================================
// 4. BOOT CALIBRATION
// ==========================================
void calibrate() {
  for (;;) {
    Serial.println("[SYSTEM] Calibrating gyro... DO NOT MOVE.");
    cal = {};
    cal.need = (int)(CAL_SECONDS * SENSOR_HZ);
    mpu::resetFifo();
    delay(50);
    drainFifo(discardSample);  // settling samples
    while (cal.n < cal.need) {
      drainFifo(calSample);
      delay(2);
    }
    const int n = cal.n;
    const double *gs = cal.gs, *gss = cal.gss, *as = cal.as;
    double var = 0;
    for (int i = 0; i < 3; i++) var += gss[i] / n - (gs[i] / n) * (gs[i] / n);
    float stdev = sqrtf((float)fmax(var, 0.0));
    if (stdev > CAL_MAX_GYRO_STD) {
      Serial.printf("[SYSTEM] Sensor moved during calibration (%.2f deg/s rms). Retrying...\n", stdev);
      delay(500);
      continue;
    }
    mpu::readTemperature(temperature);
    fusion.setBias({(float)(gs[0] / n), (float)(gs[1] / n), (float)(gs[2] / n)}, temperature);
    fusion.initFromAccel({(float)(as[0] / n), (float)(as[1] / n), (float)(as[2] / n)});
    Serial.printf("[SYSTEM] Calibration complete. Bias %.3f %.3f %.3f deg/s at %.1f C (noise %.3f)\n",
                  gs[0] / n, gs[1] / n, gs[2] / n, temperature, stdev);
    return;
  }
}

// ==========================================
// 5. NETWORK TASK (core 0)
// ==========================================
void netTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / NETWORK_HZ);
  TickType_t last = xTaskGetTickCount();
  char payload[256];
  for (;;) {
    vTaskDelayUntil(&last, period);
    Snapshot s;
    portENTER_CRITICAL(&snapMux);
    s = snap;
    portEXIT_CRITICAL(&snapMux);
    if (!s.valid || WiFi.status() != WL_CONNECTED) continue;
    int len = snprintf(payload, sizeof(payload),
                       "%lu,%.5f,%.5f,%.5f,%.5f,%d,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%lu",
                       (unsigned long)s.ms, s.q.w, s.q.x, s.q.y, s.q.z, s.still ? 1 : 0, s.temp,
                       s.bias.x, s.bias.y, s.bias.z, s.integ.x, s.integ.y, s.integ.z,
                       (unsigned long)s.overflows);
    if (len <= 0) continue;
    udp.beginPacket(RECEIVER_IP, UDP_PORT);
    udp.write((const uint8_t*)payload, min(len, (int)sizeof(payload) - 1));
    udp.endPacket();
  }
}

// ==========================================
// 6. SETUP / LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // modem sleep adds 100+ ms latency spikes
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[NET] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[NET] Connected, IP %s -> %s:%d\n", WiFi.localIP().toString().c_str(), RECEIVER_IP, UDP_PORT);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  while (!mpu::begin()) {
    Serial.println("[MPU] Not responding, retrying...");
    delay(500);
  }

  Serial.println("\n[SYSTEM] IMU booting. Hands off the sensor!");
  for (int i = 3; i > 0; i--) {
    Serial.printf("Calibrating in %d...\n", i);
    delay(1000);
  }
  calibrate();

  mpu::resetFifo();
  restartOdrClock();
  startMs = millis();
  xTaskCreatePinnedToCore(netTask, "net", 4096, nullptr, 1, nullptr, 0);
}

void loop() {
  static uint32_t lastTempMs = 0, lastLogMs = 0;
  uint32_t now = millis();

  if (now - lastTempMs >= 100) {
    lastTempMs = now;
    float c;
    if (mpu::readTemperature(c)) {
      temperature += 0.2f * (c - temperature);  // smooth the 1/340 C quantisation
      fusion.setTemperature(temperature);
    }
  }

  int n = drainFifo(fuseSample);
  updateDt();

  if (n > 0) {
    Snapshot s;
    s.valid = true;
    s.ms = millis() - startMs;
    s.q = fusion.q();
    s.still = fusion.still();
    s.temp = fusion.temperature();
    s.bias = fusion.bias();
    s.integ = fusion.integratedDeg();
    s.overflows = overflows;
    portENTER_CRITICAL(&snapMux);
    snap = s;
    portEXIT_CRITICAL(&snapMux);
  }

  if (now - lastLogMs >= 5000) {
    lastLogMs = now;
    imu::Vec3 b = fusion.bias(), k = fusion.tempSlope();
    Serial.printf("[STAT] odr %.2f Hz  T %.2f C  bias %.3f %.3f %.3f  slope %.4f %.4f %.4f  still %d  ovf %lu  i2c %lu\n",
                  1.0f / dt, fusion.temperature(), b.x, b.y, b.z, k.x, k.y, k.z, fusion.still() ? 1 : 0,
                  (unsigned long)overflows, (unsigned long)i2cErrors);
  }

  delay(1);  // FIFO buffers ~170 ms, so yielding here costs nothing
}
