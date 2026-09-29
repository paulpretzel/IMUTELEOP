// test_fusion.cpp -- invariants of imu_fusion.h.
// Build & run:  g++ -O2 -std=c++17 -o /tmp/test_fusion tests/test_fusion.cpp && /tmp/test_fusion
#include <cmath>
#include <cstdio>
#include <random>

#include "../firmware/YawDriftSocket/imu_fusion.h"

static int failures = 0;
#define CHECK(cond, ...)                          \
  do {                                            \
    if (!(cond)) {                                \
      failures++;                                 \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      std::printf(__VA_ARGS__);                   \
      std::printf("\n");                          \
    }                                             \
  } while (0)

static const float DT = 0.002f;

static float headingDeg(imu::Quat q) { return 2.0f * atan2f(q.z, q.w) * 57.2957795f; }

// Gravity predicted by the filter must match the accel used to level it.
static void testInitFromAccel() {
  const imu::Vec3 cases[] = {{0, 0, 1}, {0.5f, 0.2f, 0.84f}, {-0.7f, 0.6f, -0.3f}, {0.1f, -0.99f, 0.05f}};
  for (auto a : cases) {
    imu::Fusion f;
    f.initFromAccel(a);
    imu::Quat q = f.q();
    float n = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    float vx = 2 * (q.x * q.z - q.w * q.y), vy = 2 * (q.w * q.x + q.y * q.z), vz = 1 - 2 * (q.x * q.x + q.y * q.y);
    float err = fabsf(vx - a.x / n) + fabsf(vy - a.y / n) + fabsf(vz - a.z / n);
    CHECK(err < 1e-4f, "initFromAccel gravity mismatch %g for (%g %g %g)", err, a.x, a.y, a.z);
  }
}

// A pure rotation about the vertical must integrate exactly (no filter loss).
static void testExactYawIntegration() {
  imu::Config c;
  c.learn_bias = false;
  imu::Fusion f(c);
  f.setBias({0, 0, 0}, 25);
  f.initFromAccel({0, 0, 1});
  for (int i = 0; i < 500; i++) f.update({0, 0, 90.0f}, {0, 0, 1}, DT);  // 90 deg in 1 s
  float h = headingDeg(f.q());
  CHECK(fabsf(h - 90.0f) < 0.05f, "expected 90 deg, got %g", h);
}

// Accelerometer correction must never change heading, even with large tilt
// errors and linear acceleration.
static void testAccelDoesNotTouchHeading() {
  imu::Fusion f;
  f.setBias({0, 0, 0}, 25);
  f.initFromAccel({0, 0, 1});
  std::mt19937 rng(3);
  std::normal_distribution<float> N(0, 0.3f);
  for (int i = 0; i < 5000; i++) f.update({0, 0, 0}, {N(rng), N(rng), 1.0f + N(rng)}, DT);
  // With zero rotation the heading of the leveled frame stays 0; measure the
  // world-z twist of the whole attitude change.
  float h = headingDeg(f.q());
  CHECK(fabsf(h) < 0.5f, "accel correction leaked into heading: %g deg", h);
}

// A static sensor with a bias error must learn it and stop drifting.
static void testBiasLearningAndHold() {
  imu::Fusion f;
  f.setBias({0, 0, 0}, 25);  // boot calibration was wrong by 0.4 deg/s on z
  f.initFromAccel({0, 0, 1});
  std::mt19937 rng(5);
  std::normal_distribution<float> N(0, 0.08f);
  for (int i = 0; i < 5000; i++) f.update({N(rng), N(rng), 0.4f + N(rng)}, {0, 0, 1}, DT);  // 10 s
  CHECK(f.still(), "sensor at rest not detected as still");
  CHECK(fabsf(f.bias().z - 0.4f) < 0.02f, "bias not learned: %g", f.bias().z);
  // Only the first ~0.5 s (hold time) may integrate the bias error.
  float h = headingDeg(f.q());
  CHECK(fabsf(h) < 0.3f, "drift while resting: %g deg", h);
}

// Motion must end the rest state on the very first moving sample.
static void testStillExitsImmediately() {
  imu::Fusion f;
  f.setBias({0, 0, 0}, 25);
  f.initFromAccel({0, 0, 1});
  for (int i = 0; i < 1000; i++) f.update({0, 0, 0}, {0, 0, 1}, DT);
  CHECK(f.still(), "expected still");
  f.update({0, 0, 5.0f}, {0, 0, 1}, DT);
  CHECK(!f.still(), "still flag survived a 5 deg/s sample");
}

// A slow rotation that passes as rest may only shift the bias by a bounded amount.
static void testBiasStepBounded() {
  imu::Config c;
  c.bias_max_step_dps = 0.3f;
  imu::Fusion f(c);
  f.setBias({0, 0, 0}, 25);
  f.initFromAccel({0, 0, 1});
  for (int i = 0; i < 15000; i++) f.update({0, 0, 0.9f}, {0, 0, 1}, DT);  // 30 s at 0.9 deg/s
  CHECK(fabsf(f.bias().z) <= 0.3f + 1e-4f, "bias step unbounded: %g", f.bias().z);
}

// Bias learned at two temperatures yields the right slope, which is then used
// to predict the bias while moving.
static void testTemperatureSlope() {
  imu::Fusion f;
  f.setBias({0, 0, 0.0f}, 25);
  f.initFromAccel({0, 0, 1});
  const float slope = 0.05f;
  for (int rest = 0; rest < 4; rest++) {
    float T = 25.0f + 2.0f * (rest + 1);
    f.setTemperature(T);
    for (int i = 0; i < 2500; i++) f.update({0, 0, slope * (T - 25.0f)}, {0, 0, 1}, DT);  // 5 s rest
    f.update({0, 0, 30}, {0, 0, 1}, DT);  // move: ends the rest, records the point
  }
  CHECK(fabsf(f.tempSlope().z - slope) < 0.005f, "slope %g, expected %g", f.tempSlope().z, slope);
  f.setTemperature(37.0f);
  f.update({0, 0, 30}, {0, 0, 1}, DT);
  CHECK(fabsf(f.bias().z - slope * 12.0f) < 0.03f, "predicted bias %g, expected %g", f.bias().z, slope * 12.0f);
}

int main() {
  testInitFromAccel();
  testExactYawIntegration();
  testAccelDoesNotTouchHeading();
  testBiasLearningAndHold();
  testStillExitsImmediately();
  testBiasStepBounded();
  testTemperatureSlope();
  if (failures) {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("all fusion tests passed\n");
  return 0;
}
