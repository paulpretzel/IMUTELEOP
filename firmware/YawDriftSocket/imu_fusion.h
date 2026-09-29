// imu_fusion.h -- portable 6-axis (gyro + accel) orientation core.
//
// No Arduino dependencies, so the exact same code runs on the ESP32 and in the
// desktop simulator (tests/sim_yaw_drift.cpp).
//
// Why yaw drifts on a 6-axis IMU
// ------------------------------
// Gravity only tells the filter which way is "down". The accelerometer
// correction term e = a x v is perpendicular to the estimated gravity vector,
// so it has *zero* component about the vertical axis: neither Mahony nor
// Madgwick can ever correct heading. Heading error is simply the integral of
// every gyro error:
//
//     yaw_error(t) = integral( bias(T, t) + scale/cross-axis error * omega + noise )
//
// so this core attacks each term directly instead of trying to "filter" yaw:
//   * bias      -> boot calibration + online zero-rate learning whenever the
//                  sensor is stationary (ZARU), plus a learned temperature
//                  slope so the bias keeps tracking warm-up between rests.
//   * scale     -> 3x3 scale / cross-axis correction matrix (see
//                  tools/gyro_scale_cal.py).
//   * rest      -> orientation hold while stationary (ZUPT): nothing is
//                  integrated while the sensor is provably not rotating.
//   * tilt      -> Mahony proportional term (Ki = 0), faded out when |a| != 1 g
//                  so arm-swing accelerations do not corrupt the gravity
//                  reference.
//   * numerics  -> exact quaternion exponential integration.
//
// Conventions: q = (w, x, y, z) rotates sensor-frame vectors into the world
// frame (world z = up). Gyro in deg/s, accel in g, temperature in deg C.
#pragma once

#include <math.h>

namespace imu {

struct Vec3 {
  float x, y, z;
};

struct Quat {
  float w, x, y, z;
};

struct Config {
  // ---- Tilt correction (Mahony proportional term, Ki = 0) ----
  float kp = 0.5f;            // rad/s per rad of tilt error (old firmware: twoKp = 1.0)
  float kp_still = 2.0f;      // faster tilt convergence while stationary
  float accel_gate_g = 0.10f; // correction fades to 0 as | |a| - 1 g | approaches this

  // ---- Stationary detection ----
  // "Still" = every sample for still_hold_s had |gyro - bias| < still_rate_dps
  // and |a| within still_accel_tol_g of 1 g. Per-sample tests (instead of a
  // running variance) react within one sample when motion starts and within
  // still_hold_s after it stops, so short rests are not wasted.
  float still_rate_dps = 1.0f;     // ~7 sigma of MPU6050 rate noise at 98 Hz DLPF
  float still_accel_tol_g = 0.05f;
  float still_hold_s = 0.5f;

  // ---- Online bias learning (ZARU) and orientation hold (ZUPT) ----
  bool learn_bias = true;
  bool zupt = true;
  float bias_tau_s = 1.0f; // bias learning time constant while stationary
  float bias_converged_tau = 2.0f; // rest longer than this many tau -> trusted bias sample
  // A single rest may move the bias at most this far. Guards against a very
  // slow, smooth deliberate rotation (< still_rate_dps) being learned as bias.
  float bias_max_step_dps = 0.5f;

  // ---- Temperature compensation of the gyro bias ----
  bool temp_comp = true;
  float temp_slope_dps_per_c[3] = {0.0f, 0.0f, 0.0f}; // prior, refined online
  float temp_min_std_c = 0.5f;  // temperature spread needed before trusting a fitted slope
  float temp_slope_max = 0.2f;  // clamp (datasheet worst case is ~0.16 dps/C)
  float temp_forget = 0.9f;     // forgetting factor per regression point

  // ---- Gyro scale / cross-axis correction: omega = M * (raw - bias), row-major ----
  float gyro_M[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
};

class Fusion {
 public:
  Config cfg;

  explicit Fusion(const Config& c = Config()) : cfg(c) { reset(); }

  void reset() {
    q_ = {1.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; i++) {
      b_[i] = bAnchor_[i] = 0.0f;
      bRest_[i] = 0.0f;
      integ_[i] = 0.0f;
      slope_[i] = cfg.temp_slope_dps_per_c[i];
      rS_b_[i] = rS_Tb_[i] = 0.0f;
    }
    temp_ = tAnchor_ = tRef_ = 25.0f;
    haveAnchor_ = havePoint_ = pending_ = false;
    rS0_ = rS_T_ = rS_TT_ = 0.0f;
    still_ = false;
    stillTime_ = learnTime_ = 0.0f;
  }

  // Level the filter from a (stationary) accelerometer reading; heading = 0.
  void initFromAccel(Vec3 a) {
    float n = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    if (n < 1e-6f) return;
    float roll = atan2f(a.y, a.z);
    float pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    q_ = {cr * cp, sr * cp, cr * sp, -sr * sp};
  }

  // Seed the bias from a boot calibration taken at temperature temp_c.
  void setBias(Vec3 b, float temp_c) {
    temp_ = temp_c;
    b_[0] = b.x; b_[1] = b.y; b_[2] = b.z;
    setAnchor();
    addRegressionPoint(temp_c, b_);
  }

  void setTemperature(float c) { temp_ = c; }

  // One sensor sample. gyro in deg/s (raw, bias NOT removed), accel in g.
  void update(Vec3 gyro, Vec3 accel, float dt) {
    if (dt <= 0.0f) return;
    const float g[3] = {gyro.x, gyro.y, gyro.z};
    const float an = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

    // Between rests, predict the bias from temperature.
    if (!still_ && cfg.temp_comp && haveAnchor_) {
      for (int i = 0; i < 3; i++) b_[i] = bAnchor_[i] + slope_[i] * (temp_ - tAnchor_);
    }

    updateStillness(g, an, dt);

    // Learn the zero-rate output while provably stationary.
    if (still_ && cfg.learn_bias) {
      float k = dt / cfg.bias_tau_s;
      if (k > 1.0f) k = 1.0f;
      for (int i = 0; i < 3; i++) {
        b_[i] += k * (g[i] - b_[i]);
        float lo = bRest_[i] - cfg.bias_max_step_dps, hi = bRest_[i] + cfg.bias_max_step_dps;
        b_[i] = b_[i] < lo ? lo : (b_[i] > hi ? hi : b_[i]);
      }
      learnTime_ += dt;
      if (learnTime_ >= cfg.bias_converged_tau * cfg.bias_tau_s) {
        setAnchor();
        pending_ = true;
      }
    }

    // Bias-corrected rates: uncalibrated (for scale calibration) and calibrated.
    float r[3];
    for (int i = 0; i < 3; i++) {
      r[i] = g[i] - b_[i];
      integ_[i] += r[i] * dt;
    }
    const float* M = cfg.gyro_M;
    const float D2R = 0.017453292519943295f;
    float wx = (M[0] * r[0] + M[1] * r[1] + M[2] * r[2]) * D2R;
    float wy = (M[3] * r[0] + M[4] * r[1] + M[5] * r[2]) * D2R;
    float wz = (M[6] * r[0] + M[7] * r[1] + M[8] * r[2]) * D2R;
    if (still_ && cfg.zupt) wx = wy = wz = 0.0f;

    // Tilt correction from gravity. e is perpendicular to the estimated
    // vertical, so this never touches heading.
    if (an > 1e-6f) {
      float gate = 1.0f - fabsf(an - 1.0f) / cfg.accel_gate_g;
      if (gate > 0.0f) {
        const Quat& q = q_;
        float vx = 2.0f * (q.x * q.z - q.w * q.y);
        float vy = 2.0f * (q.w * q.x + q.y * q.z);
        float vz = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
        float ax = accel.x / an, ay = accel.y / an, az = accel.z / an;
        float k = (still_ ? cfg.kp_still : cfg.kp) * gate;
        wx += k * (ay * vz - az * vy);
        wy += k * (az * vx - ax * vz);
        wz += k * (ax * vy - ay * vx);
      }
    }

    integrate(wx, wy, wz, dt);
  }

  Quat q() const { return q_; }
  bool still() const { return still_; }
  Vec3 bias() const { return {b_[0], b_[1], b_[2]}; }
  Vec3 tempSlope() const { return {slope_[0], slope_[1], slope_[2]}; }
  float temperature() const { return temp_; }
  // Integral of bias-corrected, *uncalibrated* rates per sensor axis [deg].
  Vec3 integratedDeg() const { return {integ_[0], integ_[1], integ_[2]}; }
  void resetIntegrated() { integ_[0] = integ_[1] = integ_[2] = 0.0f; }

 private:
  Quat q_;
  float b_[3], bAnchor_[3], slope_[3];
  float bRest_[3];
  float integ_[3];
  float temp_, tAnchor_, tRef_;
  bool haveAnchor_, havePoint_, pending_;
  float rS0_, rS_T_, rS_TT_, rS_b_[3], rS_Tb_[3];
  bool still_;
  float stillTime_, learnTime_;

  void setAnchor() {
    for (int i = 0; i < 3; i++) bAnchor_[i] = b_[i];
    tAnchor_ = temp_;
    haveAnchor_ = true;
  }

  void updateStillness(const float g[3], float an, float dt) {
    float r2 = 0.0f;
    for (int i = 0; i < 3; i++) {
      float r = g[i] - b_[i];
      r2 += r * r;
    }
    bool cond = r2 < cfg.still_rate_dps * cfg.still_rate_dps &&
                fabsf(an - 1.0f) < cfg.still_accel_tol_g;
    if (cond) {
      stillTime_ += dt;
    } else {
      if (still_) endStill();
      stillTime_ = 0.0f;
    }
    bool was = still_;
    still_ = stillTime_ >= cfg.still_hold_s;
    if (still_ && !was)
      for (int i = 0; i < 3; i++) bRest_[i] = b_[i];
  }

  void endStill() {
    // Continue the temperature prediction from what was just learned.
    if (learnTime_ > 0.0f) setAnchor();
    if (pending_) addRegressionPoint(tAnchor_, bAnchor_);
    pending_ = false;
    learnTime_ = 0.0f;
  }

  // Exponentially-forgetting least squares fit of bias vs temperature.
  void addRegressionPoint(float T, const float b[3]) {
    if (!havePoint_) {
      tRef_ = T;
      havePoint_ = true;
    }
    float t = T - tRef_;
    float lam = cfg.temp_forget;
    rS0_ = lam * rS0_ + 1.0f;
    rS_T_ = lam * rS_T_ + t;
    rS_TT_ = lam * rS_TT_ + t * t;
    for (int i = 0; i < 3; i++) {
      rS_b_[i] = lam * rS_b_[i] + b[i];
      rS_Tb_[i] = lam * rS_Tb_[i] + t * b[i];
    }
    if (rS0_ < 1.5f) return;
    float mt = rS_T_ / rS0_;
    float varT = rS_TT_ / rS0_ - mt * mt;
    if (varT < cfg.temp_min_std_c * cfg.temp_min_std_c) return;
    for (int i = 0; i < 3; i++) {
      float cov = rS_Tb_[i] / rS0_ - mt * (rS_b_[i] / rS0_);
      float s = cov / varT;
      if (s > cfg.temp_slope_max) s = cfg.temp_slope_max;
      if (s < -cfg.temp_slope_max) s = -cfg.temp_slope_max;
      slope_[i] = s;
    }
  }

  void integrate(float wx, float wy, float wz, float dt) {
    float wn = sqrtf(wx * wx + wy * wy + wz * wz);
    float th = wn * dt;
    float c, s;
    if (th > 1e-6f) {
      c = cosf(0.5f * th);
      s = sinf(0.5f * th) / wn;
    } else {
      c = 1.0f - th * th / 8.0f;
      s = 0.5f * dt;
    }
    float dw = c, dx = wx * s, dy = wy * s, dz = wz * s;
    Quat a = q_;
    q_.w = a.w * dw - a.x * dx - a.y * dy - a.z * dz;
    q_.x = a.w * dx + a.x * dw + a.y * dz - a.z * dy;
    q_.y = a.w * dy - a.x * dz + a.y * dw + a.z * dx;
    q_.z = a.w * dz + a.x * dy - a.y * dx + a.z * dw;
    float n = sqrtf(q_.w * q_.w + q_.x * q_.x + q_.y * q_.y + q_.z * q_.z);
    if (n > 1e-9f) {
      q_.w /= n; q_.x /= n; q_.y /= n; q_.z /= n;
    }
  }
};

}  // namespace imu
