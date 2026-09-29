// sim_yaw_drift.cpp -- desktop simulation of heading drift for the 6-axis node.
//
// Simulates an MPU6050 strapped to a moving arm (random high-rate swings with
// rests in between) including the error sources that actually cause 6-axis
// yaw drift, then runs both the original firmware filter and the new
// imu_fusion.h core on identical data and reports the heading error.
//
// Build & run:  g++ -O2 -std=c++17 -o /tmp/sim tests/sim_yaw_drift.cpp && /tmp/sim
//
// Error model (per sensor, randomised per seed):
//   gyro bias     U(-2, 2) deg/s at boot + temperature slope N(0, 0.02) deg/s/C
//                 + bias random walk 0.002 deg/s/sqrt(s)
//   warm-up       die temperature 25 C -> 35 C, time constant 200 s
//   scale         1 + N(0, 1.5 %) per axis, cross-axis N(0, 1 %)
//   noise         0.005 deg/s/sqrt(Hz), accel 400 ug/sqrt(Hz)
//   quantisation  +-2000 deg/s and +-8 g full-scale, 16 bit
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "../firmware/YawDriftSocket/imu_fusion.h"

using imu::Quat;
using imu::Vec3;

static const double PI = 3.14159265358979323846;
static const double D2R = PI / 180.0, R2D = 180.0 / PI;

// ----------------------------------------------------------------- math
struct Q {
  double w, x, y, z;
};
static Q qmul(Q a, Q b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
static Q qconj(Q a) { return {a.w, -a.x, -a.y, -a.z}; }
static Q qnorm(Q a) {
  double n = std::sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
  return {a.w / n, a.x / n, a.y / n, a.z / n};
}
static Q qexp(double wx, double wy, double wz, double dt) {
  double n = std::sqrt(wx * wx + wy * wy + wz * wz), th = n * dt;
  if (th < 1e-12) return {1, 0.5 * wx * dt, 0.5 * wy * dt, 0.5 * wz * dt};
  double s = std::sin(th / 2) / n;
  return {std::cos(th / 2), wx * s, wy * s, wz * s};
}
static void rotT(Q q, const double v[3], double out[3]) {  // R(q)^T v
  Q p = qmul(qmul(qconj(q), {0, v[0], v[1], v[2]}), q);
  out[0] = p.x; out[1] = p.y; out[2] = p.z;
}
static void rotvec(Q q, double out[3]) {  // log map
  if (q.w < 0) q = {-q.w, -q.x, -q.y, -q.z};
  double vn = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  if (vn < 1e-12) { out[0] = out[1] = out[2] = 0; return; }
  double k = 2.0 * std::atan2(vn, q.w) / vn;
  out[0] = q.x * k; out[1] = q.y * k; out[2] = q.z * k;
}
static double wrap180(double a) { return std::fmod(std::fmod(a + 180.0, 360.0) + 360.0, 360.0) - 180.0; }
// Heading error: world-z twist of the rotation that takes the estimate onto the truth.
static double headingErrDeg(Q truth, Q est) {
  Q d = qmul(truth, qconj(est));
  return wrap180(2.0 * std::atan2(d.z, d.w) * R2D);
}

// ------------------------------------------------ original firmware filter
// Verbatim port of updateMahony() from the original YawDriftSocket.ino
// (twoKp = 1.0, twoKi = 0, first-order integration, boot calibration only).
struct Baseline {
  float mq0 = 1, mq1 = 0, mq2 = 0, mq3 = 0;
  float twoKp = 1.0f;
  float bias[3] = {0, 0, 0};
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    gx -= bias[0]; gy -= bias[1]; gz -= bias[2];
    float recipNorm, halfvx, halfvy, halfvz, halfex, halfey, halfez, qa, qb, qc;
    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      recipNorm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
      halfvx = mq1 * mq3 - mq0 * mq2;
      halfvy = mq0 * mq1 + mq2 * mq3;
      halfvz = mq0 * mq0 - 0.5f + mq3 * mq3;
      halfex = (ay * halfvz - az * halfvy);
      halfey = (az * halfvx - ax * halfvz);
      halfez = (ax * halfvy - ay * halfvx);
      gx += twoKp * halfex; gy += twoKp * halfey; gz += twoKp * halfez;
    }
    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    qa = mq0; qb = mq1; qc = mq2;
    mq0 += (-qb * gx - qc * gy - mq3 * gz);
    mq1 += (qa * gx + qc * gz - mq3 * gy);
    mq2 += (qa * gy - qb * gz + mq3 * gx);
    mq3 += (qa * gz + qb * gy - qc * gx);
    recipNorm = 1.0f / std::sqrt(mq0 * mq0 + mq1 * mq1 + mq2 * mq2 + mq3 * mq3);
    mq0 *= recipNorm; mq1 *= recipNorm; mq2 *= recipNorm; mq3 *= recipNorm;
  }
  Q q() const { return {mq0, mq1, mq2, mq3}; }
};

// ------------------------------------------------------------- scenario
struct Scenario {
  const char* name;
  double duration_s;
  double move_min_s, move_max_s;  // length of a motion bout
  double rest_min_s, rest_max_s;  // length of a rest between bouts
  double temp_rise_c;             // warm-up amplitude
};

struct Sample {
  double t, gyro[3], acc[3], temp;
  Q truth;
};

static std::vector<Sample> generate(const Scenario& sc, unsigned seed, double S[9]) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> N(0.0, 1.0);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  const double fs = 500.0, dt = 1.0 / fs;

  double b0[3], kT[3];
  for (int i = 0; i < 3; i++) {
    b0[i] = -2.0 + 4.0 * U(rng);
    kT[i] = 0.02 * N(rng);
  }
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) S[r * 3 + c] = (r == c) ? 1.0 + 0.015 * N(rng) : 0.01 * N(rng);

  // Phases: boot rest, then alternating move / rest.
  std::vector<std::pair<double, bool>> phases;  // (end time, moving)
  double t = 3.0;
  phases.push_back({t, false});
  while (t < sc.duration_s) {
    t += sc.move_min_s + (sc.move_max_s - sc.move_min_s) * U(rng);
    phases.push_back({t, true});
    t += sc.rest_min_s + (sc.rest_max_s - sc.rest_min_s) * U(rng);
    phases.push_back({t, false});
  }

  // Random initial tilt, heading 0.
  // (pitch applied after roll, i.e. Z-Y-X Euler with yaw = 0, like initFromAccel)
  Q truth = qnorm(qmul(qexp(0, 0.4 * N(rng), 0, 1.0), qexp(0.4 * N(rng), 0, 0, 1.0)));
  Q q0 = truth;
  double ou[3] = {0, 0, 0}, la[3] = {0, 0, 0}, brw[3] = {0, 0, 0};
  const double tau_w = 0.3, sig_w = 150.0 * D2R, tau_a = 0.15, sig_a = 0.4 * 9.81, k_home = 1.5;
  const double gyro_nd = 0.005 * std::sqrt(fs / 2), acc_nd = 400e-6 * std::sqrt(fs / 2);

  std::vector<Sample> out;
  out.reserve((size_t)(sc.duration_s * fs) + 10);
  size_t ph = 0;
  double phase_start = 0.0;
  for (double tt = 0.0; tt < sc.duration_s; tt += dt) {
    while (ph + 1 < phases.size() && tt >= phases[ph].first) {
      phase_start = phases[ph].first;
      ph++;
    }
    bool moving = phases[ph].second;
    double env = 0.0;
    if (moving) {
      double a = (tt - phase_start) / 0.5, b = (phases[ph].first - tt) / 0.5;
      env = std::min(1.0, std::min(a, b));
      env = env * env * (3 - 2 * env);  // smoothstep ramps
    }
    // Arm-like angular velocity: OU noise + pull back toward the start pose.
    double rv[3];
    rotvec(qmul(qconj(q0), truth), rv);
    double w[3], f[3];
    for (int i = 0; i < 3; i++) {
      ou[i] += -ou[i] / tau_w * dt + sig_w * std::sqrt(2.0 * dt / tau_w) * N(rng);
      la[i] += -la[i] / tau_a * dt + sig_a * std::sqrt(2.0 * dt / tau_a) * N(rng);
      w[i] = env * (ou[i] - k_home * rv[i]);
      f[i] = env * la[i];
    }
    f[2] += 9.81;  // specific force at rest = +1 g up

    double T = 25.0 + sc.temp_rise_c * (1.0 - std::exp(-tt / 200.0));
    Sample s;
    s.t = tt;
    s.temp = T;
    s.truth = truth;
    double fb[3];
    rotT(truth, f, fb);
    for (int r = 0; r < 3; r++) {
      brw[r] += 0.002 * std::sqrt(dt) * N(rng);
      double m = 0.0;
      for (int c = 0; c < 3; c++) m += S[r * 3 + c] * w[c] * R2D;
      double g = m + b0[r] + kT[r] * (T - 25.0) + brw[r] + gyro_nd * N(rng);
      g = std::max(-2000.0, std::min(2000.0, std::round(g * 16.4) / 16.4));
      double a = fb[r] / 9.81 + acc_nd * N(rng);
      a = std::max(-8.0, std::min(8.0, std::round(a * 4096.0) / 4096.0));
      s.gyro[r] = g;
      s.acc[r] = a;
    }
    out.push_back(s);
    truth = qnorm(qmul(truth, qexp(w[0], w[1], w[2], dt)));
  }
  return out;
}

// ------------------------------------------------------------- runners
struct Result {
  double final_err, max_err, rms_err;
};

static void bootCal(const std::vector<Sample>& d, double b[3], double a[3], double& T) {
  // Mirror the firmware: average ~1 s of samples while the sensor rests.
  int n = 0;
  b[0] = b[1] = b[2] = a[0] = a[1] = a[2] = 0;
  for (const auto& s : d) {
    if (s.t < 1.0) continue;
    if (s.t >= 2.0) break;
    for (int i = 0; i < 3; i++) { b[i] += s.gyro[i]; a[i] += s.acc[i]; }
    T = s.temp;
    n++;
  }
  for (int i = 0; i < 3; i++) { b[i] /= n; a[i] /= n; }
}

static Result score(const std::vector<Sample>& d, const std::function<Q(const Sample&)>& step) {
  Result r{0, 0, 0};
  double se = 0;
  int n = 0;
  Q est{1, 0, 0, 0};
  for (const auto& s : d) {
    // s.truth is the pose at the *start* of this sample, which is what the
    // estimate describes after consuming all previous samples.
    Q prev = est;
    est = step(s);
    if (s.t < 2.0) continue;
    double e = std::fabs(headingErrDeg(s.truth, prev));
    r.max_err = std::max(r.max_err, e);
    se += e * e;
    n++;
    r.final_err = e;
  }
  r.rms_err = std::sqrt(se / std::max(1, n));
  return r;
}

static Result runBaseline(const std::vector<Sample>& d) {
  Baseline f;
  double b[3], a[3], T = 25;
  bootCal(d, b, a, T);
  for (int i = 0; i < 3; i++) f.bias[i] = (float)b[i];
  const float dt = 1.0f / 500.0f;
  return score(d, [&](const Sample& s) {
    if (s.t >= 2.0)
      f.update(s.gyro[0], s.gyro[1], s.gyro[2], s.acc[0], s.acc[1], s.acc[2], dt);
    return f.q();
  });
}

static Result runCore(const std::vector<Sample>& d, imu::Config cfg, const double* Mcal) {
  if (Mcal)
    for (int i = 0; i < 9; i++) cfg.gyro_M[i] = (float)Mcal[i];
  imu::Fusion f(cfg);
  double b[3], a[3], T = 25;
  bootCal(d, b, a, T);
  f.setBias({(float)b[0], (float)b[1], (float)b[2]}, (float)T);
  f.initFromAccel({(float)a[0], (float)a[1], (float)a[2]});
  const float dt = 1.0f / 500.0f;
  return score(d, [&](const Sample& s) {
    if (s.t >= 2.0) {
      f.setTemperature((float)s.temp);
      f.update({(float)s.gyro[0], (float)s.gyro[1], (float)s.gyro[2]},
               {(float)s.acc[0], (float)s.acc[1], (float)s.acc[2]}, dt);
    }
    Quat q = f.q();
    return Q{q.w, q.x, q.y, q.z};
  });
}

static void inv3(const double m[9], double o[9]) {
  double a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
  double A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  double det = a * A + b * B + c * C;
  o[0] = A / det; o[1] = -(b * i - c * h) / det; o[2] = (b * f - c * e) / det;
  o[3] = B / det; o[4] = (a * i - c * g) / det; o[5] = -(a * f - c * d) / det;
  o[6] = C / det; o[7] = -(a * h - b * g) / det; o[8] = (a * e - b * d) / det;
}

// --------------------------------------------------------------- main
int main(int argc, char** argv) {
  int seeds = argc > 1 ? std::atoi(argv[1]) : 20;

  const Scenario scenarios[] = {
      {"frequent rests (8-20 s moving, 2-6 s resting), 10 min", 600, 8, 20, 2, 6, 10},
      {"sparse rests (40-80 s moving, 2-4 s resting), 10 min", 600, 40, 80, 2, 4, 10},
  };

  struct Variant {
    std::string name;
    std::function<Result(const std::vector<Sample>&, const double* S)> run;
  };
  auto cfgWith = [](bool learn, bool zupt, bool temp) {
    imu::Config c;
    c.learn_bias = learn;
    c.zupt = zupt;
    c.temp_comp = temp;
    return c;
  };
  std::vector<Variant> variants = {
      {"original firmware (boot cal only)", [](const std::vector<Sample>& d, const double*) { return runBaseline(d); }},
      {"new core: exact integ + accel gating",
       [&](const std::vector<Sample>& d, const double*) { return runCore(d, cfgWith(false, false, false), nullptr); }},
      {"  + rest bias learning & hold (ZARU/ZUPT)",
       [&](const std::vector<Sample>& d, const double*) { return runCore(d, cfgWith(true, true, false), nullptr); }},
      {"  + temperature compensation",
       [&](const std::vector<Sample>& d, const double*) { return runCore(d, cfgWith(true, true, true), nullptr); }},
      {"  + gyro scale/cross-axis calibration",
       [&](const std::vector<Sample>& d, const double* S) {
         // Calibration as produced by tools/gyro_scale_cal.py: S^-1 with a
         // 0.2 % residual error per element.
         static std::mt19937 rng(7);
         std::normal_distribution<double> N(0.0, 0.002);
         double Sp[9], M[9];
         for (int i = 0; i < 9; i++) Sp[i] = S[i] + N(rng);
         inv3(Sp, M);
         return runCore(d, cfgWith(true, true, true), M);
       }},
  };

  for (const auto& sc : scenarios) {
    std::printf("\nScenario: %s, warm-up +%.0f C, %d seeds\n", sc.name, sc.temp_rise_c, seeds);
    std::printf("  %-44s %12s %12s %12s\n", "variant", "final |err|", "max |err|", "rms err");
    std::printf("  %-44s %12s %12s %12s\n", "", "median deg", "median deg", "median deg");
    std::vector<std::vector<Result>> res(variants.size());
    for (int s = 0; s < seeds; s++) {
      double S[9];
      auto data = generate(sc, 1000 + s, S);
      for (size_t v = 0; v < variants.size(); v++) res[v].push_back(variants[v].run(data, S));
    }
    for (size_t v = 0; v < variants.size(); v++) {
      auto med = [&](double Result::*m) {
        std::vector<double> x;
        for (auto& r : res[v]) x.push_back(r.*m);
        std::sort(x.begin(), x.end());
        return x[x.size() / 2];
      };
      std::printf("  %-44s %12.2f %12.2f %12.2f\n", variants[v].name.c_str(), med(&Result::final_err),
                  med(&Result::max_err), med(&Result::rms_err));
    }
  }
  return 0;
}
