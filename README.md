# IMUTELEOP: magnetometer-free arm teleoperation for the Unitree G1

ESP32 + MPU6050 (6-axis) wearable nodes streaming orientation for arm teleop,
with the goal of losing as little yaw (heading) as possible.

```
firmware/YawDriftSocket/   ESP32 sketch (open this folder in the Arduino IDE)
  YawDriftSocket.ino       FIFO-driven 500 Hz sampling, fusion, 60 Hz UDP on core 0
  imu_fusion.h             portable orientation core (also compiled by the tests)
  calibration.h            per-board gyro scale/cross-axis matrix
  secrets.example.h        copy to secrets.h (git-ignored) and fill in
YawDriftReceiver.py        VPython dashboard + CSV logger (old and new packets)
teleop/heading.py          multi-IMU arm tracker: joint-constraint heading
                           correction + recentering, output relative to torso
tools/gyro_scale_cal.py    5-minute gyro scale/cross-axis calibration
tests/                     drift simulator + unit tests
```

## The one fact that drives everything

Gravity tells a 6-axis filter which way is down, and nothing about which way
it is facing. The Mahony/Madgwick accelerometer correction is perpendicular to
the estimated vertical, so it **cannot** correct heading, whatever the gains.
Heading error is the integral of every gyro error:

    yaw error = ∫ ( bias(temperature, t) + scale & cross-axis error × ω + noise ) dt

So you can't filter yaw drift away. You have to shrink each term at its
source, and then use the body itself as the reference. Only relative
orientations matter for driving the robot.

## What the firmware does now

| Error source | Fix |
|---|---|
| Bias left over after boot calibration, warm-up drift | Bias is re-learned whenever the node is still (ZARU), and orientation is held while still (ZUPT). A bias-vs-temperature slope is fitted online from those rests, so the bias keeps tracking warm-up while you move. |
| Gyro scale ±3 %, cross-axis ±2 % (datasheet) | 3×3 correction matrix from `tools/gyro_scale_cal.py` |
| Lost or mistimed samples on Wi-Fi stalls | The MPU6050 samples itself at exactly 500 Hz into its FIFO (~170 ms buffer). UDP runs in its own task on core 0. Wi-Fi modem sleep is disabled. |
| MPU sample clock error (a few %) | The real sample rate is measured against the ESP32 crystal and used as dt |
| Arm-swing accelerations corrupting tilt | Accelerometer correction fades out when \|a\| ≠ 1 g |
| Integration error at 800 °/s | Exact quaternion exponential integration |

### Simulated result

`tests/sim_yaw_drift.cpp` puts a simulated MPU6050 on a simulated arm: random
swings up to several hundred °/s, a 10 °C warm-up, datasheet-typical bias,
scale, cross-axis error and noise. It runs the **original firmware filter** and
the new core on identical data. Numbers are the median over 20 random sensors,
10-minute sessions:

| | frequent rests (every 8–20 s) | | sparse rests (every 40–80 s) | |
|---|---|---|---|---|
| | final | max | final | max |
| original firmware | 40.2° | 44.9° | 19.4° | 37.4° |
| + exact integration, accel gating | 38.0° | 42.9° | 19.6° | 33.6° |
| + rest bias learning / hold | 20.0° | 28.2° | 19.1° | 29.8° |
| + temperature compensation | 20.8° | 28.8° | 18.7° | 29.8° |
| **+ gyro scale / cross-axis cal** | **3.7°** | **4.6°** | **2.9°** | **5.7°** |

Takeaways:
- **Calibrate the gyro scale.** Once bias is handled, scale and cross-axis
  error dominate. It's a one-time, five-minute job per board.
- Bias learning needs real rests: the arm resting on a table or armrest
  for about 1 s. Design the operator station so that happens naturally.
- Temperature compensation matters most when rests are rare. It is cheap,
  so it stays on.
- These are simulation results. Real hardware will differ, so run the
  return-to-pose protocol below to measure it.

## Teleop: turning 6-axis nodes into G1 arm commands

The G1 needs each arm segment's orientation **relative to the torso**, not
absolute heading. `teleop/heading.py` makes that nearly drift-free:

1. **Elbow and wrist constraints.** These are not ball joints. The elbow
   flexion axis (fixed in the upper arm) and the forearm long axis (fixed in
   the forearm) stay perpendicular. With both sensors' tilt known from
   gravity, that pins down their *relative* heading.
   `JointHeadingCorrector` pulls the drift between adjacent sensors back to
   zero continuously. It also learns the drift rate, so it keeps correcting
   through poses where the axis is momentarily vertical. In tests with
   1 °/s of relative drift and 1° of per-frame noise, elbow and wrist heading
   stay within about 1.3° (95th percentile). Without correction, the error
   passes 60°.
2. **Recentering for the shoulder.** The shoulder is a ball joint, so upper
   arm vs torso heading has no constraint. That is the **one** number per arm
   that can still drift, slowly, thanks to the firmware. `ArmTracker.recenter()`
   resets it whenever the operator returns to the N-pose. Bind it to a foot
   pedal or button, the same way VR headsets recenter.

```python
from teleop import ArmTracker, elbow_axes_from_poses, wrist_axes_from_poses

# one-time per session: N-pose (arms hanging, palms in), then flex elbow 90°, then flex wrist 60°
elbow = elbow_axes_from_poses(q_npose, q_elbow_flexed)
wrist = wrist_axes_from_poses(q_npose, q_wrist_flexed)
arm = ArmTracker(elbow, wrist, torso_forward_axis=(0, 0, 1))  # chest-sensor axis pointing forward
arm.calibrate_npose(q_npose)

rel = arm.update({"torso": qt, "upper": qu, "fore": qf, "hand": qh}, dt)
# rel["upper"], rel["fore"], rel["hand"]: orientations in the torso frame
# (x forward, y left, z up; identity = N-pose) -> G1 retargeting / IK
```

Next steps toward the robot:
- **Multi-node transport.** Five nodes over ESP-NOW into one receiver ESP32,
  as the write-up plans. Each packet needs a node id; the fusion stays on
  each node.
- **Retargeting.** Either decompose `rel` into the G1 shoulder
  pitch/roll/yaw, elbow and wrist roll/pitch/yaw joint order (take the axes
  and zero pose from the G1 URDF), or compute wrist poses by forward
  kinematics with the operator's segment lengths and feed them to an IK
  solver, as Unitree's own teleop stack does.

## Quick start

1. `cp firmware/YawDriftSocket/secrets.example.h firmware/YawDriftSocket/secrets.h`
   and fill in the Wi-Fi details and the receiver IP.
2. Flash `firmware/YawDriftSocket/` (ESP32 Arduino core). No extra libraries
   are needed; the MPU6050 driver is built in.
3. Keep the node still during the boot countdown. If it moves, it retries.
4. `python3 YawDriftReceiver.py` (needs `pip install vpython`).
5. Once per board: `python3 tools/gyro_scale_cal.py`, then paste the matrix
   into `calibration.h` and reflash.

The serial monitor prints `[STAT]` every 5 s: measured sample rate,
temperature, learned bias and temperature slope, stillness, FIFO overflows
(these should stay 0).

### Measuring drift properly: return-to-pose protocol

Any single "yaw" number has a gimbal-lock pose somewhere. The old Y-axis
compass is singular when the sensor's Y axis points straight up. For drift
tests, use the new **Δ HEADING vs ZERO POSE** card instead:

1. Put the node against a fixed physical stop (a block taped to the desk).
2. Press **Z** to zero.
3. Do the motion under test.
4. Put it back against the stop. The card now reads the accumulated heading
   drift, whatever the orientation.

It is also logged as `Pose_dHeading` in the CSV.

## Tests

```bash
g++ -O2 -std=c++17 -o /tmp/test_fusion tests/test_fusion.cpp && /tmp/test_fusion
g++ -O2 -std=c++17 -o /tmp/sim tests/sim_yaw_drift.cpp && /tmp/sim 20
python3 -m unittest discover -s tests -v
```

## Known limits

- A very slow, smooth rotation (under 1 °/s for over 0.5 s) looks like rest.
  The bias change per rest is capped at 0.5 °/s to bound the damage
  (`bias_max_step_dps`).
- The new firmware was compile-checked and its math simulated, but it has not
  yet run on real hardware. Watch the first boot on the serial monitor.
