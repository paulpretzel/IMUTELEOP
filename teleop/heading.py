"""Magnetometer-free heading management for a multi-IMU arm harness.

A 6-axis IMU can never observe its own heading, but for teleoperation the
robot does not need absolute heading: it needs the orientation of each arm
segment *relative to the torso*. Two tools recover that:

1. Joint constraints (JointHeadingCorrector)
   The elbow and wrist are not ball joints. The elbow flexion axis is fixed in
   the upper arm, the forearm's long (pronation) axis is fixed in the forearm,
   and the two stay perpendicular. Given both sensors' tilt (which gravity
   keeps accurate), that constraint pins down the *relative* heading of the two
   sensors whenever the axes are not vertical. Any drift between them is then
   continuously pulled back to zero. (Approach after Laidig, Lehmann, Begin &
   Seel, "Magnetometer-free realtime inertial motion tracking by exploitation
   of kinematic constraints in 2-DoF joints", EMBC 2019.)

2. Pose recentering (ArmTracker.recenter)
   The shoulder *is* a ball joint, so the heading of the upper arm relative to
   the torso has no constraint. That single number per arm is the only
   quantity that can still drift; it is reset whenever the operator returns to
   the calibration pose (a "recenter" button or pedal, the same way VR
   headsets recenter). The firmware's bias learning keeps it slow between
   recenters.

Pure Python, no dependencies. Quaternions are (w, x, y, z), sensor -> world,
world z up, as sent by the ESP32 nodes.
"""

import math

from .quat import q_conj, q_mul, q_rotate, q_yaw, twist_z, v_normalize, wrap_pi


class JointHeadingCorrector:
    """Removes heading drift of a distal IMU relative to a proximal one.

    kind="2dof":  axis_prox (e.g. elbow flexion axis, in the proximal sensor
                  frame) and axis_dist (e.g. forearm long axis, in the distal
                  sensor frame) keep a constant angle, acos(cos_angle).
                  Elbow with the IMU near the wrist, or the wrist itself.
    kind="hinge": the same physical axis expressed in both sensor frames.
                  Elbow with the forearm IMU near the elbow (no pronation), or
                  a knee.

    The correction is a rotation about world z applied to the distal sensor.
    Gyro-bias drift grows linearly in time, so besides the offset the
    corrector learns the relative drift *rate* (a critically damped PI
    tracker): no steady-state lag, and the correction keeps extrapolating
    through poses that carry no heading information.
    """

    def __init__(self, axis_prox, axis_dist, kind="2dof", cos_angle=0.0, tau_s=2.0,
                 min_info=0.3, max_residual_deg=30.0, learn_rate=True, max_rate_dps=2.0):
        if kind not in ("2dof", "hinge"):
            raise ValueError("kind must be '2dof' or 'hinge'")
        self.axis_prox = v_normalize(axis_prox)
        self.axis_dist = v_normalize(axis_dist)
        self.kind = kind
        self.cos_angle = cos_angle
        self.tau_s = tau_s
        self.min_info = min_info
        self.max_residual = math.radians(max_residual_deg)
        self.learn_rate = learn_rate
        self.max_rate = math.radians(max_rate_dps)
        self.offset = 0.0  # rad, applied as q_yaw(offset) * q_dist
        self.rate = 0.0    # rad/s, learned relative drift rate
        self.last_residual = None
        self.last_info = 0.0

    def correct(self, q_dist):
        return q_mul(q_yaw(self.offset), q_dist)

    def residual(self, q_prox, q_dist):
        """(heading error of q_dist [rad], information weight in [0, 1]).

        The heading error is the extra world-z rotation that would make q_dist
        satisfy the joint constraint. Returns (None, 0) when the current pose
        carries no heading information (an axis is vertical).
        """
        ux, uy, uz = q_rotate(q_prox, self.axis_prox)
        vx, vy, vz = q_rotate(q_dist, self.axis_dist)
        if self.kind == "hinge":
            info = math.hypot(ux, uy) * math.hypot(vx, vy)
            if info < 1e-9:
                return None, 0.0
            return math.atan2(vx * uy - vy * ux, vx * ux + vy * uy), info
        # 2-DoF: u . Rz(d) v = A cos d + B sin d + uz vz must equal cos_angle.
        a = ux * vx + uy * vy
        b = uy * vx - ux * vy
        d = self.cos_angle - uz * vz
        r = math.hypot(a, b)
        if r < 1e-9:
            return None, 0.0
        ratio = max(-1.0, min(1.0, d / r))
        phi = math.atan2(b, a)
        alpha = math.acos(ratio)
        roots = (wrap_pi(phi + alpha), wrap_pi(phi - alpha))
        delta = min(roots, key=abs)
        info = math.sqrt(max(r * r - d * d, 0.0))
        return delta, info

    def reset(self):
        self.offset = self.rate = 0.0

    def update(self, q_prox, q_dist, dt):
        """Feed one synchronized pair; returns the corrected distal quaternion."""
        if self.learn_rate:
            self.offset = wrap_pi(self.offset + self.rate * dt)
        corrected = self.correct(q_dist)
        delta, info = self.residual(q_prox, corrected)
        self.last_residual, self.last_info = delta, info
        if delta is not None and info >= self.min_info and abs(delta) <= self.max_residual:
            k = info / self.tau_s
            self.offset = wrap_pi(self.offset + min(1.0, k * dt) * delta)
            if self.learn_rate:
                # Critical damping for s^2 + k s + ki: ki = k^2 / 4.
                self.rate += 0.25 * k * k * dt * delta
                self.rate = max(-self.max_rate, min(self.max_rate, self.rate))
            corrected = self.correct(q_dist)
        return corrected


class ArmTracker:
    """One arm: torso, upper arm, forearm and (optionally) hand IMUs.

    Workflow:
      1. calibrate_npose(q) with the operator standing upright, arms hanging
         straight down, palms facing the thighs, looking forward.
      2. update(q, dt) every frame -> segment orientations in the torso body
         frame (x forward, y left, z up; identity = N-pose). Feed these to the
         G1 retargeting / IK.
      3. recenter(q) whenever the operator is back in the N-pose (button,
         pedal, or a detected rest pose) to remove residual shoulder heading
         drift.

    `q` is a dict with keys "torso", "upper", "fore" and optionally "hand".
    """

    SEGMENTS = ("torso", "upper", "fore", "hand")

    def __init__(self, elbow=None, wrist=None, torso_forward_axis=(1.0, 0.0, 0.0)):
        self.elbow = elbow
        self.wrist = wrist
        self.torso_forward_axis = v_normalize(torso_forward_axis)
        self.s2b = {}         # sensor -> body alignment, per segment
        self.recenter_offset = {}

    @property
    def calibrated(self):
        return "torso" in self.s2b

    def calibrate_npose(self, q):
        f = q_rotate(q["torso"], self.torso_forward_axis)
        if math.hypot(f[0], f[1]) < 0.3:
            raise ValueError("torso_forward_axis is nearly vertical; pick the sensor axis "
                             "that points forward out of the chest")
        body = q_yaw(math.atan2(f[1], f[0]))
        self.s2b = {name: q_mul(q_conj(qs), body) for name, qs in q.items() if name in self.SEGMENTS}
        self.recenter_offset = {name: 0.0 for name in self.s2b}
        for c in (self.elbow, self.wrist):
            if c is not None:
                c.reset()

    def recenter(self, q):
        """Heading-only realignment; the operator must be in the N-pose."""
        if not self.calibrated:
            raise RuntimeError("calibrate_npose() first")
        b_torso = q_mul(q["torso"], self.s2b["torso"])
        for name in self.s2b:
            if name == "torso" or name not in q:
                continue
            b = q_mul(q[name], self.s2b[name])
            self.recenter_offset[name] = twist_z(q_mul(b_torso, q_conj(b)))
        for c in (self.elbow, self.wrist):
            if c is not None:
                c.reset()

    def corrected_sensors(self, q, dt):
        """Sensor quaternions with recentering and joint corrections applied."""
        out = {"torso": q["torso"]}
        for name in ("upper", "fore", "hand"):
            if name in q:
                out[name] = q_mul(q_yaw(self.recenter_offset.get(name, 0.0)), q[name])
        if self.elbow is not None and "fore" in out:
            out["fore"] = self.elbow.update(out["upper"], out["fore"], dt)
        if self.wrist is not None and "hand" in out:
            out["hand"] = self.wrist.update(out["fore"], out["hand"], dt)
        return out

    def update(self, q, dt):
        """Returns {segment: orientation relative to the torso body frame}."""
        if not self.calibrated:
            raise RuntimeError("calibrate_npose() first")
        s = self.corrected_sensors(q, dt)
        body = {name: q_mul(s[name], self.s2b[name]) for name in s if name in self.s2b}
        inv_torso = q_conj(body["torso"])
        return {name: q_mul(inv_torso, b) for name, b in body.items() if name != "torso"}


# --------------------------------------------------------------------------
# Joint axis calibration helpers
# --------------------------------------------------------------------------
def rotation_axis_world(q_before, q_after):
    """World-frame rotation axis of a segment that moved from q_before to q_after.

    Use it on a pure joint motion, e.g. flexing only the elbow by ~90 deg with
    the upper arm held still, to find the elbow flexion axis in the world.
    """
    d = q_mul(q_after, q_conj(q_before))
    if d[0] < 0:
        d = tuple(-c for c in d)
    return v_normalize(d[1:])


def axis_in_sensor(q_sensor, axis_world):
    """Express a world-frame axis in a sensor's own frame."""
    return q_rotate(q_conj(q_sensor), axis_world)


def elbow_axes_from_poses(q_npose, q_flexed, forearm_imu_near_wrist=True):
    """Elbow constraint axes from two poses.

    q_npose:  sensors in the N-pose (arm hanging straight down).
    q_flexed: same, after flexing only the elbow ~90 deg (upper arm still).
    Returns a JointHeadingCorrector for the elbow.
    """
    flex_w = rotation_axis_world(q_npose["fore"], q_flexed["fore"])
    axis_upper = axis_in_sensor(q_npose["upper"], flex_w)
    if forearm_imu_near_wrist:
        # Forearm long axis points straight down in the N-pose.
        long_fore = axis_in_sensor(q_npose["fore"], (0.0, 0.0, -1.0))
        return JointHeadingCorrector(axis_upper, long_fore, kind="2dof")
    return JointHeadingCorrector(axis_upper, axis_in_sensor(q_npose["fore"], flex_w), kind="hinge")


def wrist_axes_from_poses(q_npose, q_flexed):
    """Wrist (flexion + deviation, universal joint) axes from two poses.

    q_flexed: after flexing only the wrist ~60 deg (forearm still).
    """
    flex_w = rotation_axis_world(q_npose["hand"], q_flexed["hand"])
    down = (0.0, 0.0, -1.0)
    dev_w = v_normalize((flex_w[1] * down[2] - flex_w[2] * down[1],
                         flex_w[2] * down[0] - flex_w[0] * down[2],
                         flex_w[0] * down[1] - flex_w[1] * down[0]))
    return JointHeadingCorrector(axis_in_sensor(q_npose["fore"], flex_w),
                                 axis_in_sensor(q_npose["hand"], dev_w), kind="2dof")


__all__ = ["JointHeadingCorrector", "ArmTracker", "rotation_axis_world", "axis_in_sensor",
           "elbow_axes_from_poses", "wrist_axes_from_poses"]
