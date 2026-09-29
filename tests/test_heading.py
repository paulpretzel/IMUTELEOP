"""Tests for teleop.heading on synthetic arm motion with independent sensor drift.

Run:  python3 -m unittest discover -s tests -v
"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from teleop.heading import (ArmTracker, JointHeadingCorrector, elbow_axes_from_poses,  # noqa: E402
                            wrist_axes_from_poses)
from teleop.quat import (q_angle, q_conj, q_from_axis_angle, q_mul, q_normalize, q_rotate,  # noqa: E402
                         q_yaw, twist_z)

DEG = math.pi / 180.0
X, Y, Z = (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)


def rot(axis, deg):
    return q_from_axis_angle(axis, deg * DEG)


def random_mount(rng):
    return q_normalize((rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 1), rng.gauss(0, 1)))


class ArmSim:
    """Ground-truth arm. Segment frames are aligned with the world in the
    N-pose (x forward, y left, z up; arms hang along -z). Each IMU is strapped
    on with an arbitrary, unknown mounting rotation and drifts in heading at
    its own constant rate, which is exactly the 6-axis failure mode."""

    def __init__(self, seed=1, drift_dps=None, hinge=False):
        rng = random.Random(seed)
        self.mount = {n: random_mount(rng) for n in ("torso", "upper", "fore", "hand")}
        self.drift = drift_dps or {"torso": 0.2, "upper": -0.4, "fore": 0.6, "hand": -0.5}
        self.hinge = hinge
        self.torso_heading = 20.0  # operator is not facing world x

    def segments(self, t):
        torso = q_mul(q_yaw(self.torso_heading * DEG), rot(Y, 5 * math.sin(0.3 * t)))
        upper = q_mul(torso, q_mul(rot(Z, 50 * math.sin(0.23 * t)),
                                   q_mul(rot(Y, -45 - 40 * math.sin(0.37 * t)),
                                         rot(Z, 30 * math.sin(0.51 * t)))))
        flex = -70 - 60 * math.sin(0.61 * t)
        fore = q_mul(upper, rot(Y, flex))
        if not self.hinge:
            fore = q_mul(fore, rot(Z, 60 * math.sin(0.43 * t)))  # pronation
        hand = q_mul(fore, q_mul(rot(Y, 40 * math.sin(0.71 * t)), rot(X, 15 * math.sin(0.83 * t))))
        return {"torso": torso, "upper": upper, "fore": fore, "hand": hand}

    def npose(self):
        return {"torso": q_yaw(self.torso_heading * DEG), "upper": q_yaw(self.torso_heading * DEG),
                "fore": q_yaw(self.torso_heading * DEG), "hand": q_yaw(self.torso_heading * DEG)}

    def sensors(self, seg, t, drift=True):
        out = {}
        for n, q in seg.items():
            qs = q_mul(q, self.mount[n])
            if drift:
                qs = q_mul(q_yaw(self.drift[n] * DEG * t), qs)
            out[n] = qs
        return out


def rel_heading_err_deg(q_est_a, q_true_a, q_est_b, q_true_b):
    """Heading drift of b relative to a (0 = the pair is consistent)."""
    da = twist_z(q_mul(q_est_a, q_conj(q_true_a)))
    db = twist_z(q_mul(q_est_b, q_conj(q_true_b)))
    return abs(math.degrees(math.atan2(math.sin(db - da), math.cos(db - da))))


class TestJointHeadingCorrector(unittest.TestCase):
    def _run(self, hinge, drift=True, seconds=120.0, fs=60.0):
        sim = ArmSim(seed=3, hinge=hinge)
        a_up = q_rotate(q_conj(sim.mount["upper"]), Y)
        a_fo = q_rotate(q_conj(sim.mount["fore"]), Y if hinge else Z)
        c = JointHeadingCorrector(a_up, a_fo, kind="hinge" if hinge else "2dof")
        dt, t, errs, raw = 1.0 / fs, 0.0, [], []
        while t < seconds:
            seg = sim.segments(t)
            true = sim.sensors(seg, t, drift=False)
            est = sim.sensors(seg, t, drift=drift)
            fore = c.update(est["upper"], est["fore"], dt)
            if t > 30.0:
                errs.append(rel_heading_err_deg(est["upper"], true["upper"], fore, true["fore"]))
                raw.append(rel_heading_err_deg(est["upper"], true["upper"], est["fore"], true["fore"]))
            t += dt
        return max(errs), max(raw)

    def test_2dof_removes_relative_drift(self):
        err, raw = self._run(hinge=False)
        self.assertGreater(raw, 60.0)  # 1 deg/s relative drift for 2 minutes
        self.assertLess(err, 2.0)

    def test_hinge_removes_relative_drift(self):
        err, raw = self._run(hinge=True)
        self.assertGreater(raw, 60.0)
        self.assertLess(err, 2.0)

    def test_no_false_correction_without_drift(self):
        err, _ = self._run(hinge=False, drift=False)
        self.assertLess(err, 0.5)

    def test_vertical_axis_is_uninformative(self):
        c = JointHeadingCorrector(Z, Z, kind="hinge")
        delta, info = c.residual((1, 0, 0, 0), (1, 0, 0, 0))
        self.assertIsNone(delta)
        self.assertEqual(info, 0.0)


class TestArmTracker(unittest.TestCase):
    def test_npose_is_identity_and_recenter_removes_shoulder_drift(self):
        sim = ArmSim(seed=5)
        fwd = q_rotate(q_conj(sim.mount["torso"]), X)  # chest sensor axis that points forward
        tracker = ArmTracker(torso_forward_axis=fwd)
        tracker.calibrate_npose(sim.sensors(sim.npose(), 0.0))
        rel = tracker.update(sim.sensors(sim.npose(), 0.0), 1 / 60)
        for q in rel.values():
            self.assertLess(q_angle(q), 1e-6)

        # 60 s later, back in the N-pose: drift has accumulated...
        drifted = sim.sensors(sim.npose(), 60.0)
        self.assertGreater(math.degrees(q_angle(tracker.update(drifted, 1 / 60)["upper"])), 30.0)
        # ...and recentering removes it.
        tracker.recenter(drifted)
        for q in tracker.update(drifted, 1 / 60).values():
            self.assertLess(math.degrees(q_angle(q)), 1e-3)

    def test_full_chain_with_calibrated_axes(self):
        """Axes from the two-pose calibration, then 3 minutes of drifting motion:
        elbow and wrist angles stay correct; only the (recenterable) shoulder
        heading is allowed to drift."""
        sim = ArmSim(seed=7)
        npose = sim.npose()
        q0 = sim.sensors(npose, 0.0, drift=False)
        elbow_flexed = dict(npose, fore=q_mul(npose["upper"], rot(Y, -90)),
                            hand=q_mul(npose["upper"], rot(Y, -90)))
        wrist_flexed = dict(npose, hand=q_mul(npose["fore"], rot(Y, 60)))
        elbow = elbow_axes_from_poses(q0, sim.sensors(elbow_flexed, 0.0, drift=False))
        wrist = wrist_axes_from_poses(q0, sim.sensors(wrist_flexed, 0.0, drift=False))
        fwd = q_rotate(q_conj(sim.mount["torso"]), X)
        tracker = ArmTracker(elbow=elbow, wrist=wrist, torso_forward_axis=fwd)
        tracker.calibrate_npose(q0)

        dt, t, worst = 1 / 60, 0.0, {"elbow": 0.0, "wrist": 0.0}
        while t < 180.0:
            seg = sim.segments(t)
            rel = tracker.update(sim.sensors(seg, t), dt)
            if t > 30.0:
                # Ground truth relative joint rotations.
                true_elbow = q_mul(q_conj(seg["upper"]), seg["fore"])
                true_wrist = q_mul(q_conj(seg["fore"]), seg["hand"])
                est_elbow = q_mul(q_conj(rel["upper"]), rel["fore"])
                est_wrist = q_mul(q_conj(rel["fore"]), rel["hand"])
                worst["elbow"] = max(worst["elbow"], math.degrees(q_angle(q_mul(q_conj(true_elbow), est_elbow))))
                worst["wrist"] = max(worst["wrist"], math.degrees(q_angle(q_mul(q_conj(true_wrist), est_wrist))))
            t += dt
        self.assertLess(worst["elbow"], 3.0, worst)
        self.assertLess(worst["wrist"], 3.0, worst)


if __name__ == "__main__":
    unittest.main()
