"""Minimal quaternion helpers (pure Python, no numpy).

Quaternions are tuples (w, x, y, z) that rotate sensor-frame vectors into the
world frame (world z = up), matching the ESP32 firmware.
"""

import math


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    if n < 1e-12:
        raise ValueError("zero quaternion")
    return tuple(c / n for c in q)


def q_from_axis_angle(axis, angle):
    ax, ay, az = v_normalize(axis)
    s = math.sin(0.5 * angle)
    return (math.cos(0.5 * angle), ax * s, ay * s, az * s)


def q_yaw(angle):
    """Rotation by `angle` [rad] about world z."""
    return (math.cos(0.5 * angle), 0.0, 0.0, math.sin(0.5 * angle))


def q_rotate(q, v):
    """Rotate vector v by q (sensor frame -> world frame)."""
    w, x, y, z = q
    vx, vy, vz = v
    # v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
    cx = y * vz - z * vy
    cy = z * vx - x * vz
    cz = x * vy - y * vx
    return (vx + 2.0 * (w * cx + y * cz - z * cy),
            vy + 2.0 * (w * cy + z * cx - x * cz),
            vz + 2.0 * (w * cz + x * cy - y * cx))


def q_angle(q):
    """Total rotation angle of q in [0, pi]."""
    w = min(1.0, abs(q[0]))
    return 2.0 * math.acos(w)


def twist_z(q):
    """Angle [rad] of the rotation about world z contained in q (swing-twist).

    Well defined whenever q is not a pure 180 deg flip about a horizontal axis;
    ideal for small *differences* between two orientations.
    """
    return wrap_pi(2.0 * math.atan2(q[3], q[0]))


def v_normalize(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < 1e-12:
        raise ValueError("zero vector")
    return (v[0] / n, v[1] / n, v[2] / n)


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi
