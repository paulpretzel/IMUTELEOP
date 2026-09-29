#!/usr/bin/env python3
"""Gyro scale-factor and cross-axis calibration for one MPU6050 node.

Why: the MPU6050 gyro sensitivity is only specified to +-3 % and cross-axis
sensitivity to +-2 %. Every degree the arm rotates is integrated with that
error, and in 3D arm motion those errors do not cancel -- they show up as
heading drift. The simulator (tests/sim_yaw_drift.cpp) shows this is the
single largest drift source once bias is handled.

Procedure (needs a box or block with square faces, e.g. tape the sensor into
a small cardboard box, and a straight edge on the table):
  For each sensor axis X, Y, Z:
    1. Lay the box so that axis points straight up, pushed against the edge.
    2. Hold still, press Enter.
    3. Rotate the box N full turns about the vertical, either direction, and
       push it back against the edge in the same orientation.
    4. Hold still, press Enter.
The firmware streams the integral of the bias-corrected raw gyro per axis
(fields 11-13 of the packet), so the result is M = diag(+-N*360) * I^-1, where
column k of I is the integrated vector during the turn about axis k. Paste the
printed matrix into firmware/YawDriftSocket/calibration.h.
"""

import argparse
import socket
import sys
import time


def recv_integ(sock, settle_s=0.3):
    """Latest integrated-angle vector (deg) after letting stale packets drain."""
    deadline = time.time() + settle_s
    last = None
    while time.time() < deadline or last is None:
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue
        parts = data.decode("ascii", "ignore").strip().split(",")
        if len(parts) < 14:
            sys.exit("Packets have no integral fields: flash the new firmware first.")
        last = tuple(float(p) for p in parts[10:13])
        still = parts[5] == "1"
    if not still:
        print("  (warning: the node did not report 'still' -- hold it steady against the edge)")
    return last


def inv3(m):
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    A, B, C = e * i - f * h, -(d * i - f * g), d * h - e * g
    det = a * A + b * B + c * C
    if abs(det) < 1e-9:
        raise ValueError("singular measurement matrix -- did every axis get rotated?")
    return [[A / det, -(b * i - c * h) / det, (b * f - c * e) / det],
            [B / det, (a * i - c * g) / det, -(a * f - c * d) / det],
            [C / det, -(a * h - b * g) / det, (a * e - b * d) / det]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--turns", type=int, default=5, help="full turns per axis (more = more accurate)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)
    print(f"Listening on UDP {args.bind}:{args.port}. Close YawDriftReceiver.py first (same port).")

    cols = []
    for k, axis in enumerate("XYZ"):
        input(f"\n[{axis}] Sensor {axis} axis pointing UP, against the edge, still. Press Enter...")
        a = recv_integ(sock)
        input(f"[{axis}] Now rotate {args.turns} full turns about the vertical, return to the edge, "
              f"hold still. Press Enter...")
        b = recv_integ(sock)
        d = [b[i] - a[i] for i in range(3)]
        print(f"  integrated: x {d[0]:+9.2f}  y {d[1]:+9.2f}  z {d[2]:+9.2f} deg"
              f"   (expected {args.turns * 360} on {axis})")
        if abs(abs(d[k]) - args.turns * 360) > 0.15 * args.turns * 360:
            print("  WARNING: more than 15 % off -- wrong axis up, or turns miscounted? Repeat this axis.")
        cols.append(d)

    # I has the integrated vectors as columns; the true rotation for column k
    # is theta_k * e_k with theta_k = +-turns*360 (sign from the measurement).
    I = [[cols[c][r] for c in range(3)] for r in range(3)]
    Iinv = inv3(I)
    theta = [args.turns * 360.0 * (1 if cols[k][k] >= 0 else -1) for k in range(3)]
    M = [[theta[r] * Iinv[r][c] for c in range(3)] for r in range(3)]

    print("\nScale errors: " + "  ".join(f"{a} {100 * (1 / M[i][i] - 1):+.2f} %" for i, a in enumerate("XYZ")))
    print("\nPaste into firmware/YawDriftSocket/calibration.h:\n")
    print("static const float GYRO_CAL_M[9] = {")
    for r in range(3):
        print("    " + ", ".join(f"{M[r][c]:.6f}f" for c in range(3)) + ",")
    print("};")


if __name__ == "__main__":
    main()
