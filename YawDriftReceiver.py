#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YawDrift Experimental Dashboard  --  Lean UDP receiver + VPython UI
===================================================================
Optimized for formal data collection. Stripped of scrolling graphs and
consoles to minimize rendering overhead. Features Y-Axis compass tracking.
"""

from vpython import *  # noqa: F401,F403
import argparse
import atexit
import math
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime

# ==========================================================================
# CONFIG
# ==========================================================================
NOMINAL_HZ = 60.0
UI_HZ = 10.0
FRAME_HZ = 100
DASH_W = 1000
STALE_S, LOST_S = 0.5, 2.0
STILL_DPS = 2.0
DRIFT_MIN_SPAN_S = 5.0
DRIFT_MAX_SPAN_S = 60.0
SPEED_LAG = 6
RAD2DEG = 180.0 / math.pi

# Theme
C_PANEL, C_EDGE = "#161b26", "#2b3347"
C_TXT, C_MUTED = "#e6e9f2", "#8a93a8"
C_CYAN, C_GREEN, C_AMBER, C_RED, C_VIOLET = "#4dd0e1", "#4ade80", "#fbbf24", "#f87171", "#a78bfa"
MONO = "font-family:Consolas,'SF Mono',Menlo,monospace;font-variant-numeric:tabular-nums;"


# ==========================================================================
# MATH & LOGIC
# ==========================================================================
def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def q_normalize(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return None, 0.0
    return (w / n, x / n, y / n, z / n), n


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def euler_deg(w, x, y, z):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG


def body_rates_dps(q_old, q_new, dt):
    if dt <= 1e-6:
        return 0.0, 0.0, 0.0
    dw, dx, dy, dz = q_mul(q_conj(q_old), q_new)
    if dw < 0.0:
        dw, dx, dy, dz = -dw, -dx, -dy, -dz
    vn = math.sqrt(dx * dx + dy * dy + dz * dz)
    if vn < 1e-12:
        return 0.0, 0.0, 0.0
    k = 2.0 * math.atan2(vn, dw) / vn / dt * RAD2DEG
    return dx * k, dy * k, dz * k


def twist_z_deg(q):
    """Rotation about world z contained in q (swing-twist decomposition)."""
    return wrap180(math.degrees(2.0 * math.atan2(q[3], q[0])))


def pose_delta_heading_deg(q, q_ref):
    """Heading change between two orientations, measured about world z.

    Unlike any single "yaw" number, this has no gimbal-lock pose: it is the
    world-z twist of q * q_ref^-1, which is well defined unless the sensor is
    flipped ~180 deg about a horizontal axis relative to q_ref. Return the
    sensor to the reference pose and this reads the accumulated drift directly.
    """
    return twist_z_deg(q_mul(q, q_conj(q_ref)))


PACKET_FIELDS_V1 = 5   # ms,W,X,Y,Z
PACKET_FIELDS_V2 = 14  # + still,tempC,bias xyz,integ xyz,fifo_overflows


def parse_packet(text):
    """Parse one telemetry line from either firmware version. Raises ValueError."""
    parts = text.strip().split(",")
    if len(parts) not in (PACKET_FIELDS_V1, PACKET_FIELDS_V2):
        raise ValueError("field count")
    vals = [float(p) for p in parts]
    if not all(math.isfinite(v) for v in vals):
        raise ValueError("non-finite")
    pkt = dict(t_ms=vals[0], wxyz=tuple(vals[1:5]), still=None, temp=None, bias=None, integ=None, ovf=None)
    if len(vals) == PACKET_FIELDS_V2:
        pkt.update(still=vals[5] >= 0.5, temp=vals[6], bias=tuple(vals[7:10]), integ=tuple(vals[10:13]),
                   ovf=int(vals[13]))
    return pkt


def lsq_slope(pts):
    n = len(pts)
    if n < 3:
        return None
    mt = sum(p[0] for p in pts) / n
    mv = sum(p[1] for p in pts) / n
    den = sum((p[0] - mt) ** 2 for p in pts)
    if den < 1e-12:
        return None
    return sum((p[0] - mt) * (p[1] - mv) for p in pts) / den


class YawTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.last_raw = None
        self.unwrapped = 0.0
        self.zero = 0.0

    def update(self, raw):
        first = self.last_raw is None
        if first:
            self.unwrapped = raw
            self.zero = raw
        else:
            self.unwrapped += wrap180(raw - self.last_raw)
        self.last_raw = raw
        return first

    def rezero(self):
        self.zero = self.unwrapped

    @property
    def drift_total(self):
        return self.unwrapped - self.zero

    @property
    def bounded(self):
        return wrap180(self.unwrapped - self.zero)


class MotionEstimator:
    def __init__(self):
        self.hist = deque(maxlen=SPEED_LAG + 1)
        self.reset()

    def reset(self):
        self.hist.clear()
        self.wx = self.wy = self.wz = self.speed = 0.0

    def push(self, t_s, q):
        self.hist.append((t_s, q))
        if len(self.hist) >= 2:
            t0, q0 = self.hist[0]
            self.wx, self.wy, self.wz = body_rates_dps(q0, q, t_s - t0)
            self.speed = math.sqrt(self.wx ** 2 + self.wy ** 2 + self.wz ** 2)


class DriftEstimator:
    def __init__(self):
        self.pts = deque()

    def reset(self):
        self.pts.clear()

    def add(self, t_s, yaw_unwrapped, peak_speed):
        if peak_speed > STILL_DPS:
            self.pts.clear()
            return
        self.pts.append((t_s, yaw_unwrapped))
        while self.pts and t_s - self.pts[0][0] > DRIFT_MAX_SPAN_S:
            self.pts.popleft()

    @property
    def span(self):
        return (self.pts[-1][0] - self.pts[0][0]) if len(self.pts) > 1 else 0.0

    def rate_dpm(self):
        if self.span < DRIFT_MIN_SPAN_S:
            return None
        s = lsq_slope(self.pts)
        return None if s is None else s * 60.0


class LinkStats:
    def __init__(self, nominal_hz):
        self.expected_ms = 1000.0 / nominal_hz
        self.arrivals = deque()
        self.gaps = deque(maxlen=120)
        self.received = self.missed = self.bad = self.restarts = 0
        self.last_rx = None
        self.last_dev_ms = None
        self.src = None

    def on_packet(self, arrive, dev_ms, addr):
        restart = False
        if self.last_rx is not None:
            gap = (arrive - self.last_rx) * 1000.0
            if gap < 500.0:
                self.gaps.append(gap)
        if self.last_dev_ms is not None:
            d = dev_ms - self.last_dev_ms
            if d < -500.0:
                restart = True
                self.restarts += 1
            elif d > 1.5 * self.expected_ms:
                self.missed += max(0, int(round(d / self.expected_ms)) - 1)
        self.last_rx, self.last_dev_ms, self.src = arrive, dev_ms, addr
        self.received += 1
        self.arrivals.append(arrive)
        while self.arrivals and arrive - self.arrivals[0] > 2.0:
            self.arrivals.popleft()
        return restart

    def rate_hz(self):
        if len(self.arrivals) < 3:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0 else 0.0

    def jitter_ms(self):
        n = len(self.gaps)
        if n < 5:
            return 0.0
        m = sum(self.gaps) / n
        return math.sqrt(sum((g - m) ** 2 for g in self.gaps) / n)

    def loss_pct(self):
        tot = self.received + self.missed
        return 100.0 * self.missed / tot if tot else 0.0

    def state(self, now):
        if self.last_rx is None:
            return "WAITING", C_AMBER
        age = now - self.last_rx
        if age < STALE_S:
            return "LIVE", C_GREEN
        if age < LOST_S:
            return "STALE", C_AMBER
        return "LOST", C_RED


class UdpReceiver(threading.Thread):
    def __init__(self, bind_ip, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        self.sock.bind((bind_ip, port))
        self.sock.settimeout(0.25)
        self.q = deque(maxlen=8192)
        self.running = True

    def run(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            self.q.append((time.perf_counter(), addr, data))

    def stop(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return "?"


# ==========================================================================
# HTML HELPERS
# ==========================================================================
def pill(txt, col):
    return (f"<span style='background:{col}22;color:{col};border:1px solid {col}66;border-radius:10px;"
            f"padding:1px 9px;margin-right:6px;font-size:12px;white-space:nowrap;{MONO}'>{txt}</span>")


def card(lbl, value, unit="", sub="", accent=C_CYAN, grow=1):
    return (f"<div style='flex:{grow} 1 140px;background:{C_PANEL};border:1px solid {C_EDGE};"
            f"border-left:3px solid {accent};border-radius:6px;padding:7px 12px;'>"
            f"<div style='color:{C_MUTED};font-size:10px;letter-spacing:1.5px'>{lbl}</div>"
            f"<div style='color:{C_TXT};font-size:32px;font-weight:600;line-height:1.25;{MONO}'>{value}"
            f"<span style='color:{C_MUTED};font-size:14px;font-weight:400'> {unit}</span></div>"
            f"<div style='color:{C_MUTED};font-size:12px;min-height:14px;{MONO}'>{sub}</div></div>")


def row(inner, gap=12, opacity=1.0):
    return (f"<div style='display:flex;flex-wrap:wrap;gap:{gap}px;width:{DASH_W}px;margin:0 0 8px 0;"
            f"opacity:{opacity}'>{inner}</div>")


def fmt(v, d=2, w=7):
    return f"{v:+{w}.{d}f}"


def make_widget(cls, where, **kw):
    anchor = getattr(scene, f"{where}_anchor", None)
    if anchor is not None:
        try:
            return cls(pos=anchor, **kw)
        except Exception:
            pass
    return cls(**kw)


# ==========================================================================
# APP
# ==========================================================================
class App:
    def __init__(self, args):
        self.args = args
        self.rx = UdpReceiver(args.bind, args.port)
        self.link = LinkStats(args.hz)
        self.yaw = YawTracker()
        self.motion = MotionEstimator()
        self.drift = DriftEstimator()

        self.sample = None
        self.t_shift = None
        self.last_t_s = 0.0
        self.peak_speed = 0.0
        self.pkt_count = 0
        self.frozen = False
        self.last_ui = 0.0
        self.last_flush = time.time()
        self.shown_state = None
        self.drift_rate = None
        self.q_zero = None

        # ---- CSV LOGGING ----
        self.csv = None
        self.csv_name = "(disabled)"
        if not args.no_csv:
            self.csv_name = f"imu_telemetry_compass_{datetime.now().strftime('%H%M%S')}.csv"
            self.csv = open(self.csv_name, "w", encoding="utf-8")
            self.csv.write("Time_ms,W,X,Y,Z,Bounded_Yaw,Roll,Pitch,Yaw_Raw,Omega_dps,Drift_dpm,"
                           "Pose_dHeading,Still,TempC,Bias_x,Bias_y,Bias_z,FifoOvf\n")
        self.recording = self.csv is not None

        self._build_scene()
        self._build_widgets()
        self.cards_w.text = self._cards_html(None)
        self.status_w.text = self._status_html(time.perf_counter())

        scene.bind("keydown", self.on_key)
        atexit.register(self.close)
        print(f"[SYSTEM] Listening on UDP {args.bind}:{args.port}")
        print(f"[SYSTEM] CSV Data Logging: {self.csv_name}")

    # ------------------------------------------------------------------ scene
    def _build_scene(self):
        scene.title = (f"<div style='width:{DASH_W}px;margin:4px 0 6px 0;{MONO}'>"
                       f"<span style='font-size:22px;font-weight:700;color:{C_TXT};letter-spacing:1px'>"
                       f"EXPERIMENTAL <span style='color:{C_CYAN}'>TELEMETRY</span></span>"
                       f"<span style='color:{C_MUTED};font-size:13px;margin-left:12px'>"
                       f"Unitree G1 VR Teleoperation &middot; Gimbal-Lock-Free Visualizer</span></div>")
        scene.append_to_title("<style>html,body{background:#0b0e14 !important;color:#e6e9f2 !important}</style>")
        scene.width = DASH_W
        scene.height = 400
        scene.background = vector(0.055, 0.065, 0.09)
        scene.autoscale = False
        scene.range = 4.3
        scene.forward = vector(0, -0.3, -1)
        scene.center = vector(0, 0, 0)

        # ---------------- LEFT: digital twin ----------------
        self.twin_center = vector(-3.6, 0, 0)
        tc = self.twin_center
        arrow(pos=tc, axis=vector(2.5, 0, 0), color=color.red, shaftwidth=0.06)
        arrow(pos=tc, axis=vector(0, 2.5, 0), color=color.green, shaftwidth=0.06)
        arrow(pos=tc, axis=vector(0, 0, 2.5), color=color.blue, shaftwidth=0.06)
        for txt, off, col in (("X", vector(2.8, 0, 0), color.red), ("Y", vector(0, 2.8, 0), color.green),
                              ("Z", vector(0, 0, 2.8), color.blue)):
            text(text=txt, pos=tc + off, align="center", height=0.28, billboard=True, color=col)
        self.box = box(pos=tc, length=4, width=2, height=0.5, color=color.cyan, opacity=0.8)
        self.pointer = arrow(pos=tc, axis=vector(1.25, 0, 0), color=color.white, shaftwidth=0.05)
        self.up_arrow = arrow(pos=tc, axis=vector(0, 1.6, 0), color=color.yellow, shaftwidth=0.04)

        # ---------------- RIGHT: yaw compass ----------------
        self.cc = vector(3.6, 0, 0)
        self.R = 2.5
        cc, R = self.cc, self.R
        cylinder(pos=cc - vector(0, 0, 0.1), axis=vector(0, 0, 0.1), radius=R, color=color.gray(0.08))
        self.ring = ring(pos=cc, axis=vector(0, 0, 1), radius=R, thickness=0.05, color=color.white)
        for ang in range(0, 360, 15):
            rad = math.radians(ang)
            d = vector(math.sin(rad), math.cos(rad), 0)
            tp = cc + d * R
            if ang % 90 == 0:
                cylinder(pos=tp - d * 0.3, axis=d * 0.3, radius=0.04, color=color.cyan)
                lbl_angle = ang if ang <= 180 else ang - 360
                text(text=f"{lbl_angle}\u00b0", pos=cc + d * (R + 0.45), align="center", height=0.35,
                     billboard=True, color=color.white)
            elif ang % 45 == 0:
                cylinder(pos=tp - d * 0.2, axis=d * 0.2, radius=0.02, color=color.white)
            else:
                cylinder(pos=tp - d * 0.1, axis=d * 0.1, radius=0.01, color=color.gray(0.5))
        arrow(pos=cc, axis=vector(0, R * 0.95, 0), color=color.green, shaftwidth=0.06, opacity=0.4)
        self.needle = arrow(pos=cc, axis=vector(0, R * 0.9, 0), color=color.red, shaftwidth=0.08)

        self.yaw_readout = label(pos=cc, text="--", box=False, color=color.white, height=24,
                                 font="monospace", opacity=0)
        scene.append_to_caption("<br><hr>")

    # ---------------------------------------------------------------- widgets
    def _build_widgets(self):
        self.btn_zero = make_widget(button, "title", text="\u27f2  Zero yaw [Z]", bind=self.on_zero)
        self.btn_freeze = make_widget(button, "title", text="\u23f8  Freeze display [Space]", bind=self.on_freeze)
        self.btn_rec = make_widget(button, "title", text="\u25cf  Recording [R]", bind=self.on_rec)
        self._paint_rec_button()
        self.status_w = make_widget(wtext, "title", text="")
        self.cards_w = make_widget(wtext, "caption", text="")

    def _paint_rec_button(self):
        if self.csv is None:
            self.btn_rec.text = "\u25cf  CSV disabled"
            self.btn_rec.disabled = True
            return
        self.btn_rec.text = "\u25cf  Recording [R]" if self.recording else "\u25cb  Paused - not saving [R]"
        try:
            self.btn_rec.background = vector(0.75, 0.15, 0.15) if self.recording else color.gray(0.6)
        except Exception:
            pass

    # --------------------------------------------------------------- controls
    def on_zero(self, b=None):
        self.yaw.rezero()
        if self.sample:
            self.q_zero = self.sample["q"]
        print("[ACTION] Yaw reference mathematically zeroed.")

    def on_freeze(self, b=None):
        self.frozen = not self.frozen
        self.btn_freeze.text = ("\u25b6  Resume display [Space]" if self.frozen
                                else "\u23f8  Freeze display [Space]")

    def on_rec(self, b=None):
        if self.csv is None:
            return
        self.recording = not self.recording
        self._paint_rec_button()

    def on_key(self, evt):
        k = (evt.key or "").lower()
        if k == "z":
            self.on_zero()
        elif k in (" ", "space"):
            self.on_freeze()
        elif k == "r":
            self.on_rec()

    # ----------------------------------------------------------- packet logic
    def handle_packet(self, arrive, addr, data):
        try:
            pkt = parse_packet(data.decode("ascii", "ignore"))
        except ValueError:
            self.link.bad += 1
            return
        t_ms = pkt["t_ms"]
        q, qn = q_normalize(*pkt["wxyz"])
        if q is None:
            self.link.bad += 1
            return
        w, x, y, z = q

        if self.link.on_packet(arrive, t_ms, addr):
            self.t_shift = self.last_t_s - t_ms / 1000.0 + 1.0 / self.args.hz
            self.yaw.reset()
            self.motion.reset()
            self.drift.reset()
            self.q_zero = None
            print("[WARNING] ESP32 tracker restarted. Hardware filter reset, yaw re-zeroed.")

        if self.t_shift is None:
            self.t_shift = -t_ms / 1000.0
        t_s = t_ms / 1000.0 + self.t_shift
        self.last_t_s = t_s
        self.pkt_count += 1

        roll, pitch, yaw_raw = euler_deg(*q)

        # -----------------------------------------------------
        # Y-AXIS COMPASS TRACKING (Gimbal Lock Proof for Wrist Mount)
        # -----------------------------------------------------
        rx = 2.0 * (x * y - w * z)
        ry = 1.0 - 2.0 * (x ** 2 + z ** 2)
        y_axis_yaw = math.degrees(math.atan2(ry, rx))

        if self.yaw.update(y_axis_yaw):
            print(f"[SUCCESS] Visualizer initialized. Initial Y-Axis offset: {y_axis_yaw:.2f} deg")

        yaw_b, drift_total = self.yaw.bounded, self.yaw.drift_total
        if self.q_zero is None:
            self.q_zero = q
        pose_dh = pose_delta_heading_deg(q, self.q_zero)
        self.motion.push(t_s, q)
        speed = self.motion.speed
        self.peak_speed = max(self.peak_speed, speed)

        self.sample = dict(t=t_s, w=w, x=x, y=y, z=z, qn=qn, roll=roll, pitch=pitch, yaw_raw=yaw_raw,
                           yaw=yaw_b, drift=drift_total, speed=speed, q=q, pose_dh=pose_dh,
                           still=pkt["still"], temp=pkt["temp"], bias=pkt["bias"], ovf=pkt["ovf"])

        if self.recording and self.csv:
            dr = "" if self.drift_rate is None else f"{self.drift_rate:.3f}"
            node = ",,,,,"
            if pkt["bias"] is not None:
                bx, by, bz = pkt["bias"]
                node = f"{int(pkt['still'])},{pkt['temp']:.2f},{bx:.4f},{by:.4f},{bz:.4f},{pkt['ovf']}"
            self.csv.write(f"{t_ms:.2f},{w:.5f},{x:.5f},{y:.5f},{z:.5f},{yaw_b:.2f},"
                           f"{roll:.2f},{pitch:.2f},{yaw_raw:.2f},{speed:.3f},{dr},{pose_dh:.3f},{node}\n")

        if not self.frozen:
            self._update_twin(*q)
            self._update_compass(yaw_b)

    def _update_twin(self, w, x, y, z):
        fx = 1.0 - 2.0 * (y * y + z * z)
        fy = 2.0 * (x * y + w * z)
        fz = 2.0 * (x * z - w * y)
        ux = 2.0 * (x * z + w * y)
        uy = 2.0 * (y * z - w * x)
        uz = 1.0 - 2.0 * (x * x + y * y)
        fwd = vector(fx, -fy, fz)
        up = vector(ux, -uy, uz)
        self.box.axis = fwd
        self.box.up = up
        self.pointer.axis = fwd.rotate(angle=-pi / 2, axis=up) * 1.25
        self.pointer.up = up
        self.up_arrow.axis = up * 1.6

    def _update_compass(self, yaw_b):
        r = math.radians(yaw_b)
        sx, cy = math.sin(r) * self.R * 0.9, math.cos(r) * self.R * 0.9
        self.needle.axis = vector(sx, cy, 0)

    # --------------------------------------------------------------- UI tick
    def ui_tick(self, now):
        if self.sample:
            self.drift.add(self.last_t_s, self.yaw.unwrapped, self.peak_speed)
            self.drift_rate = self.drift.rate_dpm()
        moving = self.peak_speed > STILL_DPS
        self.peak_speed = 0.0

        self.status_w.text = self._status_html(now)
        state, col = self.link.state(now)
        if state != self.shown_state:
            self.shown_state = state
            self.ring.color = color.white if state == "LIVE" else (color.orange if state == "STALE" else color.red)
            self.box.opacity = 0.8 if state == "LIVE" else 0.25

        if self.frozen:
            return
        self.cards_w.text = self._cards_html(state, moving)
        if self.sample:
            self.yaw_readout.text = f"{self.sample['yaw']:+.2f}\u00b0"

    def _status_html(self, now):
        state, col = self.link.state(now)
        L = self.link
        bits = [pill(f"\u25cf {state}", col)]
        if L.last_rx is not None:
            bits += [pill(f"{L.rate_hz() if state != 'LOST' else 0.0:5.1f} Hz", C_TXT),
                     pill(f"jitter {L.jitter_ms():.1f} ms", C_TXT),
                     pill(f"drops {L.missed} ({L.loss_pct():.2f}%)", C_AMBER if L.missed else C_MUTED),
                     pill(f"src {L.src[0]}", C_MUTED)]
            if L.bad:
                bits.append(pill(f"bad pkts {L.bad}", C_RED))
        else:
            bits.append(f"<span style='color:{C_MUTED};font-size:12px'>listening on UDP {self.args.port} &middot; "
                        f"this PC ~ {lan_ip()}</span>")
        if self.csv is None:
            bits.append(pill("CSV off", C_MUTED))
        else:
            bits.append(
                pill("\u25cf REC" if self.recording else "\u25cb REC paused", C_RED if self.recording else C_MUTED))
        if self.frozen:
            bits.append(pill("FROZEN", C_VIOLET))
        return f"<div style='width:{DASH_W}px;margin:6px 0 12px 0'>{''.join(bits)}</div>"

    def _cards_html(self, state, moving=False):
        s = self.sample
        dim = 1.0 if state in (None, "LIVE") else 0.45
        if s is None:
            ph = "--"
            top = "".join([card("BOUNDED YAW (Y-AXIS)", ph, "\u00b0", accent=C_CYAN, grow=1),
                           card("ABSOLUTE DRIFT", ph, "\u00b0", accent=C_AMBER, grow=1),
                           card("SESSION TIME", ph, "s", accent=C_VIOLET, grow=1)])
            bottom = "".join([card("\u0394 HEADING vs ZERO POSE", ph, "\u00b0", accent=C_GREEN, grow=1),
                              card("NODE", ph, "", accent=C_MUTED, grow=2)])
            return row(top, opacity=dim) + row(bottom, opacity=dim)

        if self.drift_rate is not None:
            dr_sub = f"stable drift: {self.drift_rate:+.3f} \u00b0/min"
        elif moving:
            dr_sub = "sensor in motion..."
        else:
            dr_sub = f"measuring static phase... {self.drift.span:.0f}/{DRIFT_MIN_SPAN_S:.0f} s"

        top = "".join([
            card("BOUNDED YAW (Y-AXIS)", fmt(s["yaw"]), "\u00b0", "Gimbal-Lock-Free Heading", C_CYAN, 1),
            card("ABSOLUTE DRIFT", fmt(s["drift"]), "\u00b0", dr_sub, C_AMBER, 1),
            card("SESSION TIME", f"{int(self.last_t_s // 60):02d}:{self.last_t_s % 60:05.2f}", "s",
                 "Active Telemetry Runtime", C_VIOLET, 1),
        ])
        if s["bias"] is None:
            node_val, node_sub = "v1", "old firmware: no node diagnostics"
        else:
            bx, by, bz = s["bias"]
            node_val = (f"<span style='color:{C_GREEN if s['still'] else C_AMBER}'>"
                        f"{'STILL' if s['still'] else 'MOVING'}</span> {s['temp']:.1f}\u00b0C")
            node_sub = f"gyro bias {bx:+.3f} {by:+.3f} {bz:+.3f} \u00b0/s &middot; FIFO overflows {s['ovf']}"
        bottom = "".join([
            card("\u0394 HEADING vs ZERO POSE", fmt(s["pose_dh"]), "\u00b0",
                 "return to the zeroed pose: this is the drift", C_GREEN, 1),
            card("NODE", node_val, "", node_sub, C_MUTED, 2),
        ])
        return row(top, opacity=dim) + row(bottom, opacity=dim)

    # ------------------------------------------------------------------- run
    def pump(self):
        n = 0
        q = self.rx.q
        while q:
            arrive, addr, data = q.popleft()
            self.handle_packet(arrive, addr, data)
            n += 1
        now = time.perf_counter()
        if now - self.last_ui >= 1.0 / UI_HZ:
            self.last_ui = now
            self.ui_tick(now)
        if self.csv and time.time() - self.last_flush > 1.0:
            self.csv.flush()
            self.last_flush = time.time()
        return n

    def run(self):
        self.rx.start()
        try:
            while True:
                rate(FRAME_HZ)
                self.pump()
        except KeyboardInterrupt:
            print("\n[SYSTEM] Terminating visualizer...")
        finally:
            self.close()

    def close(self):
        self.rx.stop()
        if self.csv and not self.csv.closed:
            self.csv.flush()
            self.csv.close()


def main():
    ap = argparse.ArgumentParser(description="YawDrift Experimental Dashboard")
    ap.add_argument("--port", type=int, default=5005, help="UDP port")
    ap.add_argument("--bind", default="0.0.0.0", help="interface to bind")
    ap.add_argument("--hz", type=float, default=NOMINAL_HZ, help="nominal packet rate")
    ap.add_argument("--no-csv", action="store_true", help="do not write a CSV log")
    args = ap.parse_known_args()[0]
    try:
        app = App(args)
    except OSError as e:
        print(f"[FATAL] Could not open UDP {args.bind}:{args.port} -> {e}")
        sys.exit(1)
    app.run()


if __name__ == "__main__":
    main()