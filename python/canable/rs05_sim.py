"""rs05_sim.py - a fake RS05 on a python-can 'virtual' bus, for trying the GUI / tests without hardware.

    python rs05_gui.py --sim            (GUI starts this automatically)

Model: rotor + link inertia, Coulomb/viscous/static friction, mechanical hard stops at +-120 deg
(so you can see what the soft limits protect you from), and simple internal speed / position /
MIT / current loops that answer the private protocol like a real motor.
"""
from __future__ import annotations

import math
import os
import struct
import threading
import time

import can

import rs05_can as R

J = 0.002          # kg m^2
FC = 0.06          # Coulomb friction Nm
FS = 0.09          # static (breakaway) friction Nm
B = 0.012          # viscous Nm s/rad
KT = 0.30          # Nm/A
WALL = math.radians(120)


def enc(x, lo, hi, bits=16):
    """Like the motor firmware: float -> uint by truncation (int cast), not rounding."""
    n = (1 << bits) - 1
    return int((min(max(x, lo), hi) - lo) * n / (hi - lo))


class FakeRS05:
    def __init__(self, channel: str = "rs05sim", motor_id: int = 0x7F, host_id: int = 0xFD,
                 protocol: str = "private"):
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.id, self.host = motor_id, host_id
        self.protocol = protocol                 # "private" | "mit" | "canopen" (active after power cycle)
        self.pending_protocol = protocol
        self.uid = bytes([0x52, 0x53]) + os.urandom(6)       # unique per simulated motor
        self.pos, self.vel, self.tq = math.radians(35), 0.0, 0.0
        self.enabled, self.mode, self.report = False, 0, False
        self.p = {k: 0.0 for k in R.PARAMS}
        self.p.update(VBUS=48.0, limit_cur=2.0, vel_max=2.0, acc_set=5.0, limit_spd=2.0, acc_rad=10.0,
                      loc_kp=6.0, spd_kp=0.6, spd_ki=8.0, spd_filt_gain=0.1)   # sim loop gains (not RS05 defaults)
        self.mitcmd = (0.0, 0.0, 0.0, 0.0, 0.0)
        self._vr = 0.0
        self._integ = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._rx, daemon=True).start()
        threading.Thread(target=self._physics, daemon=True).start()
        threading.Thread(target=self._reporter, daemon=True).start()

    # ------------------------------------------------------------ physics
    def _speed_loop(self, v_target, dt):
        acc = self.p["acc_rad"] if self.mode == R.MODE_SPEED else self.p["acc_set"] or 10.0
        self._vr += max(-acc * dt, min(acc * dt, v_target - self._vr))
        err = self._vr - self.vel
        self._integ = max(-1.0, min(1.0, self._integ + self.p["spd_ki"] * err * dt))
        return self.p["spd_kp"] * err + self._integ + FC * math.copysign(1, self._vr) * (abs(self._vr) > 1e-3) + B * self._vr

    def _step(self, dt):
        p = self.p
        if not self.enabled:
            tq = 0.0
            self._vr, self._integ = self.vel, 0.0
        elif self.mode == R.MODE_SPEED:
            tq = self._speed_loop(p["spd_ref"], dt)
        elif self.mode in (R.MODE_PP, R.MODE_CSP):
            vmax = p["vel_max"] if self.mode == R.MODE_PP else p["limit_spd"]
            tq = self._speed_loop(max(-vmax, min(vmax, p["loc_kp"] * (p["loc_ref"] - self.pos))), dt)
        elif self.mode == R.MODE_MIT:
            pd, vd, kp, kd, tf = self.mitcmd
            tq = kp * (pd - self.pos) + kd * (vd - self.vel) + tf
        elif self.mode == R.MODE_CURRENT:
            tq = KT * p["iq_ref"]
        else:
            tq = 0.0
        if self.mode != R.MODE_MIT and self.enabled:
            lim = KT * p["limit_cur"] if p["limit_cur"] > 0 else R.T_MAX
            tq = max(-lim, min(lim, tq))
        tq = max(-R.T_MAX, min(R.T_MAX, tq))
        # friction + dynamics
        if abs(self.vel) < 1e-3 and abs(tq) <= FS:
            self.vel = 0.0
        else:
            fr = FC * math.copysign(1, self.vel if abs(self.vel) > 1e-3 else tq) + B * self.vel
            self.vel += (tq - fr) / J * dt
        self.pos += self.vel * dt
        if abs(self.pos) > WALL:                      # mechanical hard stop
            self.pos = math.copysign(WALL, self.pos)
            self.vel = 0.0
        self.tq = tq
        p["iqf"] = tq / KT
        p["VBUS"] = 48.0 - 0.08 * abs(tq / KT)      # supply / cable drop under load
        p["mechPos"] = self.pos
        p["mechVel"] = self.vel

    def _physics(self):
        t_last = time.perf_counter()
        while not self._stop.is_set():
            time.sleep(0.001)
            now = time.perf_counter()
            dt_total, t_last = min(now - t_last, 0.05), now
            n = max(1, int(dt_total / 0.0002))
            with self._lock:
                for _ in range(n):
                    self._step(dt_total / n)

    # ------------------------------------------------------------ CAN
    def _fb(self, typ=R.T_FEEDBACK):
        with self._lock:
            d = struct.pack(">HHHH", enc(self.pos, -R.P_MAX, R.P_MAX), enc(self.vel, -R.V_MAX, R.V_MAX),
                            enc(self.tq, -R.T_MAX, R.T_MAX), 312)
        arb = (typ << 24) | ((2 if self.enabled else 0) << 22) | (self.id << 8) | self.host
        self.bus.send(can.Message(arbitration_id=arb, is_extended_id=True, data=d))

    def _reporter(self):
        while not self._stop.is_set():
            time.sleep(0.01)
            if self.report:
                self._fb(R.T_ACTIVE_REPORT)

    # ------------------------------------------------------------ other protocols (MIT / CANopen)
    PROTO_CODES = {0: "private", 1: "canopen", 2: "mit"}

    def power_cycle(self):
        """Protocol changes (type 25 / MIT command 8 / CANopen switch) take effect here, like the real motor."""
        with self._lock:
            self.enabled, self.report = False, False
            self.protocol = self.pending_protocol
        if self.protocol == "canopen":
            self._std(0x700 + self.id, bytes([0x00]))          # CANopen boot-up

    def _std(self, arb, data):
        self.bus.send(can.Message(arbitration_id=arb, is_extended_id=False, data=data))

    def _mit_feedback(self):
        with self._lock:
            p = R.f2u(self.pos, -R.P_MAX, R.P_MAX)
            v = R.f2u(self.vel, -R.V_MAX, R.V_MAX, 12)
            t = R.f2u(self.tq, -R.T_MAX, R.T_MAX, 12)
        d = bytes([self.id, p >> 8, p & 0xFF, v >> 4, ((v & 0xF) << 4) | (t >> 8), t & 0xFF, 0x01, 0x38])
        self._std(self.host, d)                               # response command 1 on 11-bit ID = host

    def _rx_mit(self, m):
        d = bytes(m.data)
        if m.is_extended_id or m.arbitration_id != self.id or len(d) != 8 or d[:6] != b"\xff" * 6:
            return
        cmd, f = d[7], d[6]
        if cmd == 0xFD and f == 0xFF:          # stop
            self.enabled = False
            self._mit_feedback()
        elif cmd == 0xFD and f in self.PROTO_CODES:   # command 8: switch protocol
            self.pending_protocol = self.PROTO_CODES[f]
            self._std(self.id, self.uid)
        elif cmd == 0xFC and f == 0xFF:        # enable
            self.enabled = True
            self._mit_feedback()

    def _rx_canopen(self, m):
        d = bytes(m.data)
        if m.is_extended_id:
            if m.arbitration_id == 0xFFF and d[:6] == bytes([1, 2, 3, 4, 5, 6]) and d[6] in self.PROTO_CODES:
                self.pending_protocol = self.PROTO_CODES[d[6]]
                self._std(self.id, self.uid)
            return
        if m.arbitration_id == 0x600 + self.id and d[0] == 0x40:
            idx = d[1] | d[2] << 8
            if idx == 0x1000:
                self._std(0x580 + self.id, bytes([0x43]) + d[1:4] + struct.pack("<I", 0x00020192))
            else:
                self._std(0x580 + self.id, bytes([0x80]) + d[1:4] + struct.pack("<I", 0x06020000))

    def _rx(self):
        while not self._stop.is_set():
            m = self.bus.recv(0.05)
            if m is None:
                continue
            if self.protocol == "mit":
                self._rx_mit(m)
                continue
            if self.protocol == "canopen":
                self._rx_canopen(m)
                continue
            if not m.is_extended_id:
                continue
            arb, d = m.arbitration_id, bytes(m.data)
            typ, tgt = arb >> 24, arb & 0xFF
            if typ == R.T_SET_PROTOCOL:
                if tgt == self.id and d[:6] == bytes([1, 2, 3, 4, 5, 6]) and d[6] in self.PROTO_CODES:
                    self.pending_protocol = self.PROTO_CODES[d[6]]
                continue
            if typ == R.T_GET_ID:
                if tgt == self.id:
                    self.bus.send(can.Message(arbitration_id=(self.id << 8) | 0xFE, is_extended_id=True,
                                              data=self.uid))
                continue
            if tgt != self.id:
                continue
            with self._lock:
                if typ == R.T_MIT:
                    p, v, kp, kd = struct.unpack(">HHHH", d)
                    t = (arb >> 8) & 0xFFFF
                    self.mitcmd = (R.u2f(p, -R.P_MAX, R.P_MAX), R.u2f(v, -R.V_MAX, R.V_MAX),
                                   R.u2f(kp, 0, R.KP_MAX), R.u2f(kd, 0, R.KD_MAX), R.u2f(t, -R.T_MAX, R.T_MAX))
                elif typ == R.T_ENABLE:
                    self.enabled = True
                elif typ == R.T_STOP:
                    self.enabled = False
                elif typ == R.T_SET_ZERO:
                    self.pos = 0.0
                elif typ == R.T_SET_ID:
                    if not self.enabled:
                        self.id = (arb >> 16) & 0xFF
                        self.bus.send(can.Message(arbitration_id=(self.id << 8) | 0xFE, is_extended_id=True,
                                                  data=bytes(8)))
                    continue
                elif typ == R.T_ACTIVE_REPORT:
                    self.report = d[6] == 1
                elif typ == R.T_WRITE:
                    idx = d[0] | d[1] << 8
                    name, fmt = R.PARAM_BY_INDEX.get(idx, (None, "<f"))
                    if name == "run_mode":
                        if self.enabled:
                            continue          # real motor ignores run_mode while enabled
                        self.mode = d[4]
                        self.mitcmd = (self.pos, 0, 0, 0, 0)
                    if name:
                        self.p[name] = struct.unpack_from(fmt, d, 4)[0]
                elif typ == R.T_READ:
                    idx = d[0] | d[1] << 8
                    name, fmt = R.PARAM_BY_INDEX.get(idx, (None, "<f"))
                    if name:
                        v = self.p[name]
                        val = struct.pack(fmt, v if fmt == "<f" else int(v)).ljust(4, b"\0")
                        self.bus.send(can.Message(arbitration_id=(R.T_READ << 24) | (self.id << 8) | self.host,
                                                  is_extended_id=True, data=d[:4] + val))
                    continue
            if typ in (R.T_MIT, R.T_ENABLE, R.T_STOP, R.T_SET_ZERO, R.T_WRITE):
                self._fb()

    def close(self):
        self._stop.set()
        time.sleep(0.1)
        self.bus.shutdown()
