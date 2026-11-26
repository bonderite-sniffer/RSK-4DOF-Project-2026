#!/usr/bin/env python3
"""rs05_canable_test.py - bench tests for ONE RobStride RS05 straight from the PC via a CANable v2.

    PC --USB--> CANable v2 (slcan, COMx) --CAN 1 Mbit/s--> RS05 (ID 0x7F default)

Tests (each writes a CSV to logs/):
  scan        find motor CAN IDs on the bus  (--deep / --all-bitrates for a lost, undocumented ID)
  monitor     live position / speed / torque / current (iq) / VBUS / temp   (motor stays off)
  params      read the important parameters
  zero        current position = 0 (mechanical zero), optional --save to flash
  speed       speed mode (run_mode 2): list of speeds, reports tracking + current draw
  pos         position control, PP (run_mode 1) or CSP (run_mode 5): list of targets
  mit         MIT / operation control (type 1, run_mode 0): position steps with kp/kd, or --spring
  current     current draw: idle iq/VBUS, optional current-mode (run_mode 3) iq steps -> Kt
  stop        stop motor (--clear clears faults)

Examples (PowerShell, from python/canable):
  python rs05_canable_test.py scan
  python rs05_canable_test.py monitor
  python rs05_canable_test.py zero
  python rs05_canable_test.py speed --vel 2 5 -5 --hold 3
  python rs05_canable_test.py pos --mode pp --deg 90 -90 0
  python rs05_canable_test.py pos --mode csp --deg 45 0 --vmax 2
  python rs05_canable_test.py mit --deg 30 -30 0 --kp 10 --kd 0.5
  python rs05_canable_test.py mit --spring --kp 5 --kd 0.2 --time 15
  python rs05_canable_test.py current --iq 0.3 0.6 --hold 1     (shaft LOCKED / loaded!)
  python rs05_canable_test.py                                   (interactive menu)

Common options: -p COM9  --id 0x7F  --host 0xFD  --iface slcan|gs_usb|stm  --bitrate 1000000
(--iface stm: the STM32 running Firmware_Gateway is the CAN interface, both channels, 1 Mbit/s only)
Ctrl+C at any time stops the motor. This is bench software, not a safety system:
keep a hand on the PSU switch.
"""
from __future__ import annotations

import argparse
import csv
import math
import shlex
import statistics as st
import sys
import time
from pathlib import Path

import can

from rs05_can import (I_MAX, KD_MAX, KP_MAX, MODE_CSP, MODE_CURRENT, MODE_MIT, MODE_PP,
                      MODE_SPEED, P_MAX, PARAMS, RS05, RUN_MODE_NAMES, STATE_NAMES, T_MAX,
                      V_MAX, deep_scan_bitrates, fault_str, open_bus)

RPM = 60.0 / (2 * math.pi)
LOG_DIR = Path(__file__).resolve().parent / "logs"


# ====================================================================== session
class Session:
    """Opens the CANable, starts feedback + iq/VBUS polling and logs every feedback frame."""

    def __init__(self, a):
        self.a = a
        self.bus = open_bus(a.iface, a.port, a.bitrate)
        self.m = RS05(self.bus, a.id, a.host)
        self.tag = ""
        self.setpoint = float("nan")
        self._csv = None
        self._w = None
        self._t0 = time.monotonic()
        self.rows: list[tuple] = []          # (t, setpoint, pos, vel, torque, iq) for summaries
        self.m.on_feedback = self._on_fb

    # --------------------------------------------------------------- logging
    def _on_fb(self, fb):
        row = (fb.t - self._t0, self.setpoint, fb.pos, fb.vel, fb.torque, self.m.s.iq)
        self.rows.append(row)
        if len(self.rows) > 200_000:
            del self.rows[:100_000]
        if self._w:
            self._w.writerow([f"{row[0]:.4f}", self.tag, f"{self.setpoint:.5f}", f"{fb.pos:.5f}",
                              f"{fb.vel:.4f}", f"{fb.torque:.4f}", f"{self.m.s.iq:.4f}",
                              f"{self.m.s.vbus:.2f}", f"{fb.temp:.1f}", fb.state, fb.faults])

    def start_log(self, name: str) -> Path:
        LOG_DIR.mkdir(exist_ok=True)
        path = LOG_DIR / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self._csv = open(path, "w", newline="")
        self._w = csv.writer(self._csv)
        self._w.writerow(["t_s", "tag", "setpoint", "pos_rad", "vel_rad_s", "torque_Nm", "iq_A",
                          "vbus_V", "temp_C", "state", "faults"])
        return path

    def stop_log(self):
        if self._csv:
            self._w = None
            self._csv.close()
            self._csv = None

    def window(self, seconds: float):
        """Feedback rows of the last `seconds`."""
        t_min = time.monotonic() - self._t0 - seconds
        return [r for r in self.rows if r[0] >= t_min]

    # --------------------------------------------------------------- lifecycle
    def start(self):
        m = self.m
        m.stop()                  # safe state + first feedback
        m.active_report(True)     # periodic type-24 feedback even while idle
        m.start_polling(self.a.poll_hz)
        fb = m.wait_feedback(1.0)
        if fb is None:
            m.stop()
            fb = m.wait_feedback(0.5)
        if fb is None:
            raise SystemExit(
                f"No feedback from motor 0x{m.id:02X}. Check: 48 V on, CAN H/L, 120 ohm termination, "
                f"bitrate {self.a.bitrate}, motor ID (run 'scan'), CANable on {self.a.port}.")
        t_end = time.monotonic() + 0.5          # first iqf / VBUS reply
        while math.isnan(m.s.vbus) and time.monotonic() < t_end:
            time.sleep(0.02)
        if fb.faults:
            print(f"!! motor reports faults: {fault_str(fb.faults)}  (run 'stop --clear')")
        return fb

    def safe_stop(self):
        try:
            self.m.stop()
            time.sleep(0.02)
            self.m.stop()
        except can.CanError as e:
            print("stop frame failed:", e)

    def close(self):
        self.safe_stop()
        try:
            self.m.active_report(False)
        except can.CanError:
            pass
        self.stop_log()
        self.m.close()
        self.bus.shutdown()

    # --------------------------------------------------------------- helpers
    def fb(self):
        return self.m.s.fb

    def check_alive(self, max_age: float = 0.3):
        fb = self.fb()
        age = time.monotonic() - fb.t if fb else 1e9
        if age > max_age:
            raise RuntimeError(f"feedback lost ({age:.2f}s) - stopping")
        if fb.faults:
            raise RuntimeError(f"motor fault: {fault_str(fb.faults)}")
        if fb.temp > self.a.temp_max:
            raise RuntimeError(f"over temperature {fb.temp:.1f} C")

    def status_line(self) -> str:
        f, s = self.fb(), self.m.s
        if f is None:
            return "no feedback"
        return (f"pos {math.degrees(f.pos):+8.2f} deg | vel {f.vel:+7.3f} rad/s ({f.vel * RPM:+6.1f} rpm) | "
                f"T {f.torque:+6.3f} Nm | iq {s.iq:+6.3f} A | VBUS {s.vbus:5.1f} V | {f.temp:4.1f} C | "
                f"{STATE_NAMES.get(f.state, '?')} | flt {fault_str(f.faults)} | {s.fb_hz:5.0f} Hz")


def summary(rows, idx):
    vals = [r[idx] for r in rows if not math.isnan(r[idx])]
    if not vals:
        return float("nan"), float("nan"), float("nan")
    return st.fmean(vals), (st.pstdev(vals) if len(vals) > 1 else 0.0), max(abs(v) for v in vals)


def confirm(a, text: str) -> bool:
    if a.yes:
        return True
    return input(f"{text} [y/N] ").strip().lower() == "y"


def sleep_live(S: Session, seconds: float, check: bool = True):
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        if check:
            S.check_alive()
        print("  " + S.status_line(), end="\r")
        time.sleep(0.05)
    print()


# ====================================================================== tests
def cmd_scan(S: Session, a):
    m = S.m
    m.seen_ids.clear()
    for cid in range(1, 128):
        m.get_id(cid)
        time.sleep(0.002)
    time.sleep(0.2)
    ids = sorted(m.seen_ids)
    print("motor CAN IDs answering:", ", ".join(f"0x{i:02X} ({i})" for i in ids) or "none")
    if ids and m.id not in ids:
        print(f"note: --id 0x{m.id:02X} is not among them; use --id 0x{ids[0]:02X}")


def cmd_deep_scan(a):
    """Lost / undocumented motor ID. Runs without a Session: it owns the bus and may reopen it."""
    bitrates = (1_000_000, 500_000, 250_000, 125_000) if a.all_bitrates else (a.bitrate,)
    print(f"deep scan: IDs 0..255, bitrates {', '.join(str(b) for b in bitrates)}"
          + ("" if a.no_stop else "  (stop probe ON: any running motor is disabled)"))
    if not a.no_stop and not confirm(a, "Nothing on the bus should be moving. Continue?"):
        return

    def prog(f, br):
        print(f"  {br:>7} bit/s  {100 * f:5.1f} %", end="\r")

    results = deep_scan_bitrates(a.iface, a.port, a.host, bitrates, progress=prog, stop_probe=not a.no_stop)
    print()
    found = []
    for r in results:
        print(r.text())
        found += [(r.bitrate, cid) for cid in r.motors]
    if not found:
        print("\nNo RS05 answered. Check 48 V, CAN H/L, termination; try --all-bitrates. If 'other traffic'\n"
              "with 11-bit IDs shows up, the motor was switched to MIT/CANopen protocol: switch it back to\n"
              "the private protocol with the RobStride PC software.")
        return
    br, cid = found[0]
    print(f"\nNext: python rs05_canable_test.py --bitrate {br} --id 0x{cid:02X} monitor")
    print(f"      python rs05_canable_test.py --bitrate {br} --id 0x{cid:02X} setid --new <1..127>")
    if any(b != 1_000_000 for b, _ in found):
        print("note: a motor answered at a bitrate other than 1 Mbit/s - its baud rate was changed (type 23).")


def cmd_monitor(S: Session, a):
    path = S.start_log("monitor") if a.log else None
    print("Motor is NOT driven - turn the shaft by hand. Ctrl+C to quit.")
    try:
        while True:
            fb = S.fb()
            if fb is None or time.monotonic() - fb.t > 0.3:
                S.m.stop()      # poke: any frame makes the motor answer with feedback
            print("  " + S.status_line(), end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        if path:
            print("log:", path)


def cmd_params(S: Session, a):
    for name, (idx, _) in PARAMS.items():
        try:
            v = S.m.read(name)
            extra = f"  ({RUN_MODE_NAMES.get(v, '?')})" if name == "run_mode" else ""
            print(f"  0x{idx:04X} {name:14s} = {v:.5g}{extra}" if isinstance(v, float)
                  else f"  0x{idx:04X} {name:14s} = {v}{extra}")
        except TimeoutError:
            print(f"  0x{idx:04X} {name:14s} = (no reply)")


def cmd_zero(S: Session, a):
    m = S.m
    m.stop()
    print("Motor stopped. Turn the shaft to where 0 should be.")
    print(f"  now: {S.status_line()}")
    if not confirm(a, "Set THIS position as mechanical zero?"):
        return
    before = m.read("mechPos")
    m.set_zero()
    time.sleep(0.2)
    m.stop()
    m.wait_feedback(0.5)
    after = m.read("mechPos")
    fb_pos = S.fb().pos
    ok = abs(after) < 0.02 and abs(fb_pos) < 0.02
    print(f"  mechPos {before:+.4f} -> {after:+.4f} rad, feedback pos {math.degrees(fb_pos):+.2f} deg  "
          f"=> {'ZERO OK' if ok else 'ZERO FAILED'}")
    if a.save:
        m.save_params()
        time.sleep(0.2)
        print("  save-to-flash (type 22) sent. Power-cycle and run 'monitor' to check the zero is kept.")
    else:
        print("  (zero is checked again after power-cycle; add --save if it is lost)")


def cmd_setid(S: Session, a):
    m, old = S.m, S.m.id
    if m.ping(a.new):
        raise SystemExit(f"ID 0x{a.new:02X} is already used on this bus")
    print(f"Change motor 0x{old:02X} -> 0x{a.new:02X}. Only ONE motor with ID {old} may be on the bus.")
    if not confirm(a, "Continue?"):
        return
    ok = m.set_can_id(a.new)
    print(f"  {'OK: motor answers on' if ok else 'FAILED: no answer on'} 0x{a.new:02X}."
          + (f" Use --id 0x{a.new:02X} from now on; power-cycle + scan to confirm." if ok else ""))


def cmd_stop(S: Session, a):
    S.m.stop(clear_fault=a.clear)
    S.m.wait_feedback(0.3)
    print("stopped" + (" + faults cleared" if a.clear else ""), "|", S.status_line())


def cmd_speed(S: Session, a):
    m = S.m
    for v in a.vel:
        if abs(v) > a.max_vel:
            raise SystemExit(f"{v} rad/s > --max-vel {a.max_vel} (raise --max-vel deliberately)")
    print(f"SPEED MODE: {a.vel} rad/s, hold {a.hold}s, I-limit {a.cur} A, acc {a.acc} rad/s^2")
    if not confirm(a, "Shaft is free to rotate continuously?"):
        return
    path = S.start_log("speed")
    results = []
    try:
        m.set_mode(MODE_SPEED)
        m.write("limit_cur", a.cur)
        m.write("acc_rad", a.acc)
        for v in a.vel:
            S.tag, S.setpoint = f"vel={v}", v
            m.write("spd_ref", v)
            print(f"-> {v:+.2f} rad/s ({v * RPM:+.1f} rpm)")
            sleep_live(S, a.hold)
            rows = S.window(a.hold / 2)          # steady-state half
            vm, vsd, _ = summary(rows, 3)
            tm, _, tmax = summary(rows, 4)
            im, _, imax = summary(rows, 5)
            results.append((v, vm, vsd, tm, tmax, im, imax))
        S.tag, S.setpoint = "decel", 0.0
        m.write("spd_ref", 0.0)
        sleep_live(S, max(abs(x) for x in a.vel) / a.acc + 0.5)
    finally:
        S.safe_stop()
        S.stop_log()
    print("\n  cmd rad/s | meas rad/s (sd)   | err %  | torque Nm (max)  | iq A (max)")
    for v, vm, vsd, tm, tmax, im, imax in results:
        err = 100 * (vm - v) / v if v else float("nan")
        print(f"  {v:+9.2f} | {vm:+8.3f} ({vsd:.3f}) | {err:+6.1f} | {tm:+7.3f} ({tmax:.3f}) | "
              f"{im:+6.3f} ({imax:.3f})")
    print("log:", path)


def cmd_pos(S: Session, a):
    m = S.m
    start = m.read("mechPos")
    targets = []
    ref = start
    for d in a.deg:
        tgt = (ref + math.radians(d)) if a.rel else math.radians(d)
        if abs(tgt) > P_MAX - 0.1:
            raise SystemExit(f"target {math.degrees(tgt):.1f} deg outside +-720 deg feedback range")
        targets.append(tgt)
        ref = tgt
    steps = [abs(targets[0] - start)] + [abs(b - x) for x, b in zip(targets, targets[1:])]
    if max(steps) > math.radians(a.max_step) and not a.force:
        raise SystemExit(f"a step is {math.degrees(max(steps)):.0f} deg > --max-step {a.max_step} deg "
                         f"(now at {math.degrees(start):+.1f} deg; run 'zero' first or add --force)")
    mode = MODE_PP if a.mode == "pp" else MODE_CSP
    print(f"POSITION {a.mode.upper()}: from {math.degrees(start):+.1f} deg to "
          f"{[round(math.degrees(t), 1) for t in targets]} deg, vmax {a.vmax} rad/s, I-limit {a.cur} A")
    if not confirm(a, "Swing area clear?"):
        return
    path = S.start_log(f"pos_{a.mode}")
    results = []
    try:
        m.set_mode(mode)
        m.write("limit_cur", a.cur)
        if mode == MODE_PP:
            m.write("vel_max", a.vmax)
            m.write("acc_set", a.acc)
        else:
            m.write("limit_spd", a.vmax)
        for tgt in targets:
            S.tag, S.setpoint = f"pos={math.degrees(tgt):.1f}", tgt
            m.write("loc_ref", tgt)
            t0 = time.monotonic()
            settled_since, t_reach = None, None
            timeout = abs(tgt - S.fb().pos) / a.vmax * 2 + a.timeout
            while time.monotonic() - t0 < timeout:
                S.check_alive()
                fb = S.fb()
                inside = abs(fb.pos - tgt) < math.radians(a.tol) and abs(fb.vel) < 0.1
                if inside:
                    settled_since = settled_since or time.monotonic()
                    if time.monotonic() - settled_since > 0.2:
                        t_reach = settled_since - t0
                        break
                else:
                    settled_since = None
                print("  " + S.status_line(), end="\r")
                time.sleep(0.01)
            print()
            sleep_live(S, a.dwell)
            rows = S.window(a.dwell)
            pm, _, _ = summary(rows, 2)
            tm, _, tmax = summary(rows, 4)
            im, _, imax = summary(rows, 5)
            peak_i = summary(S.window(time.monotonic() - t0), 5)[2]
            results.append((tgt, t_reach, pm - tgt, tm, im, peak_i))
    finally:
        S.safe_stop()
        S.stop_log()
    print("\n  target deg | reached in | ss error deg | hold torque Nm | hold iq A | peak iq A")
    for tgt, tr, err, tm, im, pk in results:
        print(f"  {math.degrees(tgt):+10.1f} | {('%.2f s' % tr) if tr is not None else 'NOT REACHED':>10s} | "
              f"{math.degrees(err):+12.3f} | {tm:+14.3f} | {im:+9.3f} | {pk:9.3f}")
    print("log:", path)


def cmd_mit(S: Session, a):
    m = S.m
    kp, kd = min(a.kp, a.max_kp), min(a.kd, KD_MAX)
    tff = max(-a.max_tff, min(a.max_tff, a.tff))
    if (kp, kd, tff) != (a.kp, a.kd, a.tff):
        print(f"(clamped to kp {kp}, kd {kd}, tff {tff}; see --max-kp / --max-tff)")
    start = S.fb().pos
    if a.spring:
        targets = [start]
        hold = a.time
        print(f"MIT SPRING: hold {math.degrees(start):+.1f} deg, kp {kp}, kd {kd}, tff {tff} Nm for {hold}s. "
              f"Push the shaft by hand.")
    else:
        targets = [(start + math.radians(d)) if a.rel else math.radians(d) for d in a.deg]
        hold = a.hold
        if max(abs(t) for t in targets) > P_MAX - 0.1:
            raise SystemExit("MIT target outside +-720 deg")
        print(f"MIT STEPS: {[round(math.degrees(t), 1) for t in targets]} deg, kp {kp}, kd {kd}, "
              f"ramp {a.vmax} rad/s, tff {tff} Nm, {a.rate:.0f} Hz")
    if not confirm(a, "Swing area clear?"):
        return
    path = S.start_log("mit")
    period = 1.0 / a.rate
    results = []
    try:
        m.set_mode(MODE_MIT)
        p_des = start
        nxt = time.monotonic()
        for tgt in targets:
            S.tag = f"mit={math.degrees(tgt):.1f}"
            t0 = time.monotonic()
            t_hold_start = None
            while True:
                now = time.monotonic()
                # ramp p_des toward the target at vmax -> no step torque spike
                dp = tgt - p_des
                step = a.vmax * period
                if abs(dp) > step:
                    p_des += math.copysign(step, dp)
                    v_des = math.copysign(a.vmax, dp)
                else:
                    p_des, v_des = tgt, 0.0
                    t_hold_start = t_hold_start or now
                S.setpoint = p_des
                m.mit(p_des, v_des, kp, kd, tff)
                if t_hold_start and now - t_hold_start >= hold:
                    break
                if now - t0 > 1.0:
                    S.check_alive()
                print("  " + S.status_line(), end="\r")
                nxt += period
                time.sleep(max(0.0, nxt - time.monotonic()))
            print()
            rows = S.window(min(hold, 1.0))
            pm, _, _ = summary(rows, 2)
            tm, _, tmax = summary(rows, 4)
            im, _, imax = summary(rows, 5)
            results.append((tgt, pm - tgt, tm, tmax, im, imax))
        # release softly: zero gains for a moment, then stop
        for _ in range(int(0.1 * a.rate)):
            m.mit(p_des, 0.0, 0.0, 0.0, 0.0)
            time.sleep(period)
    finally:
        S.safe_stop()
        S.stop_log()
    print("\n  target deg | ss error deg | torque Nm (max) | iq A (max)")
    for tgt, err, tm, tmax, im, imax in results:
        print(f"  {math.degrees(tgt):+10.1f} | {math.degrees(err):+12.3f} | {tm:+7.3f} ({tmax:.3f}) | "
              f"{im:+6.3f} ({imax:.3f})")
    if not a.spring and kp > 0:
        print(f"  (expected ss error ~ friction / kp; kp {kp} Nm/rad -> 0.05 Nm gives "
              f"{math.degrees(0.05 / kp):.2f} deg)")
    print("log:", path)


def cmd_current(S: Session, a):
    m = S.m
    for i in a.iq:
        if abs(i) > a.max_iq:
            raise SystemExit(f"{i} A > --max-iq {a.max_iq}")
    m.stop()
    print("Idle current draw (motor stopped), 2 s ...")
    sleep_live(S, 2.0, check=False)
    rows = S.window(1.5)
    im, isd, _ = summary(rows, 5)
    print(f"  idle iq {im:+.3f} A (sd {isd:.3f}), VBUS {m.s.vbus:.2f} V")
    if not a.iq:
        print("  (current is also reported in 'speed', 'pos' and 'mit' summaries)")
        return
    print(f"CURRENT MODE: iq_ref {a.iq} A, {a.hold}s each. An unloaded shaft ACCELERATES - lock the shaft "
          f"or load it. Speed guard {a.vel_guard} rad/s.")
    if not confirm(a, "Shaft locked / loaded?"):
        return
    path = S.start_log("current")
    results = []
    try:
        m.set_mode(MODE_CURRENT)
        for i in a.iq:
            S.tag, S.setpoint = f"iq={i}", i
            m.write("iq_ref", i)
            t_end = time.monotonic() + a.hold
            while time.monotonic() < t_end:
                S.check_alive()
                if abs(S.fb().vel) > a.vel_guard:
                    m.write("iq_ref", 0.0)
                    raise RuntimeError(f"speed guard: {S.fb().vel:+.2f} rad/s > {a.vel_guard}")
                print("  " + S.status_line(), end="\r")
                time.sleep(0.02)
            print()
            rows = S.window(a.hold / 2)
            im, _, _ = summary(rows, 5)
            tm, _, _ = summary(rows, 4)
            vm, _, _ = summary(rows, 3)
            results.append((i, im, tm, vm))
            m.write("iq_ref", 0.0)
            time.sleep(a.rest)
    finally:
        try:
            m.write("iq_ref", 0.0)
        except can.CanError:
            pass
        S.safe_stop()
        S.stop_log()
    print("\n  iq_ref A | iqf meas A | torque Nm | vel rad/s | Kt Nm/A")
    for i, im, tm, vm in results:
        kt = tm / im if im and abs(im) > 0.05 else float("nan")
        print(f"  {i:+8.2f} | {im:+10.3f} | {tm:+9.3f} | {vm:+9.3f} | {kt:7.3f}")
    print("log:", path)


# ====================================================================== CLI
def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="COM9", help="CANable COM port (slcan) or device index (gs_usb)")
    ap.add_argument("--iface", default="slcan", help="slcan (default CANable firmware) | gs_usb (candleLight) | stm (STM32 gateway: -p = STM COM port)")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--id", type=lambda x: int(x, 0), default=0x7F, help="motor CAN ID (default 0x7F)")
    ap.add_argument("--host", type=lambda x: int(x, 0), default=0xFD, help="host ID (default 0xFD)")
    ap.add_argument("--poll-hz", type=float, default=20.0, help="iqf/VBUS read rate")
    ap.add_argument("--temp-max", type=float, default=70.0)
    ap.add_argument("-y", "--yes", action="store_true", help="skip confirmations")
    sub = ap.add_subparsers(dest="cmd")

    s = sub.add_parser("scan")
    s.add_argument("--deep", action="store_true",
                   help="find a lost ID: IDs 0..255, get-ID + stop probe, passive listen, decode every frame")
    s.add_argument("--all-bitrates", action="store_true", help="deep scan at 1M, 500k, 250k and 125k")
    s.add_argument("--no-stop", action="store_true", help="deep scan without the stop (type 4) probe")
    s = sub.add_parser("monitor"); s.add_argument("--log", action="store_true")
    sub.add_parser("params")
    s = sub.add_parser("zero"); s.add_argument("--save", action="store_true", help="also save to flash (type 22)")
    s = sub.add_parser("stop"); s.add_argument("--clear", action="store_true", help="clear faults")
    s = sub.add_parser("setid"); s.add_argument("--new", type=lambda x: int(x, 0), required=True,
                                                help="new CAN ID 1..127")

    s = sub.add_parser("speed")
    s.add_argument("--vel", type=float, nargs="+", default=[2.0, 5.0, -5.0], help="rad/s")
    s.add_argument("--hold", type=float, default=3.0)
    s.add_argument("--cur", type=float, default=2.0, help="limit_cur A")
    s.add_argument("--acc", type=float, default=10.0, help="acc_rad rad/s^2")
    s.add_argument("--max-vel", type=float, default=20.0)

    s = sub.add_parser("pos")
    s.add_argument("--mode", choices=("pp", "csp"), default="pp")
    s.add_argument("--deg", type=float, nargs="+", default=[90.0, -90.0, 0.0])
    s.add_argument("--rel", action="store_true", help="targets are relative steps")
    s.add_argument("--vmax", type=float, default=3.0, help="rad/s (PP vel_max / CSP limit_spd)")
    s.add_argument("--acc", type=float, default=10.0, help="PP acc_set rad/s^2")
    s.add_argument("--cur", type=float, default=2.0, help="limit_cur A")
    s.add_argument("--tol", type=float, default=1.0, help="deg")
    s.add_argument("--dwell", type=float, default=1.5, help="s held at each target")
    s.add_argument("--timeout", type=float, default=5.0)
    s.add_argument("--max-step", type=float, default=360.0, help="deg")
    s.add_argument("--force", action="store_true")

    s = sub.add_parser("mit")
    s.add_argument("--deg", type=float, nargs="+", default=[30.0, -30.0, 0.0])
    s.add_argument("--rel", action="store_true")
    s.add_argument("--spring", action="store_true", help="hold current position (feel kp/kd by hand)")
    s.add_argument("--kp", type=float, default=10.0, help="Nm/rad (0..500)")
    s.add_argument("--kd", type=float, default=0.5, help="Nm.s/rad (0..5)")
    s.add_argument("--tff", type=float, default=0.0, help="feed-forward torque Nm")
    s.add_argument("--vmax", type=float, default=2.0, help="target ramp rad/s")
    s.add_argument("--hold", type=float, default=2.0)
    s.add_argument("--time", type=float, default=10.0, help="--spring duration")
    s.add_argument("--rate", type=float, default=100.0, help="command Hz")
    s.add_argument("--max-kp", type=float, default=50.0)
    s.add_argument("--max-tff", type=float, default=1.0)

    s = sub.add_parser("current")
    s.add_argument("--iq", type=float, nargs="*", default=[], help="current-mode steps A (empty = idle only)")
    s.add_argument("--hold", type=float, default=1.0)
    s.add_argument("--rest", type=float, default=0.5)
    s.add_argument("--max-iq", type=float, default=2.0)
    s.add_argument("--vel-guard", type=float, default=5.0, help="abort above this rad/s")
    return ap


COMMANDS = {"scan": cmd_scan, "monitor": cmd_monitor, "params": cmd_params, "zero": cmd_zero,
            "stop": cmd_stop, "setid": cmd_setid, "speed": cmd_speed, "pos": cmd_pos, "mit": cmd_mit, "current": cmd_current}

MENU = """
================ RS05 / CANable test ================
 1 monitor        6 position CSP  (pos --mode csp ...)
 2 params         7 MIT steps     (mit ...)
 3 zero           8 MIT spring    (mit --spring ...)
 4 speed          9 current draw  (current ...)
 5 position PP    s stop + clear faults     q quit
 or type any command line, e.g.:  speed --vel 3 -3 --hold 2
"""
MENU_KEYS = {"1": "monitor", "2": "params", "3": "zero", "4": "speed", "5": "pos --mode pp",
             "6": "pos --mode csp", "7": "mit", "8": "mit --spring", "9": "current", "s": "stop --clear"}


def run(S: Session, a):
    try:
        COMMANDS[a.cmd](S, a)
    except KeyboardInterrupt:
        print("\nCtrl+C -> stop")
    except (RuntimeError, TimeoutError, can.CanError) as e:
        print(f"\n!! {e}")
    finally:
        S.safe_stop()
        S.stop_log()


def main():
    ap = build_parser()
    a = ap.parse_args()
    print(f"CANable {a.iface}:{a.port} @ {a.bitrate} | motor 0x{a.id:02X} | host 0x{a.host:02X}")
    if a.cmd == "scan" and (a.deep or a.all_bitrates):
        cmd_deep_scan(a)
        return
    S = Session(a)
    try:
        if a.cmd == "scan":
            cmd_scan(S, a)
            return
        S.start()
        print("  " + S.status_line())
        if a.cmd:
            run(S, a)
            return
        while True:                       # interactive menu
            print(MENU)
            line = input("> ").strip()
            if line.lower() in ("q", "quit", "exit"):
                break
            line = MENU_KEYS.get(line, line)
            if not line:
                continue
            try:
                sub = ap.parse_args(shlex.split(line))
            except SystemExit:
                continue
            for k in ("port", "iface", "bitrate", "id", "host", "poll_hz"):
                setattr(sub, k, getattr(a, k))   # connection stays as opened
            if sub.cmd:
                try:
                    run(S, sub)
                except SystemExit as e:       # validation messages from the tests
                    print(f"!! {e}")
    except KeyboardInterrupt:
        print()
    finally:
        S.close()
        print("motor stopped, CAN closed.")


if __name__ == "__main__":
    main()
