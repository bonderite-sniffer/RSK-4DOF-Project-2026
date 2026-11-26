#!/usr/bin/env python3
"""rs05_multi.py - run up to 4 RS05 with different CAN IDs on ONE CANable bus at the same time.

Purpose
  * prove the ID setup: every motor answers on its own ID only, no duplicates, no leftover 0x7F,
    and you can see which physical joint each ID is ("identify" wiggle)
  * stress the wiring: all motors move together with their own position / velocity / MIT sequence,
    at the current limit you choose, while VBUS sag, current, torque, temperature, faults and
    CAN feedback gaps are recorded per motor

Each motor has a plan (MotorPlan):
  mode      pos-pp | pos-csp | vel | mit
  steps     "value[:seconds], ..."   deg for pos / mit, rad/s for vel      e.g. "0:2, 45:3, -45:3"
  dwell     seconds for steps without ":seconds"
  cycles    repeat the step list N times (0 = until the test time limit)
  delay     start delay, s (stagger motors, or 0 for all at once = max current)
  cur       current limit A (pos / vel), vmax rad/s, acc rad/s^2, kp / kd / ramp for MIT
  rel       steps relative to the position at start
  end       "hold" (keep holding the last step until the test ends) or "stop"
All the single-motor safety (joint limits, virtual wall, feedback loss, faults, temperature) stays
active per motor (rs05_controller.Controller). On top of that the test stops everything when the time
limit is reached, VBUS drops below a threshold, or (optional) any motor trips.

GUI: rs05_multi_gui.py.  Headless:
  python rs05_multi.py -p COM9 --check-ids --ids 1 2 3 4
  python rs05_multi.py -p COM9 --plan rs05_multi_plan.json --duration 60 -y
  python rs05_multi.py --sim --plan rs05_multi_plan.json --duration 20 -y
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

import can

from rs05_can import KP_MAX, RS05, T_GET_ID, BusHub, fault_str, open_bus
from rs05_controller import Controller, SafetyTrip, load_config, save_config

HERE = Path(__file__).resolve().parent
PLAN_PATH = HERE / "rs05_multi_plan.json"
LOG_DIR = HERE / "logs"
MODES = ("pos-pp", "pos-csp", "vel", "mit")
TICK_HZ = 50.0
GAP_WARN_S = 0.05            # feedback gap counted as a dropout


# ====================================================================== plan
@dataclass
class MotorPlan:
    enabled: bool = True
    name: str = "J1"
    motor_id: int = 1
    mode: str = "pos-csp"
    steps: str = "0:2, 20:2, -20:2"
    dwell: float = 2.0
    cycles: int = 0
    delay: float = 0.0
    cur: float = 2.0
    vmax: float = 3.0
    acc: float = 10.0
    kp: float = 10.0
    kd: float = 0.5
    ramp: float = 2.0
    rel: bool = False
    end: str = "hold"
    model: str = "RS05"          # "RS05" or "RS02" (MIT / feedback scaling)

    def parse_steps(self) -> list[tuple[float, float]]:
        out = []
        for tok in self.steps.replace(";", ",").split(","):
            tok = tok.strip()
            if not tok:
                continue
            if ":" in tok:
                v, t = tok.split(":", 1)
                out.append((float(v), float(t)))
            else:
                out.append((float(tok), float(self.dwell)))
        if not out:
            raise ValueError(f"{self.name}: no steps")
        if any(t <= 0 for _, t in out):
            raise ValueError(f"{self.name}: step times must be > 0")
        return out

    def unit(self) -> str:
        return "rad/s" if self.mode == "vel" else "deg"


@dataclass
class TestSettings:
    duration: float = 60.0           # s, hard time limit of the whole test
    vbus_min: float = 40.0           # V, stop all below this (0 = off)
    stop_all_on_trip: bool = True
    record: bool = True


def default_plans() -> list[MotorPlan]:
    return [MotorPlan(name=f"J{i}", motor_id=i, steps=s, mode=m)
            for i, (m, s) in enumerate([("pos-csp", "0:2, 20:2, -20:2"), ("pos-csp", "0:2, -20:2, 20:2"),
                                        ("vel", "0:1, 1:1, -1:1"), ("mit", "0:2, 15:2, -15:2")], start=1)]


def load_plans(path: Path = PLAN_PATH) -> tuple[list[MotorPlan], TestSettings]:
    try:
        d = json.loads(path.read_text())
        known = MotorPlan.__dataclass_fields__
        plans = [MotorPlan(**{k: v for k, v in p.items() if k in known}) for p in d["motors"]]
        st = TestSettings(**{k: v for k, v in d.get("test", {}).items() if k in TestSettings.__dataclass_fields__})
        return plans, st
    except (OSError, ValueError, KeyError, TypeError):
        return default_plans(), TestSettings()


def save_plans(plans, settings, path: Path = PLAN_PATH):
    path.write_text(json.dumps({"motors": [asdict(p) for p in plans], "test": asdict(settings)}, indent=2))


# ====================================================================== rig: bus + motors + controllers
class MultiRig:
    """Opens the bus once and attaches one RS05 + Controller per motor ID through a BusHub."""

    def __init__(self, bus: can.BusABC, ids: list[int], host: int = 0xFD, poll_hz: float = 50.0,
                 models: Optional[dict] = None):
        self.bus, self.host = bus, host
        self.hub = BusHub(bus)
        self.motors: dict[int, RS05] = {}
        self.ctrls: dict[int, Controller] = {}
        self.missing: list[int] = []
        for mid in ids:
            m = RS05(bus, mid, host, hub=self.hub)
            m.stop()
            m.active_report(True)
            m.start_polling(poll_hz)
            self.motors[mid] = m
        for mid, m in self.motors.items():
            if m.wait_feedback(1.0) is None:
                m.stop()
                if m.wait_feedback(0.5) is None:
                    self.missing.append(mid)
            cfg = load_config(mid)
            if models and mid in models:
                cfg.model = models[mid]
            self.ctrls[mid] = Controller(m, cfg)

    def stop_all(self, reason="user"):
        for c in self.ctrls.values():
            try:
                c.stop(reason)
            except Exception:  # noqa: BLE001
                pass

    def save_configs(self):
        for mid, c in self.ctrls.items():
            save_config(mid, c.cfg)

    def close(self):
        self.stop_all()
        for c in self.ctrls.values():
            c.close()
        for m in self.motors.values():
            try:
                m.active_report(False)
            except can.CanError:
                pass
            m.close()
        self.hub.close()


# ====================================================================== ID checks
def check_ids(rig: MultiRig, expected: list[int], scan_range=range(1, 128)) -> tuple[bool, list[str]]:
    """1. each expected ID answers get-ID, and with exactly one UID (two UIDs = two motors share the ID)
       2. scan 1..127: report IDs that are on the bus but not expected (e.g. a leftover 0x7F)
       3. addressing: a parameter read sent to ID n is answered by ID n (mechPos + VBUS)."""
    hub, lines, ok = rig.hub, [], True

    def get_id(cid):
        arb = (T_GET_ID << 24) | (rig.host << 8) | cid
        with hub.tx_lock:
            rig.bus.send(can.Message(arbitration_id=arb, is_extended_id=True, data=bytes(8)), timeout=0.1)

    uids = {}
    for cid in expected:
        hub.id_replies.clear()
        get_id(cid)
        time.sleep(0.08)
        got = {u for s, u in list(hub.id_replies) if s == cid}
        uids[cid] = got
        if not got:
            ok = False
            lines.append(f"ID 0x{cid:02X} ({cid}): NO ANSWER - wrong ID, cable/connector, or power")
        elif len(got) > 1:
            ok = False
            lines.append(f"ID 0x{cid:02X} ({cid}): DUPLICATE - {len(got)} motors answer "
                         f"(UIDs {', '.join(u.hex() for u in got)}). Re-ID one of them alone on the bus.")
        else:
            lines.append(f"ID 0x{cid:02X} ({cid}): OK, UID {next(iter(got)).hex()}")

    hub.id_replies.clear()
    for cid in scan_range:
        get_id(cid)
        time.sleep(0.002)
    time.sleep(0.3)
    seen = sorted({s for s, _ in list(hub.id_replies)})
    extra = [s for s in seen if s not in expected]
    lines.append("bus scan 1..127: " + (", ".join(f"0x{s:02X}" for s in seen) or "nothing"))
    if extra:
        ok = False
        lines.append("UNEXPECTED IDs on the bus: " + ", ".join(f"0x{s:02X} ({s})" for s in extra)
                     + " - not in the plan (factory 0x7F left over?)")

    for cid in expected:
        m = rig.motors.get(cid)
        if not m or not uids.get(cid):
            continue
        try:
            pos = m.read("mechPos")
            vb = m.read("VBUS")
            lines.append(f"ID 0x{cid:02X}: addressed read OK - mechPos {math.degrees(pos):+.1f} deg, VBUS {vb:.1f} V")
        except TimeoutError as e:
            ok = False
            lines.append(f"ID 0x{cid:02X}: addressed read FAILED ({e})")
    return ok, lines


def identify(ctrl: Controller, amp_deg: float = 5.0, kp: float = 5.0, kd: float = 0.2):
    """Small MIT wiggle (+amp, -amp, back, stop) so you can see which joint this ID drives."""
    ctrl.set_mit(kp=kp, kd=kd, tff=0.0, ramp=1.0)
    ctrl.start_mit()
    p0 = ctrl.fb.pos
    try:
        for d in (amp_deg, -amp_deg, amp_deg, 0.0):
            ctrl.set_mit(target=p0 + math.radians(d))
            time.sleep(0.5)
            if ctrl.mode != "mit":
                break
    finally:
        ctrl.stop()


# ====================================================================== test runner
@dataclass
class Stats:
    fb_n: int = 0
    gap_max: float = 0.0
    gaps: int = 0
    iq_peak: float = 0.0
    iq_sq: float = 0.0
    iq_n: int = 0
    tq_peak: float = 0.0
    temp_max: float = 0.0
    vbus_min: float = math.inf
    vbus_max: float = 0.0
    faults: int = 0
    step_err: list = field(default_factory=list)       # (step value, error at end of step)
    _last_fb: Optional[float] = None

    @property
    def iq_rms(self):
        return math.sqrt(self.iq_sq / self.iq_n) if self.iq_n else float("nan")


@dataclass
class MotorRun:
    plan: MotorPlan
    ctrl: Controller
    steps: list
    state: str = "wait"           # wait | run | done | trip | off
    step: int = 0
    cycle: int = 0
    t_step: float = 0.0
    base: float = 0.0
    target: float = float("nan")
    note: str = ""
    stats: Stats = field(default_factory=Stats)

    def on_fb(self, fb):
        s = self.stats
        if self.state == "run" and s._last_fb is not None:
            g = fb.t - s._last_fb
            s.gap_max = max(s.gap_max, g)
            if g > GAP_WARN_S:
                s.gaps += 1
        s._last_fb = fb.t
        s.fb_n += 1


class MultiTest:
    """Runs every enabled MotorPlan on its controller at the same time, in its own thread."""

    def __init__(self, rig: MultiRig, plans: list[MotorPlan], settings: TestSettings, log=print):
        self.rig, self.settings, self.log = rig, settings, log
        self.runs: list[MotorRun] = []
        for p in plans:
            if not p.enabled:
                continue
            if p.motor_id not in rig.ctrls:
                raise ValueError(f"{p.name}: motor ID 0x{p.motor_id:02X} is not connected")
            if p.mode not in MODES:
                raise ValueError(f"{p.name}: mode must be one of {MODES}")
            self.runs.append(MotorRun(p, rig.ctrls[p.motor_id], p.parse_steps()))
        if not self.runs:
            raise ValueError("no motor enabled")
        ids = [r.plan.motor_id for r in self.runs]
        if len(set(ids)) != len(ids):
            raise ValueError("two plans use the same motor ID")
        for r in self.runs:                       # refuse before anything moves
            r.ctrl.motion_allowed()
            if r.plan.mode == "vel" and max(abs(v) for v, _ in r.steps) > r.plan.vmax:
                raise ValueError(f"{r.plan.name}: a speed step is above vmax {r.plan.vmax} rad/s")
        self.t0 = 0.0
        self.t_end: Optional[float] = None
        self.running = False
        self.end_reason = ""
        self.csv_path: Optional[Path] = None
        self.report_path: Optional[Path] = None
        self.vbus_trace: list[tuple[float, float]] = []
        self._stop = threading.Event()
        self._th: Optional[threading.Thread] = None
        self._csv = None
        self._w = None

    # ------------------------------------------------------------- control
    def start(self):
        LOG_DIR.mkdir(exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        if self.settings.record:
            self.csv_path = LOG_DIR / f"multi_{stamp}.csv"
            self._csv = open(self.csv_path, "w", newline="")
            self._w = csv.writer(self._csv)
            hdr = ["t_s"]
            for r in self.runs:
                n = r.plan.name
                hdr += [f"{n}_state", f"{n}_step", f"{n}_setpoint", f"{n}_pos_deg", f"{n}_vel", f"{n}_torque",
                        f"{n}_iq", f"{n}_temp", f"{n}_vbus", f"{n}_fault"]
            self._w.writerow(hdr)
        self.report_path = LOG_DIR / f"multi_{stamp}_report.txt"
        for r in self.runs:
            r.ctrl.recorder = lambda row, fb, r=r: r.on_fb(fb)
        self.t0 = time.monotonic()
        self.running = True
        self.log(f"TEST START: {len(self.runs)} motors, limit {self.settings.duration:.0f} s")
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def abort(self, reason="user STOP"):
        self.end_reason = self.end_reason or reason
        self._stop.set()

    def wait(self, timeout=None):
        if self._th:
            self._th.join(timeout)

    @property
    def elapsed(self):
        if not self.t0:
            return 0.0
        return (self.t_end or time.monotonic()) - self.t0

    # ------------------------------------------------------------- motor start / steps
    def _start_motor(self, r: MotorRun):
        p, c = r.plan, r.ctrl
        if c.cfg.model != p.model:
            c.cfg.model = p.model
            c.m.set_model(p.model)
        c.cfg.cur_limit = max(0.1, min(11.0, p.cur))
        c.cfg.pos_vmax = c.cfg.vel_max = max(0.05, p.vmax)
        c.cfg.pos_acc = c.cfg.vel_acc = max(0.1, p.acc)
        c.cfg.mit_kp_max = max(c.cfg.mit_kp_max, min(KP_MAX, p.kp))
        r.base = c.fb.pos if p.rel else 0.0
        if p.mode.startswith("pos"):
            c.start_pos("pp" if p.mode == "pos-pp" else "csp")
        elif p.mode == "vel":
            c.start_vel()
        else:
            c.set_mit(kp=p.kp, kd=p.kd, tff=0.0, ramp=p.ramp)
            c.start_mit()
        r.state, r.step, r.cycle = "run", 0, 0
        self._apply(r, time.monotonic())
        self.log(f"{p.name} (0x{p.motor_id:02X}) started: {p.mode}, {len(r.steps)} steps")

    def _apply(self, r: MotorRun, now: float):
        p, c = r.plan, r.ctrl
        val, _ = r.steps[r.step]
        r.t_step = now
        if p.mode == "vel":
            r.target = val
            c.set_vel(val)
        else:
            r.target = r.base + math.radians(val)
            if p.mode == "mit":
                c.set_mit(target=r.target)
            else:
                c.set_pos_target(r.target)

    def _step_error(self, r: MotorRun):
        fb = r.ctrl.fb
        if fb is None:
            return
        val = r.steps[r.step][0]
        err = (r.target - fb.vel) if r.plan.mode == "vel" else math.degrees(r.target - fb.pos)
        r.stats.step_err.append((val, err))

    def _finish_motor(self, r: MotorRun):
        r.state = "done"
        if r.plan.mode == "vel":
            r.ctrl.set_vel(0.0)
        if r.plan.end == "stop":
            if r.plan.mode == "vel":
                time.sleep(0.3)
            r.ctrl.stop()
        self.log(f"{r.plan.name} finished ({r.plan.end})")

    # ------------------------------------------------------------- loop
    def _loop(self):
        period = 1.0 / TICK_HZ
        nxt = time.monotonic()
        try:
            while not self._stop.is_set():
                nxt += period
                time.sleep(max(0.0, nxt - time.monotonic()))
                now = time.monotonic()
                t = now - self.t0
                if t >= self.settings.duration:
                    self.end_reason = self.end_reason or f"time limit {self.settings.duration:.0f} s reached"
                    break
                for r in self.runs:
                    try:
                        self._tick_motor(r, now, t)
                    except (SafetyTrip, TimeoutError, can.CanError, ValueError) as e:
                        r.state, r.note = "trip", str(e)
                        r.ctrl.stop(f"test: {e}")
                        self.log(f"{r.plan.name} TRIP: {e}")
                if self.settings.stop_all_on_trip and any(r.state == "trip" for r in self.runs):
                    bad = [r for r in self.runs if r.state == "trip"][0]
                    self.end_reason = f"{bad.plan.name} tripped: {bad.note}"
                    break
                vbs = [r.ctrl.m.s.vbus for r in self.runs if not math.isnan(r.ctrl.m.s.vbus)]
                if vbs:
                    self.vbus_trace.append((t, min(vbs)))
                    if self.settings.vbus_min > 0 and min(vbs) < self.settings.vbus_min:
                        self.end_reason = f"VBUS {min(vbs):.1f} V below {self.settings.vbus_min:.1f} V"
                        break
                if all(r.state in ("done", "trip", "off") for r in self.runs) and \
                        not any(r.state == "done" and r.plan.end == "hold" for r in self.runs):
                    self.end_reason = self.end_reason or "all sequences finished"
                    break
                self._write_row(t)
        except Exception as e:  # noqa: BLE001
            self.end_reason = f"test error: {e}"
        finally:
            self.t_end = time.monotonic()
            self.rig.stop_all("user")
            for r in self.runs:
                r.ctrl.recorder = None
                if r.state in ("run", "wait"):
                    r.state = "off"
            self.running = False
            if self._csv:
                self._csv.close()
            self.end_reason = self.end_reason or "stopped"
            self.log(f"TEST END after {self.elapsed:.1f} s: {self.end_reason}")
            self._write_report()

    def _tick_motor(self, r: MotorRun, now: float, t: float):
        p, c, s = r.plan, r.ctrl, r.stats
        fb, ms = c.fb, c.m.s
        if fb is not None and r.state in ("run", "done"):
            s.tq_peak = max(s.tq_peak, abs(fb.torque))
            s.temp_max = max(s.temp_max, fb.temp)
            s.faults |= fb.faults
        if not math.isnan(ms.iq) and r.state in ("run", "done"):
            s.iq_peak = max(s.iq_peak, abs(ms.iq))
            s.iq_sq += ms.iq * ms.iq
            s.iq_n += 1
        if not math.isnan(ms.vbus) and r.state in ("run", "done"):
            s.vbus_min = min(s.vbus_min, ms.vbus)
            s.vbus_max = max(s.vbus_max, ms.vbus)

        if r.state == "wait":
            if t >= p.delay:
                self._start_motor(r)
            return
        if r.state == "run":
            if c.mode == "idle":
                raise SafetyTrip(c.trip_reason or "controller stopped")
            if now - r.t_step >= r.steps[r.step][1]:
                self._step_error(r)
                r.step += 1
                if r.step >= len(r.steps):
                    r.step, r.cycle = 0, r.cycle + 1
                    if p.cycles and r.cycle >= p.cycles:
                        r.step = len(r.steps) - 1
                        self._finish_motor(r)
                        return
                self._apply(r, now)
        elif r.state == "done" and p.end == "hold" and c.mode == "idle":
            raise SafetyTrip(c.trip_reason or "controller stopped while holding")

    def _write_row(self, t):
        if not self._w:
            return
        row = [f"{t:.3f}"]
        for r in self.runs:
            fb, ms = r.ctrl.fb, r.ctrl.m.s
            sp = r.target if r.plan.mode == "vel" else math.degrees(r.target)
            row += [r.state, r.step, f"{sp:.3f}",
                    f"{math.degrees(fb.pos):.3f}" if fb else "", f"{fb.vel:.3f}" if fb else "",
                    f"{fb.torque:.3f}" if fb else "", f"{ms.iq:.3f}", f"{fb.temp:.1f}" if fb else "",
                    f"{ms.vbus:.2f}", fb.faults if fb else ""]
        self._w.writerow(row)

    # ------------------------------------------------------------- report
    def summary(self) -> str:
        hub = self.rig.hub
        L = [f"RS05 multi-motor test  {time.strftime('%Y-%m-%d %H:%M:%S')}",
             f"duration {self.elapsed:.1f} s / limit {self.settings.duration:.0f} s - end: {self.end_reason or 'running'}",
             f"CSV: {self.csv_path}", ""]
        hdr = (f"{'motor':10s}{'ID':>5s}{'mode':>9s}{'state':>7s}{'cyc':>5s}{'fb n':>8s}{'gap max':>9s}"
               f"{'gaps>50ms':>10s}{'iq pk A':>9s}{'iq rms':>8s}{'tq pk Nm':>9s}{'T max C':>8s}"
               f"{'VBUS min':>9s}{'VBUS max':>9s}{'tx err':>7s}  faults")
        L += [hdr, "-" * len(hdr)]
        for r in self.runs:
            s = r.stats
            L.append(f"{r.plan.name:10s}{r.plan.motor_id:>5d}{r.plan.mode:>9s}{r.state:>7s}{r.cycle:>5d}"
                     f"{s.fb_n:>8d}{1000 * s.gap_max:>7.0f}ms{s.gaps:>10d}{s.iq_peak:>9.2f}{s.iq_rms:>8.2f}"
                     f"{s.tq_peak:>9.2f}{s.temp_max:>8.1f}{s.vbus_min:>9.2f}{s.vbus_max:>9.2f}"
                     f"{r.ctrl.m.tx_errors:>7d}  {fault_str(s.faults)}")
        L.append("")
        for r in self.runs:
            if r.stats.step_err:
                errs = [abs(e) for _, e in r.stats.step_err]
                L.append(f"{r.plan.name}: end-of-step error mean {sum(errs) / len(errs):.2f} max {max(errs):.2f} "
                         f"{r.plan.unit()} over {len(errs)} steps")
            if r.note:
                L.append(f"{r.plan.name}: {r.note}")
        if self.vbus_trace:
            vmin = min(v for _, v in self.vbus_trace)
            L.append(f"\nVBUS (lowest of all motors): min {vmin:.2f} V over the test")
        L.append(f"bus: error frames {hub.error_frames}, rx errors {hub.rx_errors}, "
                 f"frames per ID {dict(sorted(hub.rx_count.items()))}")
        if hasattr(self.rig.bus, "stats_text"):          # through the STM32 gateway: per-channel counters
            L.append(self.rig.bus.stats_text())
        L.append("\nRead it as: gaps / tx errors / VBUS dips that grow with current point at cables or connectors;"
                 "\n'undervoltage' faults = supply or wiring drop; one motor alone misbehaving = its own lead.")
        return "\n".join(L)

    def _write_report(self):
        try:
            text = self.summary()
            ev = []
            for r in self.runs:
                ev += [f"[{r.plan.name}] {e}" for e in r.ctrl.events]
            self.report_path.write_text(text + "\n\nEvents:\n" + "\n".join(ev))
        except OSError as e:
            self.log(f"report not written: {e}")


# ====================================================================== CLI
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="COM9")
    ap.add_argument("--iface", default="slcan",
                    help="slcan | gs_usb (CANable direct) | stm (STM32 gateway, -p = its COM port) | stm-sim")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--host", type=lambda x: int(x, 0), default=0xFD)
    ap.add_argument("--sim", action="store_true", help="4 simulated motors")
    ap.add_argument("--plan", type=Path, default=PLAN_PATH)
    ap.add_argument("--ids", type=lambda x: int(x, 0), nargs="+", help="override: motor IDs (check-ids)")
    ap.add_argument("--check-ids", action="store_true")
    ap.add_argument("--duration", type=float)
    ap.add_argument("-y", "--yes", action="store_true")
    a = ap.parse_args()

    plans, st = load_plans(a.plan)
    if a.duration:
        st.duration = a.duration
    ids = a.ids or [p.motor_id for p in plans if p.enabled]
    sims, emu = [], None
    if a.iface == "stm-sim":                         # simulated STM: first half of the IDs on CAN1, rest on CAN2
        from rs05_stm_emu import open_stm_sim
        half = (len(ids) + 1) // 2
        bus, emu = open_stm_sim({0: ids[:half], 1: ids[half:]})
        a.sim = True
    elif a.sim:
        from rs05_sim import FakeRS05
        sims = [FakeRS05("multisim", i, a.host) for i in ids]
        bus = can.Bus(interface="virtual", channel="multisim")
    else:
        bus = open_bus(a.iface, a.port, a.bitrate)
    rig = MultiRig(bus, ids, a.host, models={p.motor_id: p.model for p in plans})
    try:
        if rig.missing:
            print("no feedback from:", ", ".join(f"0x{i:02X}" for i in rig.missing))
        ok, lines = check_ids(rig, ids)
        print("\n".join(lines))
        print("ID check:", "PASS" if ok else "FAIL")
        if a.check_ids:
            return
        if a.sim:
            for c in rig.ctrls.values():
                c.cfg.free_shaft = True
        for p in plans:
            if p.enabled:
                print(f"  {p.name} 0x{p.motor_id:02X} {p.mode:8s} steps [{p.steps}] {p.unit()}, cycles {p.cycles}, "
                      f"delay {p.delay}s, I {p.cur} A")
        if not a.yes and input(f"Run for max {st.duration:.0f} s? [y/N] ").strip().lower() != "y":
            return
        test = MultiTest(rig, plans, st)
        test.start()
        try:
            while test.running:
                print(f"  t {test.elapsed:6.1f} s  " + "  ".join(
                    f"{r.plan.name}:{r.state}/{r.step}" for r in test.runs), end="\r")
                time.sleep(0.5)
        except KeyboardInterrupt:
            test.abort("Ctrl+C")
        test.wait(3)
        print("\n" + test.summary())
        print("report:", test.report_path)
    finally:
        rig.close()
        bus.shutdown()
        for s in sims:
            s.close()
        if emu:
            emu.close()


if __name__ == "__main__":
    main()
