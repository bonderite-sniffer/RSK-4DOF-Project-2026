"""rs05_controller.py - motion modes + safety supervisor + friction identification for one RS05.

No GUI code here (rs05_gui.py drives it; it can also be scripted / tested headless).

Safety model (runs at 100 Hz in its own thread, in EVERY mode):
  * joint limits [lim_min, lim_max] (deg, taught in the GUI). Targets are clamped to them.
      position mode : target clamped; motor stopped if it ends up > HARD_MARGIN past a limit
      velocity mode : speed forced to 0 when the braking distance would cross a limit
      MIT/impedance : virtual wall (stiff spring) at the limit, stop if > MIT_HARD_MARGIN past it
  * feedback lost > 0.3 s, motor fault bits, over-temperature  ->  stop
  * motion is refused until limits are taught (or "free shaft" is ticked)
"""
from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from rs05_can import MODE_CSP, MODE_MIT, MODE_PP, MODE_SPEED, MODELS, RS05, fault_str

CONFIG_PATH = Path(__file__).resolve().parent / "rs05_gui_config.json"
LOOP_HZ = 100.0
HARD_MARGIN_DEG = 5.0       # pos / vel: stop if this far past a limit
MIT_HARD_MARGIN_DEG = 12.0  # MIT: stop if this far past a limit (wall should prevent it)
BRAKE_MARGIN_DEG = 2.0      # vel: extra distance kept before a limit
WALL_KP, WALL_KD = 40.0, 1.5
WALL_ZONE_DEG = 3.0         # MIT: damping zone inside each limit (+ braking distance)
MIT_BRAKE_ACC = 30.0        # rad/s^2 assumed when sizing the MIT braking zone
FREE_LIMIT_DEG = 700.0      # "free shaft": just stay inside the +-720 deg feedback range
VEL_DEADBAND = 0.2          # rad/s: below this the measured speed is treated as 0 when deciding the
                            # MIT wall / brake. The motor's own speed estimate jumps +-0.085 rad/s each
                            # time the encoder ticks one count, so its SIGN at standstill is random.


@dataclass
class JointConfig:
    lim_min_deg: float = -20.0
    lim_max_deg: float = 20.0
    limits_taught: bool = False
    free_shaft: bool = False
    model: str = "RS05"          # "RS05" (EduLite 05) or "RS02" - MIT / feedback scaling
    temp_max: float = 70.0
    cur_limit: float = 2.0       # A, position / velocity modes
    pos_vmax: float = 2.0        # rad/s
    pos_acc: float = 5.0         # rad/s^2
    vel_acc: float = 5.0         # rad/s^2
    vel_max: float = 10.0        # rad/s, GUI slider range
    mit_kp: float = 10.0
    mit_kd: float = 0.5
    mit_ramp: float = 2.0        # rad/s
    mit_tff_max: float = 1.0     # Nm
    mit_kp_max: float = 60.0
    mit_vel_max: float = 8.0     # rad/s, MIT overspeed brake


def load_config(motor_id: int) -> JointConfig:
    try:
        data = json.loads(CONFIG_PATH.read_text()).get(f"motor_{motor_id}", {})
        known = JointConfig.__dataclass_fields__
        return JointConfig(**{k: v for k, v in data.items() if k in known})
    except (OSError, ValueError):
        return JointConfig()


def save_config(motor_id: int, cfg: JointConfig):
    try:
        data = json.loads(CONFIG_PATH.read_text())
    except (OSError, ValueError):
        data = {}
    data[f"motor_{motor_id}"] = asdict(cfg)
    CONFIG_PATH.write_text(json.dumps(data, indent=2))


def move_config(old_id: int, new_id: int, cfg: JointConfig):
    """After a CAN ID change the joint settings follow the motor to its new ID."""
    try:
        data = json.loads(CONFIG_PATH.read_text())
    except (OSError, ValueError):
        data = {}
    data.pop(f"motor_{old_id}", None)
    data[f"motor_{new_id}"] = asdict(cfg)
    CONFIG_PATH.write_text(json.dumps(data, indent=2))


class SafetyTrip(RuntimeError):
    pass


class Controller:
    MODES = ("idle", "pos", "vel", "mit")

    def __init__(self, motor: RS05, cfg: JointConfig):
        self.m = motor
        self.cfg = cfg
        if cfg.model not in MODELS:
            cfg.model = "RS05"
        motor.set_model(cfg.model)
        self.mode = "idle"
        self.lock = threading.RLock()
        self.events: deque[str] = deque(maxlen=200)
        self.trip_reason = ""
        # position
        self.pos_kind = "pp"
        self.pos_target = 0.0
        # velocity
        self.vel_target = 0.0
        self.vel_sent = None
        self.vel_limited = False
        # MIT
        self.mit_target = 0.0
        self.mit_p_des = 0.0
        self.mit_kp = cfg.mit_kp
        self.mit_kd = cfg.mit_kd
        self.mit_tff = 0.0
        self.mit_ramp = cfg.mit_ramp
        self.mit_wall = False
        self.mit_overspeed = False
        # history for plots: (t, pos, setpoint_pos, vel, setpoint_vel, torque, iq)
        self.hist: deque = deque(maxlen=6000)
        self._t0 = time.monotonic()
        self._t_tick = None
        self.recorder: Optional[Callable] = None
        self.m.on_feedback = self._on_feedback
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------ helpers
    def log(self, text: str):
        self.events.append(time.strftime("%H:%M:%S ") + text)

    @property
    def fb(self):
        return self.m.s.fb

    def limits_rad(self):
        if self.cfg.free_shaft:
            return -math.radians(FREE_LIMIT_DEG), math.radians(FREE_LIMIT_DEG)
        return math.radians(self.cfg.lim_min_deg), math.radians(self.cfg.lim_max_deg)

    def clamp_to_limits(self, x: float) -> float:
        lo, hi = self.limits_rad()
        return min(max(x, lo), hi)

    def motion_allowed(self):
        if not (self.cfg.limits_taught or self.cfg.free_shaft):
            raise SafetyTrip("Teach the joint limits first (Setup tab), or tick 'free shaft'.")
        fb = self.fb
        if fb is None or time.monotonic() - fb.t > 0.3:
            raise SafetyTrip("No live feedback from the motor.")
        if fb.faults:
            raise SafetyTrip(f"Motor fault active: {fault_str(fb.faults)} - press Clear faults.")
        lo, hi = self.limits_rad()
        tol = math.radians(HARD_MARGIN_DEG)
        if not (lo - tol <= fb.pos <= hi + tol):
            raise SafetyTrip(f"Joint is at {math.degrees(fb.pos):+.1f} deg, outside the limits "
                             f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]. Move it inside by hand first.")

    def setpoints(self):
        if self.mode == "pos":
            return self.pos_target, float("nan")
        if self.mode == "vel":
            return float("nan"), (self.vel_sent or 0.0)
        if self.mode == "mit":
            return self.mit_p_des, float("nan")
        return float("nan"), float("nan")

    def _on_feedback(self, fb):
        sp, sv = self.setpoints()
        row = (fb.t - self._t0, fb.pos, sp, fb.vel, sv, fb.torque, self.m.s.iq)
        self.hist.append(row)
        if self.recorder:
            self.recorder(row, fb)

    # ------------------------------------------------------------------ stop / faults
    def stop(self, reason: str = "user"):
        with self.lock:
            was = self.mode
            self.mode = "idle"
            try:
                if was == "vel":
                    self.m.write("spd_ref", 0.0)
                self.m.stop()
            except Exception as e:      # noqa: BLE001 - keep trying to stop
                self.log(f"stop frame error: {e}")
            if reason != "user":
                self.trip_reason = reason
                self.log(f"SAFETY STOP: {reason}")
            elif was != "idle":
                self.log(f"stopped ({was})")

    def clear_faults(self):
        with self.lock:
            self.mode = "idle"
            self.m.stop(clear_fault=True)
            self.trip_reason = ""
            self.log("faults cleared")

    # ------------------------------------------------------------------ position
    def start_pos(self, kind: str = "pp", target: Optional[float] = None):
        """Enable position control. target None = hold where it is now."""
        self.motion_allowed()
        mech = self.m.read("mechPos")
        if abs(mech - self.fb.pos) > 0.05:
            raise SafetyTrip(f"mechPos {math.degrees(mech):.1f} deg != feedback {math.degrees(self.fb.pos):.1f} deg; "
                             f"re-zero the motor (Setup tab) before position control.")
        c = self.cfg
        with self.lock:
            self.stop()
            self.pos_kind = kind
            self.pos_target = self.clamp_to_limits(self.fb.pos if target is None else target)
            self.m.set_mode(MODE_PP if kind == "pp" else MODE_CSP)
            self.m.write("limit_cur", c.cur_limit)
            if kind == "pp":
                self.m.write("vel_max", c.pos_vmax)
                self.m.write("acc_set", c.pos_acc)
            else:
                self.m.write("limit_spd", c.pos_vmax)
            self.m.write("loc_ref", self.pos_target)
            self.mode = "pos"
            self.trip_reason = ""
        self.log(f"position {kind.upper()} on, target {math.degrees(self.pos_target):+.1f} deg")

    def set_pos_target(self, target_rad: float):
        with self.lock:
            if self.mode != "pos":
                raise SafetyTrip("Position mode is not enabled.")
            t = self.clamp_to_limits(target_rad)
            if t != target_rad:
                self.log(f"target clamped to limit {math.degrees(t):+.1f} deg")
            self.pos_target = t
            self.m.write("loc_ref", t)

    def update_pos_params(self):
        with self.lock:
            if self.mode != "pos":
                return
            self.m.write("limit_cur", self.cfg.cur_limit)
            self.m.write("vel_max" if self.pos_kind == "pp" else "limit_spd", self.cfg.pos_vmax)
            if self.pos_kind == "pp":
                self.m.write("acc_set", self.cfg.pos_acc)

    # ------------------------------------------------------------------ velocity
    def start_vel(self):
        self.motion_allowed()
        with self.lock:
            self.stop()
            self.m.set_mode(MODE_SPEED)
            self.m.write("limit_cur", self.cfg.cur_limit)
            self.m.write("acc_rad", self.cfg.vel_acc)
            self.m.write("spd_ref", 0.0)
            self.vel_target, self.vel_sent, self.vel_limited = 0.0, 0.0, False
            self.mode = "vel"
            self.trip_reason = ""
        self.log("velocity mode on (0 rad/s)")

    def set_vel(self, v: float):
        with self.lock:
            if self.mode != "vel":
                raise SafetyTrip("Velocity mode is not enabled.")
            self.vel_target = max(-self.cfg.vel_max, min(self.cfg.vel_max, v))

    def update_vel_params(self):
        with self.lock:
            if self.mode == "vel":
                self.m.write("limit_cur", self.cfg.cur_limit)
                self.m.write("acc_rad", self.cfg.vel_acc)

    # ------------------------------------------------------------------ MIT / impedance
    def start_mit(self):
        self.motion_allowed()
        with self.lock:
            self.stop()
            self._t_tick = None
            self.mit_target = self.mit_p_des = self.clamp_to_limits(self.fb.pos)
            self.m.set_mode(MODE_MIT)
            self.mode = "mit"
            self.trip_reason = ""
        self.log(f"MIT on: kp {self.mit_kp} kd {self.mit_kd} tff {self.mit_tff}")

    def set_mit(self, target=None, kp=None, kd=None, tff=None, ramp=None):
        with self.lock:
            if target is not None:
                self.mit_target = self.clamp_to_limits(target)
            if kp is not None:
                self.mit_kp = max(0.0, min(self.cfg.mit_kp_max, kp))
            if kd is not None:
                self.mit_kd = max(0.0, min(self.m.kd_max, kd))
            if tff is not None:
                self.mit_tff = max(-self.cfg.mit_tff_max, min(self.cfg.mit_tff_max, tff))
            if ramp is not None:
                self.mit_ramp = max(0.05, ramp)

    # ------------------------------------------------------------------ zero / limits
    def zero_here(self, save: bool = False):
        """Current position -> 0. Limits are shifted so they stay at the same physical place."""
        with self.lock:
            if self.mode != "idle":
                raise SafetyTrip("Stop the motor before zeroing.")
            self.m.stop()
            time.sleep(0.05)
            old = self.m.read("mechPos")
            self.m.set_zero()
            time.sleep(0.2)
            self.m.stop()
            self.m.wait_feedback(0.5)
            new = self.m.read("mechPos")
            ok = abs(new) < 0.02
            if ok and self.cfg.limits_taught:
                d = math.degrees(old)
                self.cfg.lim_min_deg -= d
                self.cfg.lim_max_deg -= d
            if save:
                self.m.save_params()
                time.sleep(0.2)
            self.log(f"zero {'OK' if ok else 'FAILED'}: {math.degrees(old):+.2f} -> {math.degrees(new):+.2f} deg"
                     + (" (saved to flash)" if save else ""))
            return old, new, ok

    def teach(self, which: str):
        if self.fb is None:
            raise SafetyTrip("No feedback.")
        deg = math.degrees(self.fb.pos)
        if which == "min":
            self.cfg.lim_min_deg = deg
        else:
            self.cfg.lim_max_deg = deg
        self.log(f"limit {which} taught at {deg:+.1f} deg")
        return deg

    def set_limits(self, lo_deg: float, hi_deg: float):
        if hi_deg - lo_deg < 2.0:
            raise SafetyTrip("Max limit must be at least 2 deg above min limit.")
        if max(abs(lo_deg), abs(hi_deg)) > FREE_LIMIT_DEG:
            raise SafetyTrip(f"Limits must be within +-{FREE_LIMIT_DEG:.0f} deg.")
        self.cfg.lim_min_deg, self.cfg.lim_max_deg = lo_deg, hi_deg
        self.cfg.limits_taught = True
        self.log(f"limits set [{lo_deg:+.1f}, {hi_deg:+.1f}] deg")

    # ------------------------------------------------------------------ supervisor loop
    def _loop(self):
        period = 1.0 / LOOP_HZ
        nxt = time.monotonic()
        poke = 0
        while not self._stop.is_set():
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 0.5:
                nxt = time.monotonic()
            try:
                with self.lock:
                    self._tick()
                    if self.mode == "idle":
                        poke += 1
                        fb = self.fb
                        if (fb is None or time.monotonic() - fb.t > 0.3) and poke % 10 == 0:
                            self.m.stop()        # no active reporting -> poll feedback
            except SafetyTrip as e:
                self.stop(str(e))
            except Exception as e:  # noqa: BLE001
                self.stop(f"controller error: {e}")

    def _tick(self):
        if self.mode == "idle":
            return
        fb = self.fb
        now = time.monotonic()
        if fb is None or now - fb.t > 0.3:
            raise SafetyTrip("feedback lost")
        if fb.faults:
            raise SafetyTrip(f"motor fault: {fault_str(fb.faults)}")
        if fb.temp > self.cfg.temp_max:
            raise SafetyTrip(f"over temperature {fb.temp:.1f} C")
        lo, hi = self.limits_rad()
        pos = fb.pos
        hard = math.radians(MIT_HARD_MARGIN_DEG if self.mode == "mit" else HARD_MARGIN_DEG)
        if pos > hi + hard or pos < lo - hard:
            raise SafetyTrip(f"joint at {math.degrees(pos):+.1f} deg, past limit "
                             f"[{math.degrees(lo):.1f}, {math.degrees(hi):.1f}]")

        if self.mode == "vel":
            v = self.vel_target
            acc = max(self.cfg.vel_acc, 0.1)
            brake = max(v * v, fb.vel * fb.vel) / (2 * acc) + math.radians(BRAKE_MARGIN_DEG)
            limited = (v > 0 and pos + brake >= hi) or (v < 0 and pos - brake <= lo)
            if limited:
                v = 0.0
                if not self.vel_limited:
                    self.log("velocity stopped at joint limit")
            self.vel_limited = limited
            if v != self.vel_sent:
                self.m.write("spd_ref", v)
                self.vel_sent = v

        elif self.mode == "mit":
            dp = self.mit_target - self.mit_p_des
            dt = min(0.05, now - self._t_tick) if self._t_tick else 1.0 / LOOP_HZ
            self._t_tick = now
            step = self.mit_ramp * dt          # real elapsed time: ramp speed stays right on late ticks
            if abs(dp) > step:
                self.mit_p_des += math.copysign(step, dp)
                v_des = math.copysign(self.mit_ramp, dp)
            else:
                self.mit_p_des, v_des = self.mit_target, 0.0
            p, kp, kd, tff = self.mit_p_des, self.mit_kp, self.mit_kd, self.mit_tff
            wall = pos > hi or pos < lo
            vel = fb.vel if abs(fb.vel) > VEL_DEADBAND else 0.0
            zone = math.radians(WALL_ZONE_DEG) + vel * vel / (2 * MIT_BRAKE_ACC)
            approaching = (vel > 0 and pos > hi - zone) or (vel < 0 and pos < lo + zone)
            overspeed = abs(vel) > self.cfg.mit_vel_max
            if overspeed and not self.mit_overspeed:
                self.log(f"MIT overspeed brake ({fb.vel:+.1f} rad/s)")
            self.mit_overspeed = overspeed
            if overspeed:
                approaching = True
            if wall:                       # virtual wall: stiff, damped spring back to the limit
                p = hi if pos > hi else lo
                kp, kd, tff, v_des = max(kp, WALL_KP), max(kd, WALL_KD), 0.0, 0.0
            elif approaching:              # just inside the limit and moving toward it: add damping
                kd = max(kd, WALL_KD)
                tff = 0.0 if (tff > 0) == (vel > 0) else tff
            if wall and not self.mit_wall:
                self.log("MIT virtual wall active")
            self.mit_wall = wall
            pm, tm = self.m.p_max, self.m.t_max
            self.m.mit(max(-pm, min(pm, p)), v_des, kp, kd, max(-tm, min(tm, tff)))

    def close(self):
        self.stop()
        self._stop.set()
        self._thread.join(timeout=1.0)


# ====================================================================== friction identification
@dataclass
class FrictionParams:
    drive: str = "vel"           # how the constant-speed passes are made:
                                 #   "vel" speed loop (spd_ref) | "csp" position CSP at limit_spd = v
                                 #   "mit" MIT: θd ramps at v with ωd = v, torque = Kp·e + Kd·ė
    speeds: list = field(default_factory=lambda: [0.3, 0.6, 1.0, 1.5, 2.0, 3.0])   # rad/s
    passes: int = 2              # passes per direction per speed
    acc: float = 10.0            # rad/s^2 during the sweep (higher = shorter range needed)
    edge_deg: float = 5.0        # ignore samples this close to the limits
    breakaway: bool = True
    tff_max: float = 0.8         # Nm, breakaway ramp ceiling
    tff_rate: float = 0.1        # Nm/s
    break_vel: float = 0.15      # rad/s => "it moved"
    mit_kp: float = 2.0          # drive "mit". Keep Kp SMALL: θd is updated at 100 Hz, so the motor sees a
    mit_kd: float = 1.0          # staircase; Kp·v·10 ms of sawtooth torque (0.1 Nm at Kp 10, 1 rad/s) is
                                 # bigger than the friction and makes the joint stick-slip. Kd tracks ωd
                                 # smoothly (ωd is constant during a pass), so let Kd do the work.


FRICTION_DRIVES = {"vel": "velocity loop", "csp": "position CSP", "mit": "MIT"}


@dataclass
class FrictionResult:
    points: list = field(default_factory=list)       # (v_cmd, v_mean, torque_mean, torque_sd, n)
    raw: list = field(default_factory=list)          # (vel, torque) steady-state samples
    fc_pos: float = float("nan")
    fc_neg: float = float("nan")
    b_pos: float = float("nan")
    b_neg: float = float("nan")
    fc: float = float("nan")         # symmetric fit: tau = fc*sign(v) + b*v + offset
    b: float = float("nan")
    offset: float = float("nan")
    r2: float = float("nan")
    fs_pos: float = float("nan")     # breakaway torque
    fs_neg: float = float("nan")

    def text(self) -> str:
        f = lambda x: "  -  " if math.isnan(x) else f"{x:+.4f}"  # noqa: E731
        return (f"Model  tau = Fc*sign(w) + B*w + offset\n"
                f"  Fc (Coulomb)      = {f(self.fc)} Nm\n"
                f"  B  (viscous)      = {f(self.b)} Nm.s/rad\n"
                f"  offset (gravity/bias) = {f(self.offset)} Nm     R^2 = {f(self.r2)}\n"
                f"Per direction\n"
                f"  +w : Fc {f(self.fc_pos)} Nm  B {f(self.b_pos)}\n"
                f"  -w : Fc {f(self.fc_neg)} Nm  B {f(self.b_neg)}\n"
                f"Breakaway (static) torque\n"
                f"  +  : {f(self.fs_pos)} Nm     - : {f(self.fs_neg)} Nm")


class FrictionTest:
    """Velocity sweep back and forth between the joint limits, then a torque-ramp breakaway test.
    Run in a thread: FrictionTest(ctrl, params).run(); poll .status / .progress; .abort()."""

    def __init__(self, ctrl: Controller, params: FrictionParams):
        self.c = ctrl
        self.p = params
        self.res = FrictionResult()
        self.status = "idle"
        self.progress = 0.0
        self.done = False
        self.error = ""
        self._abort = threading.Event()

    def abort(self):
        self._abort.set()

    @property
    def ctrl_mode(self) -> str:
        return {"vel": "vel", "csp": "pos", "mit": "mit"}[self.p.drive]

    def _check(self, mode: str):
        if self._abort.is_set():
            raise SafetyTrip("aborted")
        if self.c.mode != mode:
            raise SafetyTrip(f"controller left {mode} mode: {self.c.trip_reason or 'stopped'}")

    def run(self):
        try:
            self._sweep()
            self._fit()
            if self.p.breakaway:
                self._breakaway()
            self.status = "done"
        except SafetyTrip as e:
            self.error = str(e)
            self.status = f"stopped: {e}"
        except Exception as e:  # noqa: BLE001
            self.error = repr(e)
            self.status = f"error: {e!r}"
        finally:
            self.c.stop()
            self.done = True
            self.progress = 1.0

    # ------------------------------------------------------------------ sweep
    def _sweep(self):
        c, p = self.c, self.p
        saved_acc = c.cfg.vel_acc
        c.cfg.vel_acc = p.acc
        try:
            self._sweep_inner()
        finally:
            c.cfg.vel_acc = saved_acc

    def required_range_deg(self) -> float:
        top = max(abs(v) for v in self.p.speeds)
        return math.degrees(top * top / self.p.acc + 0.5 * top) + 2 * self.p.edge_deg + 2 * BRAKE_MARGIN_DEG

    def _drive_start(self):
        c, p = self.c, self.p
        if p.drive == "vel":
            c.start_vel()
        elif p.drive == "csp":
            c.start_pos("csp")                                   # holds where it is
        elif p.drive == "mit":
            c.set_mit(kp=p.mit_kp, kd=p.mit_kd, tff=0.0, ramp=min(abs(v) for v in p.speeds))
            c.start_mit()
        else:
            raise SafetyTrip(f"unknown friction drive {p.drive!r}")

    def _drive_go(self, d: int, v: float, far: float):
        """Start one pass toward `far` at speed v."""
        c, p = self.c, self.p
        if p.drive == "vel":
            c.set_vel(d * v)
        elif p.drive == "csp":
            with c.lock:
                c.m.write("limit_spd", v)
            c.set_pos_target(far)
        else:
            c.set_mit(ramp=v, target=far)

    def _drive_halt(self):
        c, p = self.c, self.p
        if p.drive == "vel":
            c.set_vel(0.0)
        elif p.drive == "csp":
            c.set_pos_target(c.fb.pos)
        else:
            c.set_mit(target=c.fb.pos)

    def _sweep_inner(self):
        c, p = self.c, self.p
        mode = self.ctrl_mode
        lo, hi = c.limits_rad()
        edge = math.radians(p.edge_deg)
        acc = p.acc
        top = max(abs(v) for v in p.speeds)
        need = self.required_range_deg()
        if math.degrees(hi - lo) < need:
            raise SafetyTrip(f"joint range {math.degrees(hi - lo):.0f} deg too small for {top} rad/s "
                             f"(needs ~{need:.0f} deg). Lower the top speed or raise acceleration.")
        self._drive_start()
        n_total = len(p.speeds) * 2 * p.passes
        n_done = 0
        for v in p.speeds:
            v = abs(v)
            brake = v * v / (2 * acc) + math.radians(BRAKE_MARGIN_DEG) + 0.02
            for k in range(2 * p.passes):
                d = 1 if c.fb.pos < (lo + hi) / 2 else -1     # head toward the far side
                far = hi - brake if d > 0 else lo + brake
                self.status = (f"sweep [{FRICTION_DRIVES[p.drive]}] {v:.2f} rad/s {'+' if d > 0 else '-'} "
                               f"({n_done + 1}/{n_total})")
                self._drive_go(d, v, far)
                samples = []
                t0 = time.monotonic()
                while True:
                    self._check(mode)
                    fb = c.fb
                    if p.drive == "vel":
                        if (d > 0 and fb.pos + brake >= hi) or (d < 0 and fb.pos - brake <= lo):
                            break
                    elif (abs(fb.pos - far) < math.radians(1.0) or (fb.pos - far) * d > 0
                          or (abs(fb.vel) < 0.05 and abs(fb.pos - far) < math.radians(8.0)
                              and time.monotonic() - t0 > 0.5)):
                        break      # csp / mit: at the end, or stopped just short of it (low Kp + stiction)
                    if time.monotonic() - t0 > (hi - lo) / v + 5:
                        raise SafetyTrip("sweep pass timed out (motor not moving?)")
                    in_mid = lo + edge < fb.pos < hi - edge
                    steady = abs(fb.vel - d * v) < 0.1 * v + 0.05 and time.monotonic() - t0 > 0.3
                    if in_mid and steady:
                        samples.append((fb.t, fb.vel, fb.torque))
                    time.sleep(0.005)
                self._drive_halt()
                t_stop = time.monotonic()
                while abs(c.fb.vel) > 0.05 and time.monotonic() - t_stop < 3:
                    self._check(mode)
                    time.sleep(0.01)
                uniq = {s[0]: s for s in samples}.values()          # one sample per frame
                if len(uniq) >= 5:
                    vel = np.array([s[1] for s in uniq])
                    tq = np.array([s[2] for s in uniq])
                    self.res.points.append((d * v, float(vel.mean()), float(tq.mean()), float(tq.std()), len(tq)))
                    self.res.raw.extend(zip(vel.tolist(), tq.tolist()))
                n_done += 1
                self.progress = 0.8 * n_done / n_total
        c.stop()
        if len(self.res.points) < 3:
            raise SafetyTrip("not enough steady-state data (range too small or speed not reached)")

    def _fit(self):
        r = self.res
        pts = np.array([(pt[1], pt[2]) for pt in r.points])
        w, tq = pts[:, 0], pts[:, 1]
        A = np.column_stack([np.sign(w), w, np.ones_like(w)])
        sol, *_ = np.linalg.lstsq(A, tq, rcond=None)
        r.fc, r.b, r.offset = map(float, sol)
        pred = A @ sol
        ss = float(((tq - tq.mean()) ** 2).sum())
        r.r2 = 1 - float(((tq - pred) ** 2).sum()) / ss if ss > 0 else float("nan")
        for sign, attr in ((1, "pos"), (-1, "neg")):
            m = np.sign(w) == sign
            if m.sum() >= 2:
                slope, icpt = np.polyfit(w[m], tq[m], 1)
                setattr(r, f"b_{attr}", float(slope))
                setattr(r, f"fc_{attr}", float(icpt))

    # ------------------------------------------------------------------ breakaway
    def _breakaway(self):
        c, p = self.c, self.p
        lo, hi = c.limits_rad()
        mid = (lo + hi) / 2
        self.status = "moving to mid-range for breakaway test"
        saved = (c.mit_kp, c.mit_kd, c.mit_tff, c.mit_ramp)
        if p.drive == "mit":
            # positioning only (nothing measured): stiffer than the sweep Kp, or stiction leaves it degrees short
            c.set_mit(kp=max(p.mit_kp, 10.0), kd=max(p.mit_kd, 0.5), tff=0.0, ramp=1.0)
            c.start_mit()
            c.set_mit(target=mid)
            move_mode = "mit"
        else:
            c.start_pos("csp", mid)
            move_mode = "pos"
        t0 = time.monotonic()
        while abs(c.fb.pos - mid) > math.radians(1) or abs(c.fb.vel) > 0.05:
            self._check(move_mode)
            if time.monotonic() - t0 > 15:
                raise SafetyTrip("could not reach mid-range")
            time.sleep(0.02)
        span = min(hi - mid, mid - lo)
        try:
            for sign, attr in ((1, "pos"), (-1, "neg")):
                self._breakaway_one(sign, attr, mid, span)
        finally:
            c.mit_kp, c.mit_kd, c.mit_tff, c.mit_ramp = saved

    def _breakaway_one(self, sign, attr, mid, span):
        c, p = self.c, self.p
        self.status = f"breakaway {'+' if sign > 0 else '-'}: ramping torque"
        c.set_mit(kp=0.0, kd=0.0, tff=0.0)
        c.start_mit()
        start = c.fb.pos
        tff, moved = 0.0, False
        t_last = time.monotonic()
        while True:
            self._check("mit")
            now = time.monotonic()
            tff += p.tff_rate * (now - t_last)
            t_last = now
            c.set_mit(tff=sign * tff)
            fb = c.fb
            moved = abs(fb.vel) > p.break_vel or abs(fb.pos - start) > math.radians(3)
            if moved or tff >= p.tff_max or abs(fb.pos - mid) > 0.5 * span:
                break
            time.sleep(0.005)
        c.set_mit(tff=0.0, kd=1.0)          # brake with damping
        time.sleep(0.3)
        if moved:
            setattr(self.res, f"fs_{attr}", sign * tff)
        self.progress = 0.8 + (0.1 if sign > 0 else 0.2)
        c.stop()
        time.sleep(0.2)


# ====================================================================== tuning: step response
STEP_MODES = {"mit": "MIT (Kp / Kd)", "pp": "Position PP", "csp": "Position CSP"}
LOOP_GAINS = ("loc_kp", "spd_kp", "spd_ki", "spd_filt_gain")   # motor-internal loops (PP / CSP / speed)


@dataclass
class StepParams:
    mode: str = "mit"            # "mit" | "pp" | "csp"
    step_deg: float = 10.0       # steps go start -> start+step -> start -> ...
    reps: int = 3                # up+down pairs
    dwell: float = 1.5           # s per step (recording window)
    kp: float = 10.0             # MIT
    kd: float = 0.5
    ramp: float = 0.0            # MIT target ramp rad/s, 0 = true step
    vmax: float = 3.0            # PP / CSP speed limit rad/s
    acc: float = 20.0            # PP accel rad/s^2
    cur: float = 2.0             # PP / CSP current limit A
    band_pct: float = 2.0        # settling band, % of the step (at least 0.2 deg)


@dataclass
class StepMetrics:
    direction: int
    start_deg: float
    target_deg: float
    rise_s: float = float("nan")       # 10 % -> 90 %
    overshoot_pct: float = float("nan")
    settle_s: float = float("nan")     # first time it stays inside the band
    sse_deg: float = float("nan")      # target - pos, mean of the last 20 % of the window
    tq_peak: float = float("nan")
    band_deg: float = float("nan")     # settling band used


def _nan(x) -> bool:
    return x != x


@dataclass
class StepResult:
    params: dict = field(default_factory=dict)
    traces: list = field(default_factory=list)    # per step: [(t, pos, vel, torque)], t = 0 at the command
    metrics: list = field(default_factory=list)   # StepMetrics per step

    def averages(self) -> dict:
        out = {}
        for attr in ("rise_s", "overshoot_pct", "settle_s", "sse_deg", "tq_peak"):
            xs = [abs(getattr(m, attr)) if attr == "sse_deg" else getattr(m, attr) for m in self.metrics]
            xs = [x for x in xs if not _nan(x)]
            out[attr] = sum(xs) / len(xs) if xs else float("nan")
        return out

    def text(self) -> str:
        def f(x, fmt):
            return "     -" if _nan(x) else format(x, fmt)
        L = [f"{'#':>3} {'dir':>3} {'rise ms':>8} {'OS %':>6} {'settle ms':>9} {'sse deg':>8} {'tq pk':>6}"]
        for i, m in enumerate(self.metrics, 1):
            L.append(f"{i:>3} {'+' if m.direction > 0 else '-':>3} {f(1000 * m.rise_s, '8.0f')} "
                     f"{f(m.overshoot_pct, '6.1f')} {f(1000 * m.settle_s, '9.0f')} {f(m.sse_deg, '+8.3f')} "
                     f"{f(m.tq_peak, '6.2f')}")
        if self.metrics:
            a = self.averages()
            L.append(f"{'avg':>7} {f(1000 * a['rise_s'], '8.0f')} {f(a['overshoot_pct'], '6.1f')} "
                     f"{f(1000 * a['settle_s'], '9.0f')} {f(a['sse_deg'], '8.3f')} {f(a['tq_peak'], '6.2f')}"
                     f"   (sse avg = |sse|)")
            unsettled = [m for m in self.metrics if _nan(m.settle_s)]
            offset = [m for m in unsettled if abs(m.sse_deg) > m.band_deg]
            if offset:
                L.append(f"{len(offset)} step(s) end OUTSIDE the band (steady-state error > {offset[0].band_deg:.2f} deg):"
                         f" friction / gravity vs stiffness -> raise Kp (MIT) or loc_kp, or add tff")
            if len(unsettled) > len(offset):
                L.append(f"{len(unsettled) - len(offset)} step(s) still moving / oscillating at the end of the "
                         f"window -> longer dwell or more damping (Kd)")
        return "\n".join(L)


def step_metrics(trace, start, target, band_pct=2.0) -> StepMetrics:
    """trace: [(t, pos, vel, torque)] with t = 0 at the step command. Angles in rad."""
    d = target - start
    m = StepMetrics(direction=1 if d > 0 else -1, start_deg=math.degrees(start), target_deg=math.degrees(target))
    if len(trace) < 3 or abs(d) < 1e-6:
        return m
    t = np.array([x[0] for x in trace])
    y = (np.array([x[1] for x in trace]) - start) / d          # 0 -> 1
    m.tq_peak = float(np.abs([x[3] for x in trace]).max())
    i10, i90 = int(np.argmax(y >= 0.1)), int(np.argmax(y >= 0.9))
    if y[i10] >= 0.1 and y[i90] >= 0.9:
        m.rise_s = float(t[i90] - t[i10])
    m.overshoot_pct = float(max(0.0, y.max() - 1.0) * 100)
    band = max(band_pct / 100, math.radians(0.2) / abs(d))
    m.band_deg = band * abs(math.degrees(d))
    outside = np.nonzero(np.abs(y - 1.0) > band)[0]
    if len(outside) == 0:
        m.settle_s = 0.0
    elif outside[-1] < len(y) - 1:
        m.settle_s = float(t[outside[-1] + 1])
    tail = np.array([x[1] for x in trace[int(len(trace) * 0.8):]])
    m.sse_deg = float(math.degrees(target - tail.mean()))
    return m


class StepTest:
    """Step-response test in MIT, PP or CSP: start -> start+step -> start, `reps` times.
    Run in a thread: StepTest(ctrl, params).run(); poll .status / .progress; .abort()."""

    def __init__(self, ctrl: Controller, params: StepParams):
        self.c, self.p = ctrl, params
        self.res = StepResult(params=asdict(params))
        self.status, self.progress, self.done, self.error = "idle", 0.0, False, ""
        self._abort = threading.Event()

    def abort(self):
        self._abort.set()

    def plan(self):
        """(start, other) in rad, both inside the limits, else SafetyTrip."""
        c, p = self.c, self.p
        lo, hi = c.limits_rad()
        start = c.clamp_to_limits(c.fb.pos)
        step = math.radians(abs(p.step_deg))
        margin = math.radians(1.0)
        if start + step <= hi - margin:
            return start, start + step
        if start - step >= lo + margin:
            return start, start - step
        raise SafetyTrip(f"no room for a {abs(p.step_deg):.1f} deg step inside the limits from here; "
                         f"move the joint toward the middle or use a smaller step")

    def run(self):
        c, p = self.c, self.p
        mode = "mit" if p.mode == "mit" else "pos"
        saved = (c.mit_kp, c.mit_kd, c.mit_tff, c.mit_ramp, c.cfg.cur_limit, c.cfg.pos_vmax, c.cfg.pos_acc)
        try:
            c.motion_allowed()
            a, b = self.plan()
            if p.mode == "mit":
                c.set_mit(kp=p.kp, kd=p.kd, tff=0.0, ramp=p.ramp if p.ramp > 0 else 1000.0)
                c.start_mit()
            elif p.mode in ("pp", "csp"):
                c.cfg.cur_limit, c.cfg.pos_vmax, c.cfg.pos_acc = p.cur, p.vmax, p.acc
                c.start_pos(p.mode)
            else:
                raise SafetyTrip(f"unknown step mode {p.mode!r}")
            time.sleep(0.5)                                     # settle at the start point
            n = 2 * max(1, p.reps)
            for i in range(n):
                to = b if i % 2 == 0 else a
                self.status = f"step {i + 1}/{n} [{STEP_MODES[p.mode]}] -> {math.degrees(to):+.1f} deg"
                start = c.fb.pos
                t0 = time.monotonic()
                if p.mode == "mit":
                    c.set_mit(target=to)
                else:
                    c.set_pos_target(to)
                trace, last = [], None
                while time.monotonic() - t0 < p.dwell:
                    if self._abort.is_set():
                        raise SafetyTrip("aborted")
                    if c.mode != mode:
                        raise SafetyTrip(f"controller left {mode} mode: {c.trip_reason or 'stopped'}")
                    fb = c.fb
                    if fb.t != last:
                        trace.append((fb.t - t0, fb.pos, fb.vel, fb.torque))
                        last = fb.t
                    time.sleep(0.002)
                self.res.traces.append(trace)
                self.res.metrics.append(step_metrics(trace, start, to, p.band_pct))
                self.progress = (i + 1) / n
            self.status = "done"
        except SafetyTrip as e:
            self.error, self.status = str(e), f"stopped: {e}"
        except Exception as e:  # noqa: BLE001
            self.error, self.status = repr(e), f"error: {e!r}"
        finally:
            c.stop()
            (c.mit_kp, c.mit_kd, c.mit_tff, c.mit_ramp, c.cfg.cur_limit, c.cfg.pos_vmax, c.cfg.pos_acc) = saved
            self.done, self.progress = True, 1.0


def read_loop_gains(motor: RS05) -> dict:
    return {k: motor.read(k) for k in LOOP_GAINS}


def write_loop_gains(motor: RS05, gains: dict):
    """RAM only (lost at power-off) unless motor.save_params() is called afterwards."""
    for k, v in gains.items():
        if k not in LOOP_GAINS:
            raise ValueError(f"{k} is not a loop gain")
        if not (v > 0 and math.isfinite(v)):
            raise ValueError(f"{k} must be > 0")
    for k, v in gains.items():
        motor.write(k, v)
