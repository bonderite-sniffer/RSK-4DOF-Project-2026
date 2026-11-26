"""rs05_can.py - RobStride RS05 (EduLite 05) private CAN protocol over a CANable v2.

Talks DIRECTLY to one motor from the PC (no STM32 in between). Used by
rs05_canable_test.py; can also be imported from your own scripts.

Private protocol, 29-bit extended ID, 1 Mbit/s:
    bits 28..24  communication type
    bits 23..8   "data area 2" (host ID, or torque for type 1)
    bits  7..0   target motor CAN ID

Frames used here
    type 0   get device ID           reply type 0  (bits 15..8 = motor ID)
    type 1   operation control (MIT) torque in ID bits 23..8, p/v/kp/kd in data (big endian)
    type 2   feedback                p, v, t, temp (big endian u16), mode + faults in ID
    type 3   enable
    type 4   stop                    data[0] = 1 also clears faults
    type 6   set mechanical zero     data[0] = 1
    type 17  read parameter          data[0..1] = index (LE), reply data[4..7] = value (LE)
    type 18  write parameter         data[0..1] = index (LE), data[4..7] = value (LE), RAM only
    type 21  fault feedback
    type 22  save parameters to flash  data = 01..08
    type 24  active reporting on/off   data = 01 02 03 04 05 06 F 00
"""
from __future__ import annotations

import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import can

# ------------------------------------------------------------------ RS05 ranges
P_MAX = 4 * math.pi      # +-12.566 rad  (type-1 command and type-2 feedback)
V_MAX = 50.0             # rad/s
T_MAX = 5.5              # Nm   (RS05 peak torque; type-1 torque field and feedback)
KP_MAX = 500.0
KD_MAX = 5.0
I_MAX = 11.0             # A    (limit_cur / iq_ref range)

# Scaling of the type-1 command / type-2 feedback fields differs per model (same protocol otherwise).
# Using the RS05 ranges on an RS02 makes its torque read ~3x too small and its speeds ~12 % off.
MODELS = {
    "RS05": dict(p_max=4 * math.pi, v_max=50.0, t_max=5.5, kp_max=500.0, kd_max=5.0),
    "RS02": dict(p_max=4 * math.pi, v_max=44.0, t_max=17.0, kp_max=500.0, kd_max=5.0),
}

# ------------------------------------------------------------------ comm types
T_GET_ID, T_MIT, T_FEEDBACK, T_ENABLE, T_STOP = 0, 1, 2, 3, 4
T_SET_ZERO, T_SET_ID = 6, 7
T_READ, T_WRITE, T_FAULT, T_SAVE = 17, 18, 21, 22
T_SET_BAUD, T_ACTIVE_REPORT, T_SET_PROTOCOL = 23, 24, 25

# ------------------------------------------------------------------ run_mode (0x7005)
MODE_MIT, MODE_PP, MODE_SPEED, MODE_CURRENT, MODE_CSP = 0, 1, 2, 3, 5
RUN_MODE_NAMES = {0: "MIT/operation", 1: "PP position", 2: "speed", 3: "current", 5: "CSP position"}

# ------------------------------------------------------------------ parameters
# name: (index, struct format of the value in data[4..8])
PARAMS = {
    "run_mode":      (0x7005, "<B"),
    "iq_ref":        (0x7006, "<f"),
    "spd_ref":       (0x700A, "<f"),
    "limit_torque":  (0x700B, "<f"),
    "cur_kp":        (0x7010, "<f"),
    "cur_ki":        (0x7011, "<f"),
    "cur_filt_gain": (0x7014, "<f"),
    "loc_ref":       (0x7016, "<f"),
    "limit_spd":     (0x7017, "<f"),
    "limit_cur":     (0x7018, "<f"),
    "mechPos":       (0x7019, "<f"),
    "iqf":           (0x701A, "<f"),
    "mechVel":       (0x701B, "<f"),
    "VBUS":          (0x701C, "<f"),
    "loc_kp":        (0x701E, "<f"),
    "spd_kp":        (0x701F, "<f"),
    "spd_ki":        (0x7020, "<f"),
    "spd_filt_gain": (0x7021, "<f"),
    "acc_rad":       (0x7022, "<f"),
    "vel_max":       (0x7024, "<f"),
    "acc_set":       (0x7025, "<f"),
    "EPScan_time":   (0x7026, "<H"),
    "canTimeout":    (0x7028, "<I"),
    "zero_sta":      (0x7029, "<B"),
}
PARAM_BY_INDEX = {idx: (name, fmt) for name, (idx, fmt) in PARAMS.items()}

FAULT_BITS = {0: "undervoltage", 1: "overcurrent", 2: "overtemp",
              3: "mag-encoder", 4: "HALL/overload", 5: "uncalibrated"}
STATE_NAMES = {0: "reset", 1: "cali", 2: "run"}


def fault_str(bits: int) -> str:
    return ",".join(n for b, n in FAULT_BITS.items() if bits & (1 << b)) or "-"


# ------------------------------------------------------------------ scaling
def f2u(x: float, xmin: float, xmax: float, bits: int = 16) -> int:
    x = min(max(x, xmin), xmax)
    return int(round((x - xmin) * ((1 << bits) - 1) / (xmax - xmin)))


def u2f(u: int, xmin: float, xmax: float, bits: int = 16) -> float:
    return u * (xmax - xmin) / ((1 << bits) - 1) + xmin


def u2f_fb(u: int, xmin: float, xmax: float, bits: int = 16) -> float:
    """Decode a value the MOTOR encoded. The firmware truncates (int cast), so the code u stands for
    [u, u+1) -> take the middle. Plain u2f() reads a motor at rest as -0.0008 rad/s / -0.011 deg
    (half a step below zero) instead of 0. Verified on the real log: idle vel was always u=32767."""
    n = (1 << bits) - 1
    return min(xmax, (u + 0.5) * (xmax - xmin) / n + xmin)


# ------------------------------------------------------------------ data
@dataclass
class Feedback:
    t: float = 0.0          # time.monotonic() of reception
    pos: float = 0.0        # rad, relative to mechanical zero (+-4pi)
    vel: float = 0.0        # rad/s
    torque: float = 0.0     # Nm (estimated by the motor)
    temp: float = 0.0       # degC
    state: int = 0          # 0 reset, 1 cali, 2 run
    faults: int = 0         # 6 fault bits


@dataclass
class MotorState:
    fb: Optional[Feedback] = None
    fb_count: int = 0
    fb_times: deque = field(default_factory=lambda: deque(maxlen=100))
    iq: float = float("nan")      # A   (iqf, polled)
    vbus: float = float("nan")    # V   (VBUS, polled)
    fault_word: int = 0
    warn_word: int = 0

    @property
    def fb_hz(self) -> float:
        if len(self.fb_times) < 2:
            return 0.0
        dt = self.fb_times[-1] - self.fb_times[0]
        return (len(self.fb_times) - 1) / dt if dt > 0 else 0.0


INTERFACES = ("slcan", "gs_usb", "stm")      # + "sim" / "stm-sim" handled by the GUIs and CLIs


def open_bus(interface: str = "slcan", channel: str = "COM9", bitrate: int = 1_000_000) -> can.BusABC:
    """Direct to the motors through a CANable ('slcan' COM port / 'gs_usb' candleLight), or through the
    STM32 gateway firmware ('stm', channel = the STM's USB COM port, both CAN channels, see rs05_stm.py)."""
    if interface == "stm":
        from rs05_stm import StmBus
        return StmBus(channel, bitrate=bitrate)
    if interface == "slcan":
        return can.Bus(interface="slcan", channel=channel, bitrate=bitrate, sleep_after_open=2)
    if interface == "gs_usb":
        return can.Bus(interface="gs_usb", channel=channel, index=int(channel) if channel.isdigit() else 0,
                       bitrate=bitrate)
    return can.Bus(interface=interface, channel=channel, bitrate=bitrate)


# ------------------------------------------------------------------ deep scan (lost / unknown IDs)
# Private-protocol frames FROM a motor carry the motor's CAN ID in bits 15..8.
_MOTOR_TX_TYPES = {T_GET_ID: "get-ID reply", T_FEEDBACK: "feedback", T_READ: "param reply",
                   T_FAULT: "fault frame", T_ACTIVE_REPORT: "active report"}


@dataclass
class ScanResult:
    bitrate: Optional[int]
    motors: dict = field(default_factory=dict)      # can id -> set of how it was seen
    uid: dict = field(default_factory=dict)         # can id -> 8-byte MCU UID (get-ID reply data)
    other: dict = field(default_factory=dict)       # arbitration id -> count, frames that are not RS05 replies
    error: str = ""

    def text(self) -> str:
        br = f"{self.bitrate} bit/s" if self.bitrate else "bus"
        if self.error:
            return f"{br}: error: {self.error}"
        lines = []
        for cid in sorted(self.motors):
            uid = self.uid.get(cid)
            lines.append(f"{br}: motor ID 0x{cid:02X} ({cid})  via {', '.join(sorted(self.motors[cid]))}"
                         + (f"  UID {uid.hex()}" if uid else ""))
        if self.other:
            top = sorted(self.other.items(), key=lambda kv: -kv[1])[:8]
            ids = ", ".join(f"{('0x%08X' if a > 0x7FF else '0x%03X') % a} x{n}" for a, n in top)
            lines.append(f"{br}: other traffic (not RS05 private replies): {ids}"
                         + ("  - 11-bit IDs suggest a motor switched to MIT/CANopen protocol"
                            if any(a <= 0x7FF for a in self.other) else ""))
        return "\n".join(lines) or f"{br}: nothing answered"


def deep_scan(bus: can.BusABC, host: int = 0xFD, ids=range(0, 256), listen: float = 1.0,
              stop_probe: bool = True, bitrate: Optional[int] = None, progress=None) -> ScanResult:
    """Find motors whose ID is unknown. Nothing else may read `bus` meanwhile (own receive loop).

    1. listen passively (motors with active reporting on show up by themselves),
    2. send 'get ID' (type 0) to every ID 0..255 - not only 1..127,
    3. optionally send 'stop' (type 4) to every ID - every motor answers a stop with a feedback frame.
       This disables a running motor, so only use it while nothing should be moving.
    Every frame received is decoded; non-RS05 frames are listed in `other`.
    """
    res = ScanResult(bitrate=bitrate)
    done = threading.Event()

    def rx():
        while not done.is_set():
            try:
                m = bus.recv(timeout=0.05)
            except (can.CanError, OSError, ValueError):
                time.sleep(0.02)
                continue
            if m is None or m.is_error_frame:
                continue
            a = m.arbitration_id
            typ, src = (a >> 24) & 0x1F, (a >> 8) & 0xFF
            if m.is_extended_id and src == host:
                continue                                      # our own request (echo / other host)
            if m.is_extended_id and typ in _MOTOR_TX_TYPES:
                res.motors.setdefault(src, set()).add(_MOTOR_TX_TYPES[typ])
                if typ == T_GET_ID and len(m.data) == 8:
                    res.uid[src] = bytes(m.data)
            else:
                res.other[a] = res.other.get(a, 0) + 1

    th = threading.Thread(target=rx, daemon=True)
    th.start()
    ids = list(ids)
    total = len(ids) * (2 if stop_probe else 1)
    sent = 0

    def tx(arb, data=bytes(8)):
        nonlocal sent
        try:
            bus.send(can.Message(arbitration_id=arb, is_extended_id=True, data=data), timeout=0.1)
        except can.CanError as e:
            res.error = res.error or f"send failed ({e}) - wrong bitrate / no ACK?"
        sent += 1
        if progress and sent % 16 == 0:
            progress(sent / total)
        time.sleep(0.002)

    try:
        time.sleep(listen)
        for cid in ids:
            tx((T_GET_ID << 24) | (host << 8) | cid)
        time.sleep(0.3)
        if stop_probe:
            for cid in ids:
                if cid not in res.motors:
                    tx((T_STOP << 24) | (host << 8) | cid)
                else:
                    sent += 1
            time.sleep(0.3)
    finally:
        done.set()
        th.join(timeout=0.5)
    if res.motors:
        res.error = ""      # motors answered, so the send errors were transient
    return res


def deep_scan_bitrates(interface: str, channel: str, host: int = 0xFD,
                       bitrates=(1_000_000, 500_000, 250_000, 125_000), progress=None, **kw) -> list[ScanResult]:
    """deep_scan at every bitrate (a motor's baud rate can be changed with type 23).
    Opens and closes the CANable itself, so it must not be connected elsewhere."""
    out = []
    for i, br in enumerate(bitrates):
        try:
            bus = open_bus(interface, channel, br)
        except Exception as e:  # noqa: BLE001
            out.append(ScanResult(bitrate=br, error=f"cannot open: {e}"))
            continue
        try:
            p = (lambda f, i=i: progress((i + f) / len(bitrates), br)) if progress else None
            out.append(deep_scan(bus, host, bitrate=br, progress=p, **kw))
        finally:
            bus.shutdown()
    return out


# ------------------------------------------------------------------ several motors on one bus
class BusHub:
    """Several RS05 on ONE bus: a single receive thread hands every frame to the motor whose ID sent it,
    and one transmit lock serialises all sends (slcan is not thread safe).

        hub = BusHub(bus)
        j1 = RS05(bus, 1, hub=hub); j2 = RS05(bus, 2, hub=hub) ...

    It also counts frames per source ID (for ID / cabling checks) and keeps every get-ID reply
    with its MCU UID, so two motors sharing one ID show up as two UIDs on that ID."""

    def __init__(self, bus: can.BusABC):
        self.bus = bus
        self.tx_lock = threading.Lock()
        self.motors: list = []
        self.rx_count: dict[int, int] = {}          # source ID -> frames
        self.id_replies: list[tuple[int, bytes]] = []   # (source ID, UID) of get-ID replies
        self.error_frames = 0
        self.rx_errors = 0
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def attach(self, m):
        if m not in self.motors:
            self.motors.append(m)

    def detach(self, m):
        if m in self.motors:
            self.motors.remove(m)

    def _loop(self):
        while not self._stop.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError, ValueError):
                self.rx_errors += 1
                time.sleep(0.05)
                continue
            if msg is None:
                continue
            if msg.is_error_frame:
                self.error_frames += 1
                continue
            if not msg.is_extended_id:
                continue
            src = (msg.arbitration_id >> 8) & 0xFF
            typ = (msg.arbitration_id >> 24) & 0x1F
            self.rx_count[src] = self.rx_count.get(src, 0) + 1
            if typ == T_GET_ID:
                self.id_replies.append((src, bytes(msg.data)))
            for m in list(self.motors):
                if typ == T_GET_ID or m.id == src:
                    m._handle(msg)

    def close(self):
        self._stop.set()
        self._th.join(timeout=0.5)


# ------------------------------------------------------------------ motor
class RS05:
    """One RS05 on a python-can bus. A background thread decodes everything it receives
    (or, with hub=BusHub, the hub's thread does - required for several motors on one bus)."""

    def __init__(self, bus: can.BusABC, motor_id: int = 0x7F, host_id: int = 0xFD, hub: Optional[BusHub] = None,
                 model: str = "RS05"):
        self.bus = bus
        self.id = motor_id
        self.host = host_id
        self.hub = hub
        self.s = MotorState()
        self.seen_ids: set[int] = set()
        self.on_feedback = None            # optional callback(Feedback)
        self.tx_errors = 0
        self.set_model(model)
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._replies: dict[int, bytes] = {}
        self._stop = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None
        self._poll_stop = threading.Event()
        if hub:
            self._tx_lock = hub.tx_lock
            self._rx = None
            hub.attach(self)
        else:
            self._tx_lock = threading.Lock()
            self._rx = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx.start()

    def set_model(self, model: str):
        """'RS05' or 'RS02' - selects the scaling of MIT commands and feedback."""
        if model not in MODELS:
            raise ValueError(f"unknown motor model {model!r} (known: {', '.join(MODELS)})")
        self.model = model
        for k, v in MODELS[model].items():
            setattr(self, k, v)

    # -------------------------------------------------------------- TX
    def _send(self, comm_type: int, data2: int, data: bytes = bytes(8)):
        arb = ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (self.id & 0xFF)
        msg = can.Message(arbitration_id=arb, is_extended_id=True, data=bytes(data).ljust(8, b"\0"))
        try:
            with self._tx_lock:
                self.bus.send(msg, timeout=0.1)
        except can.CanError:
            self.tx_errors += 1
            raise

    def enable(self):
        self._send(T_ENABLE, self.host)

    def stop(self, clear_fault: bool = False):
        self._send(T_STOP, self.host, bytes([1 if clear_fault else 0]) + bytes(7))

    def set_zero(self):
        """Current shaft position becomes 0. Motor must be stopped."""
        self._send(T_SET_ZERO, self.host, bytes([1]) + bytes(7))

    def save_params(self):
        self._send(T_SAVE, self.host, bytes(range(1, 9)))

    def get_id(self, target: Optional[int] = None):
        arb = (T_GET_ID << 24) | (self.host << 8) | ((self.id if target is None else target) & 0xFF)
        with self._tx_lock:
            self.bus.send(can.Message(arbitration_id=arb, is_extended_id=True, data=bytes(8)), timeout=0.1)

    def ping(self, target: int, timeout: float = 0.2) -> bool:
        """True if a motor answers 'get ID' (type 0) on CAN ID `target`."""
        self.seen_ids.discard(target)
        self.get_id(target)
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if target in self.seen_ids:
                return True
            time.sleep(0.005)
        return False

    def scan(self, ids=range(1, 128), settle: float = 0.25) -> list[int]:
        self.seen_ids.clear()
        for cid in ids:
            self.get_id(cid)
            time.sleep(0.002)
        time.sleep(settle)
        return sorted(self.seen_ids)

    def set_can_id(self, new_id: int) -> bool:
        """Type 7: ID bits 23..16 = new ID, 15..8 = host, 7..0 = current ID. Motor must be stopped.
        Returns True when the motor answers on the new ID; self.id is switched to it then."""
        if not 1 <= new_id <= 127:
            raise ValueError("CAN ID must be 1..127")
        if new_id == self.host:
            raise ValueError("new ID equals the host ID")
        self.stop()
        time.sleep(0.05)
        self._send(T_SET_ID, ((new_id & 0xFF) << 8) | self.host)
        time.sleep(0.3)
        if self.ping(new_id, 0.5):
            self.id = new_id
            return True
        return False

    def active_report(self, on: bool):
        self._send(T_ACTIVE_REPORT, self.host, bytes([1, 2, 3, 4, 5, 6, 1 if on else 0, 0]))

    def mit(self, p: float, v: float, kp: float, kd: float, t: float):
        """Type 1 (operation control). Needs run_mode=0 and enable first. Motor computes
        torque = kp*(p - pos) + kd*(v - vel) + t  and answers with a type-2 frame."""
        data = struct.pack(">HHHH", f2u(p, -self.p_max, self.p_max), f2u(v, -self.v_max, self.v_max),
                           f2u(kp, 0.0, self.kp_max), f2u(kd, 0.0, self.kd_max))
        self._send(T_MIT, f2u(t, -self.t_max, self.t_max), data)

    # -------------------------------------------------------------- parameters
    @staticmethod
    def _param(name_or_index):
        if isinstance(name_or_index, str):
            return PARAMS[name_or_index]
        return name_or_index, PARAM_BY_INDEX.get(name_or_index, ("?", "<f"))[1]

    def write(self, name_or_index, value):
        idx, fmt = self._param(name_or_index)
        val = struct.pack(fmt, int(value) if fmt != "<f" else float(value)).ljust(4, b"\0")
        self._send(T_WRITE, self.host, struct.pack("<HH", idx, 0) + val)
        time.sleep(0.005)   # let the motor digest it; keeps writes ordered on slcan

    def request(self, name_or_index):
        """Fire-and-forget read; the reply lands in self.s (iqf / VBUS) or the reply table."""
        idx, _ = self._param(name_or_index)
        self._send(T_READ, self.host, struct.pack("<HH4x", idx, 0))

    def read(self, name_or_index, timeout: float = 0.3):
        idx, fmt = self._param(name_or_index)
        with self._cv:
            self._replies.pop(idx, None)
        self.request(idx)
        deadline = time.monotonic() + timeout
        with self._cv:
            while idx not in self._replies:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise TimeoutError(f"no reply for parameter 0x{idx:04X} (motor ID 0x{self.id:02X}?)")
                self._cv.wait(left)
            raw = self._replies.pop(idx)
        return struct.unpack_from(fmt, raw, 4)[0]

    # -------------------------------------------------------------- mode helpers
    def set_mode(self, run_mode: int):
        """run_mode may only change while stopped: stop -> run_mode -> enable."""
        self.stop()
        time.sleep(0.01)
        self.write("run_mode", run_mode)
        self.enable()
        time.sleep(0.01)

    def start_polling(self, hz: float = 20.0):
        """Poll iqf (phase current) and VBUS in the background -> current draw."""
        self.stop_polling()
        self._poll_stop.clear()

        def run():
            k = 0
            while not self._poll_stop.wait(1.0 / hz):
                try:
                    self.request("VBUS" if k % 10 == 5 else "iqf")
                except can.CanError:
                    pass
                k += 1

        self._poll_thread = threading.Thread(target=run, daemon=True)
        self._poll_thread.start()

    def stop_polling(self):
        self._poll_stop.set()
        if self._poll_thread:
            self._poll_thread.join(timeout=0.5)
            self._poll_thread = None

    def wait_feedback(self, timeout: float = 1.0) -> Optional[Feedback]:
        n0 = self.s.fb_count
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if self.s.fb_count != n0:
                return self.s.fb
            time.sleep(0.005)
        return None

    # -------------------------------------------------------------- RX
    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
            except (can.CanError, OSError, ValueError):
                time.sleep(0.05)
                continue
            if msg is not None:
                self._handle(msg)

    def _handle(self, msg: can.Message):
        if not msg.is_extended_id or msg.dlc < 8:
            return
        arb = msg.arbitration_id
        typ = (arb >> 24) & 0x1F
        src = (arb >> 8) & 0xFF
        d = bytes(msg.data)
        if typ == T_GET_ID:
            self.seen_ids.add(src)
            return
        if src != self.id:
            return
        if typ in (T_FEEDBACK, T_ACTIVE_REPORT):
            p, v, t, tmp = struct.unpack(">HHHH", d)
            fb = Feedback(t=time.monotonic(),
                          pos=u2f_fb(p, -self.p_max, self.p_max), vel=u2f_fb(v, -self.v_max, self.v_max),
                          torque=u2f_fb(t, -self.t_max, self.t_max), temp=tmp / 10.0,
                          state=(arb >> 22) & 0x03, faults=(arb >> 16) & 0x3F)
            with self._lock:
                self.s.fb = fb
                self.s.fb_count += 1
                self.s.fb_times.append(fb.t)
            if self.on_feedback:
                self.on_feedback(fb)
        elif typ == T_READ:
            idx = d[0] | (d[1] << 8)
            if idx == PARAMS["iqf"][0]:
                self.s.iq = struct.unpack_from("<f", d, 4)[0]
            elif idx == PARAMS["VBUS"][0]:
                self.s.vbus = struct.unpack_from("<f", d, 4)[0]
            with self._cv:
                self._replies[idx] = d
                self._cv.notify_all()
        elif typ == T_FAULT:
            self.s.fault_word, self.s.warn_word = struct.unpack("<II", d)

    def close(self):
        self.stop_polling()
        self._stop.set()
        if self.hub:
            self.hub.detach(self)
        elif self._rx:
            self._rx.join(timeout=0.5)
