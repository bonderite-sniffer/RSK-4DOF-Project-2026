"""
rs05_host.py - host library for the RS05 HAL firmware (protocol v2, 4 motors).

Frame:  A5 5A | type | len | payload | crc16(lo,hi)   CRC-16/CCITT-FALSE over type,len,payload
Payload little-endian. Must match Firmware/App/Inc/protocol.h.

Motors are numbered 1..4 here (M1, M2 on CAN1; M3, M4 on CAN2). Internally the
firmware uses index 0..3 and a bit mask (bit0 = M1).

    from rs05_host import RS05Link, mask
    with RS05Link("COM8") as L:
        print(L.ping())
        L.zero_here(mask(1, 2))
        L.pos(mask(1), math.radians(45), vel_lim=1.0, cur_lim=3.0)
        print(L.latest.m[0].pos_deg)
"""
from __future__ import annotations

import csv
import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

# ---------------------------------------------------------------- protocol ids
CMD_PING, CMD_ESTOP, CMD_HEARTBEAT, CMD_SET_CFG, CMD_SCAN, CMD_GET_HEALTH = 0x01, 0x02, 0x03, 0x05, 0x06, 0x07
CMD_DISABLE, CMD_POS, CMD_VEL, CMD_MIT = 0x11, 0x20, 0x21, 0x22
CMD_READ_PARAM, CMD_WRITE_PARAM, CMD_CALIB = 0x30, 0x31, 0x40
EVT_ACK, EVT_TELEM, EVT_SCAN, EVT_PARAM, EVT_EVENT, EVT_PONG, EVT_HEALTH = 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86

CAL_ZERO_HERE, CAL_HOME, CAL_SET_CAN_ID, CAL_SAVE = 1, 2, 3, 4
CFG = dict(TSCALE=1, TELEM_DIV=2, SOFTLIM_EN=3, POS_MIN=4, POS_MAX=5, TEMP_MAX=6,
           WD_MS=7, HOST_ID=10, SAFE_KP=11, SAFE_KD=12, MOTOR_ID0=16)

ACK_STATUS = {0: "OK", -1: "BAD_LEN", -2: "BUSY (motor not idle)", -4: "BAD_ARG",
              -5: "CAN_ERR", -6: "UNKNOWN_CMD"}
EVENT_NAMES = {1: "HOME_DONE", 2: "HOME_FAIL", 3: "SOFTLIMIT", 4: "OVERTEMP", 5: "FAULT",
               6: "OFFLINE", 7: "WATCHDOG", 8: "ESTOP", 9: "CAN_ERROR", 10: "HOME_STALL",
               11: "ZERO_DONE", 12: "ZERO_FAIL", 13: "ID_CHANGED", 14: "ID_FAIL"}
MODE_NAMES = {0: "IDLE", 1: "MIT", 2: "VEL", 3: "POS", 4: "CALIB"}
FAULT_BITS = {0: "undervoltage", 1: "overcurrent", 2: "overtemp", 3: "mag-encoder",
              4: "HALL/encoder", 5: "uncalibrated"}

P = dict(run_mode=0x7005, iq_ref=0x7006, spd_ref=0x700A, limit_torque=0x700B, loc_ref=0x7016,
         limit_spd=0x7017, limit_cur=0x7018, mechPos=0x7019, iqf=0x701A, mechVel=0x701B,
         VBUS=0x701C, acc_rad=0x7022, zero_sta=0x7029)

TELEM_HDR = struct.Struct("<IHBB")
TELEM_MOT = struct.Struct("<BBBB6f")      # 28 bytes
HEALTH_BUS = struct.Struct("<BBBB7I")     # 32 bytes
PONG = struct.Struct("<IIBBBB")
RPM_PER_RADS = 60.0 / (2 * math.pi)
N_MOTORS = 4


def mask(*motors: int) -> int:
    """mask(1, 3) -> 0b0101. Motors numbered 1..4."""
    m = 0
    for k in motors:
        if not 1 <= k <= N_MOTORS:
            raise ValueError(f"motor {k} out of range 1..{N_MOTORS}")
        m |= 1 << (k - 1)
    return m


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(ftype: int, payload: bytes = b"") -> bytes:
    body = bytes([ftype, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return b"\xA5\x5A" + body + bytes([crc & 0xFF, crc >> 8])


class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        while True:
            i = self.buf.find(b"\xA5\x5A")
            if i < 0:
                del self.buf[:-1]
                return
            if i:
                del self.buf[:i]
            if len(self.buf) < 4:
                return
            ln = self.buf[3]
            if ln > 128:
                del self.buf[:2]
                continue
            if len(self.buf) < 6 + ln:
                return
            body = bytes(self.buf[2:4 + ln])
            crc = self.buf[4 + ln] | (self.buf[5 + ln] << 8)
            if crc16_ccitt(body) == crc:
                del self.buf[:6 + ln]
                yield body[0], body[2:]
            else:
                self.crc_errors += 1
                del self.buf[:2]


@dataclass
class MotorTelem:
    mode: int
    rs_mode: int
    fault: int
    flags: int
    pos: float        # rad, relative to zero
    vel: float        # rad/s
    torque: float     # Nm
    temp: float       # degC
    iq: float         # A
    sp: float         # active setpoint

    @property
    def online(self) -> bool:
        return bool(self.flags & 1)

    @property
    def pos_deg(self) -> float:
        return math.degrees(self.pos)

    @property
    def rpm(self) -> float:
        return self.vel * RPM_PER_RADS


@dataclass
class Telem:
    t_ms: int
    seq: int
    host_ok: int
    m: list = field(default_factory=list)
    host_t: float = 0.0
    tag: str = ""


@dataclass
class Health:
    t_ms: int
    bus: list        # list of dicts per bus
    vbus: list       # V per motor
    fb_age_ms: list  # ms since last feedback per motor


def decode_telem(pl: bytes) -> Telem:
    t_ms, seq, host_ok, n = TELEM_HDR.unpack_from(pl, 0)
    ms = [MotorTelem(*TELEM_MOT.unpack_from(pl, TELEM_HDR.size + i * TELEM_MOT.size)) for i in range(4)]
    return Telem(t_ms, seq, host_ok, ms[:n])


def decode_health(pl: bytes) -> Health:
    t_ms, = struct.unpack_from("<I", pl, 0)
    keys = ("init_ok", "eflg", "tec", "rec", "tx_sent", "tx_fail", "tx_busy", "rx_count", "rx_ovf", "bus_off", "spi_err")
    buses = [dict(zip(keys, HEALTH_BUS.unpack_from(pl, 4 + b * 32))) for b in range(2)]
    vbus = list(struct.unpack_from("<4f", pl, 68))
    age = list(struct.unpack_from("<4H", pl, 84))
    return Health(t_ms, buses, vbus, age)


def fault_str(bits: int) -> str:
    return ",".join(n for b, n in FAULT_BITS.items() if bits & (1 << b)) or "-"


# ---------------------------------------------------------------- link
class RS05Link:
    def __init__(self, port: str, baud: int = 921600, heartbeat_hz: float = 20.0,
                 verbose_events: bool = True):
        import serial  # pyserial (lazy import so the codec works without it)
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.parser = FrameParser()
        self.latest: Optional[Telem] = None
        self.health: Optional[Health] = None
        self.history: deque = deque(maxlen=200_000)
        self.events: deque = deque(maxlen=2000)
        self.tag = ""
        self.on_telem: Optional[Callable[[Telem], None]] = None
        self.verbose_events = verbose_events
        self.n_motors = N_MOTORS
        self._acks: dict = {}
        self._waiters: dict = {}
        self._tx_lock = threading.Lock()
        self._csv = None
        self._csv_writer = None
        self._run = True
        self._hb_period = 1.0 / heartbeat_hz
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._hb_loop, daemon=True).start()

    # -------------------------------------------------- lifecycle
    def close(self):
        try:
            self.estop()
        except Exception:
            pass
        self._run = False
        time.sleep(0.1)
        self.stop_log()
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # -------------------------------------------------- io
    def _send(self, ftype: int, payload: bytes = b""):
        with self._tx_lock:
            self.ser.write(encode_frame(ftype, payload))

    def _hb_loop(self):
        while self._run:
            try:
                self._send(CMD_HEARTBEAT)
            except Exception:
                pass
            time.sleep(self._hb_period)

    def _rx_loop(self):
        while self._run:
            try:
                data = self.ser.read(4096)
            except Exception:
                break
            if data:
                for ftype, pl in self.parser.feed(data):
                    self._dispatch(ftype, pl)

    def _dispatch(self, ftype: int, pl: bytes):
        if ftype == EVT_TELEM and len(pl) == 120:
            t = decode_telem(pl)
            t.host_t = time.time()
            t.tag = self.tag
            self.latest = t
            self.history.append(t)
            if self._csv_writer:
                self._write_row(t)
            if self.on_telem:
                self.on_telem(t)
            return
        if ftype == EVT_HEALTH and len(pl) == 92:
            self.health = decode_health(pl)
        elif ftype == EVT_ACK:
            cmd, st = struct.unpack("<Bb", pl)
            w = self._acks.pop(cmd, None)
            if w:
                w[1].append(st)
                w[0].set()
            elif st != 0:
                print(f"[ack] cmd 0x{cmd:02X}: {ACK_STATUS.get(st, st)}")
            return
        elif ftype == EVT_EVENT:
            m, code, val = struct.unpack("<BBf", pl)
            self.events.append((time.time(), m, code, val))
            if self.verbose_events:
                who = "ALL" if m == 0xFF else f"M{m + 1}"
                print(f"[event] {who} {EVENT_NAMES.get(code, code)} value={val:.3f}")
        w = self._waiters.pop(ftype, None)
        if w:
            w[1].append(pl)
            w[0].set()

    def _request(self, ftype: int, payload: bytes = b"", timeout: float = 1.0,
                 expect: Optional[int] = None):
        ev, box = threading.Event(), []
        table, key = (self._waiters, expect) if expect else (self._acks, ftype)
        table[key] = (ev, box)
        self._send(ftype, payload)
        if not ev.wait(timeout):
            table.pop(key, None)
            raise TimeoutError(f"no reply to cmd 0x{ftype:02X} - firmware running? right COM port?")
        res = box[0]
        if not expect and res != 0:
            raise RuntimeError(f"cmd 0x{ftype:02X} rejected: {ACK_STATUS.get(res, res)}")
        return res

    def wait_event(self, code: int, motor: Optional[int] = None, timeout: float = 10.0,
                   since: Optional[float] = None):
        """Wait for an event (motor 1..4 or None for any). Returns (motor, value) or None."""
        t0 = time.time()
        since = since if since is not None else t0
        while time.time() - t0 < timeout:
            for (t, m, c, v) in list(self.events):
                if t >= since and c == code and (motor is None or m == motor - 1):
                    return m + 1, v
            time.sleep(0.01)
        return None

    # -------------------------------------------------- commands
    def ping(self):
        g = PONG.unpack(self._request(CMD_PING, expect=EVT_PONG))
        ver, hz, n_motors, n_bus, proto, _ = g
        self.n_motors = n_motors
        return dict(fw=f"{ver >> 16}.{(ver >> 8) & 0xFF}.{ver & 0xFF}", ctrl_hz=hz,
                    n_motors=n_motors, n_bus=n_bus, proto=proto)

    def estop(self):
        self._send(CMD_ESTOP)

    def set_cfg(self, key: str, value: float):
        self._request(CMD_SET_CFG, struct.pack("<Bf", CFG[key], float(value)))

    def set_motor_id(self, motor: int, can_id: int):
        """Tell the firmware which CAN ID motor 1..4 has (does NOT change the motor)."""
        self._request(CMD_SET_CFG, struct.pack("<Bf", CFG["MOTOR_ID0"] + motor - 1, float(can_id)))

    def scan(self, bus_mask: int = 3) -> dict:
        out = {}
        for b in range(2):
            if bus_mask & (1 << b):
                pl = self._request(CMD_SCAN, struct.pack("<B", 1 << b), timeout=3.0, expect=EVT_SCAN)
                out[f"CAN{pl[0] + 1}"] = {"ids": list(pl[3:3 + pl[2]]), "bus_error": bool(pl[1])}
        return out

    def get_health(self) -> Health:
        return decode_health(self._request(CMD_GET_HEALTH, expect=EVT_HEALTH))

    def disable(self, m: int = 0x0F, clear_fault: bool = False):
        self._request(CMD_DISABLE, struct.pack("<BB", m, int(clear_fault)))

    def pos(self, m: int, target: float, vel_lim: float = 1.0, cur_lim: float = 3.0):
        self._request(CMD_POS, struct.pack("<Bfff", m, target, vel_lim, cur_lim))

    def vel(self, m: int, v: float, cur_lim: float = 5.0, acc: float = 10.0):
        self._request(CMD_VEL, struct.pack("<Bfff", m, v, cur_lim, acc))

    def mit(self, m: int, p: float = 0.0, v: float = 0.0, kp: float = 0.0, kd: float = 0.0, t: float = 0.0):
        self._request(CMD_MIT, struct.pack("<Bfffff", m, p, v, kp, kd, t))

    def calib(self, op: int, m: int, a: float = 0, b: float = 0, c: float = 0, d: float = 0):
        self._request(CMD_CALIB, struct.pack("<BBffff", op, m, a, b, c, d))

    def zero_here(self, m: int, timeout: float = 2.0) -> dict:
        """Current pose of the motors in mask m becomes 0. Returns {motor: ok}."""
        t0 = time.time()
        self.calib(CAL_ZERO_HERE, m)
        return {k: self.wait_event(11, k, timeout, t0) is not None
                for k in range(1, 5) if m & (1 << (k - 1))}

    def home(self, m: int, direction: int, speed: float = 0.5, cur_lim: float = 1.5, backoff: float = 0.2):
        self.calib(CAL_HOME, m, float(direction), speed, cur_lim, backoff)

    def set_can_id(self, motor: int, new_id: int, timeout: float = 2.0) -> bool:
        """Change the CAN ID of ONE motor (1..4). Only that motor's ID may be on the bus as the old one."""
        t0 = time.time()
        self.calib(CAL_SET_CAN_ID, mask(motor), float(new_id))
        return self.wait_event(13, motor, timeout, t0) is not None

    def save_params(self, m: int):
        self.calib(CAL_SAVE, m)

    def read_param(self, motor: int, index: int, as_float: bool = True):
        pl = self._request(CMD_READ_PARAM, struct.pack("<BH", motor - 1, index), expect=EVT_PARAM)
        _m, idx, ok, raw = struct.unpack("<BHBI", pl)
        if not ok:
            raise RuntimeError(f"param 0x{index:04X} read failed")
        return struct.unpack("<f", struct.pack("<I", raw))[0] if as_float else raw

    def write_param(self, motor: int, index: int, value, is_float: bool = True):
        raw = struct.unpack("<I", struct.pack("<f", value))[0] if is_float else int(value)
        self._request(CMD_WRITE_PARAM, struct.pack("<BHBI", motor - 1, index, int(is_float), raw))

    # -------------------------------------------------- helpers
    def wait_telem(self, timeout: float = 1.0) -> Telem:
        t0 = time.time()
        seq = self.latest.seq if self.latest else None
        while time.time() - t0 < timeout:
            if self.latest and self.latest.seq != seq:
                return self.latest
            time.sleep(0.002)
        raise TimeoutError("no telemetry")

    def collect(self, duration: float) -> list:
        t0 = time.time()
        time.sleep(duration)
        return [t for t in list(self.history) if t.host_t >= t0]

    # -------------------------------------------------- csv
    COLS = ["host_t", "t_ms", "seq", "tag"] + [
        f"m{i}_{k}" for i in range(1, 5) for k in
        ("mode", "fault", "flags", "pos", "vel", "torque", "temp", "iq", "sp")]

    def start_log(self, path: str):
        self.stop_log()
        self._csv = open(path, "w", newline="")
        self._csv_writer = csv.writer(self._csv)
        self._csv_writer.writerow(self.COLS)

    def _write_row(self, t: Telem):
        row = [f"{t.host_t:.4f}", t.t_ms, t.seq, t.tag]
        for m in t.m:
            row += [m.mode, m.fault, m.flags, f"{m.pos:.5f}", f"{m.vel:.4f}", f"{m.torque:.4f}",
                    f"{m.temp:.1f}", f"{m.iq:.4f}", f"{m.sp:.4f}"]
        self._csv_writer.writerow(row)

    def stop_log(self):
        if self._csv:
            w, self._csv_writer = self._csv_writer, None
            self._csv.close()
            self._csv = None
