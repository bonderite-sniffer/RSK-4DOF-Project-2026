"""RobStride RS05 terminal controller + live telemetry.

Default interface: CANable V2 running SLCAN on Windows (COM9).
Private protocol is the primary, tested path. MIT mode uses the common
RobStride/Mini-Cheetah 8-byte MIT packing and should be verified against the
exact motor firmware/manual before applying torque.

This program deliberately uses conservative software limits for first tests.
It is not a safety-rated controller or emergency-stop system.
"""

from __future__ import annotations

import csv
import math
import struct
import threading
import time
import msvcrt  # Windows console key polling for live dashboard
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import can

# ----------------------------- USER CONFIG -----------------------------
PORT = "COM9"
MOTOR_ID = 0x7F
HOST_ID = 0xFD
START_BITRATE = 1_000_000
CSV_PATH = Path("logs/rs05_telemetry.csv")

# RS05 ranges used by the RobStride sample/protocol references.
P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -50.0, 50.0
T_MIN, T_MAX = -5.5, 5.5
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

# Conservative first-motion software caps. Increase only after validation.
SAFE_CURRENT_A = 11.0
SAFE_SPEED_RAD_S = 44.0
SAFE_ACCEL_RAD_S2 = 5.0
SAFE_POSITION_STEP_RAD = 0.25
SAFE_MIT_SPEED_RAD_S = 0.5
SAFE_MIT_TORQUE_NM = 0.5
SAFE_MIT_KP = 20.0
SAFE_MIT_KD = 1.0
MIT_STREAM_HZ = 50.0

BAUD_OPTIONS = {
    "1": (1_000_000, 0x01, "1 Mbps"),
    "2": (500_000, 0x02, "500 kbps"),
    "3": (250_000, 0x03, "250 kbps"),
    "4": (125_000, 0x04, "125 kbps"),
}

# Private-protocol parameter addresses
PARAM_RUN_MODE = 0x7005
PARAM_SPEED_REF = 0x700A
PARAM_POSITION_REF = 0x7016
PARAM_VELOCITY_LIMIT = 0x7017
PARAM_CURRENT_LIMIT = 0x7018
PARAM_VELOCITY_ACCEL = 0x7022
PARAM_PP_VELOCITY_MAX = 0x7024
PARAM_PP_ACCEL = 0x7025

# Private communication types
TYPE_OPERATION_STATUS = 2
TYPE_ENABLE = 3
TYPE_DISABLE = 4
TYPE_WRITE_PARAMETER = 18
TYPE_SET_BAUDRATE = 23
TYPE_ACTIVE_REPORT = 24
TYPE_SET_PROTOCOL = 25

# ----------------------------- UTILITIES ------------------------------
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def float_to_uint(value: float, vmin: float, vmax: float, bits: int) -> int:
    value = clamp(value, vmin, vmax)
    span = vmax - vmin
    return int((value - vmin) * ((1 << bits) - 1) / span)


def uint_to_float(value: int, vmin: float, vmax: float, bits: int) -> float:
    return value * (vmax - vmin) / ((1 << bits) - 1) + vmin


def private_id(comm_type: int) -> int:
    return ((comm_type & 0x1F) << 24) | ((HOST_ID & 0xFF) << 8) | (MOTOR_ID & 0xFF)


def pack_private_float(parameter: int, value: float) -> bytes:
    # Parameter index and float32 are little-endian in private protocol.
    return struct.pack("<HHf", parameter, 0, float(value))


def pack_private_u8(parameter: int, value: int) -> bytes:
    return struct.pack("<H H B 3x", parameter, 0, value & 0xFF)


def pack_mit_command(position: float, velocity: float, kp: float,
                     kd: float, torque: float) -> bytes:
    """Classic 16/12/12/12/12-bit MIT command packing."""
    p = float_to_uint(position, P_MIN, P_MAX, 16)
    v = float_to_uint(velocity, V_MIN, V_MAX, 12)
    kp_i = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_i = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t = float_to_uint(torque, T_MIN, T_MAX, 12)
    return bytes([
        (p >> 8) & 0xFF,
        p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0x0F) << 4) | ((kp_i >> 8) & 0x0F),
        kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0x0F) << 4) | ((t >> 8) & 0x0F),
        t & 0xFF,
    ])


@dataclass
class Telemetry:
    timestamp: str = ""
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    mode: int = 0
    faults: int = 0
    can_id: int = 0
    frame_type: str = ""


class RS05Console:
    def __init__(self, port: str, bitrate: int):
        self.port = port
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None
        self.rx_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest: Optional[Telemetry] = None
        self.timestamps = deque(maxlen=200)
        self.frame_count = 0
        self.rx_errors = 0
        self.csv_file = None
        self.csv_writer = None
        self.protocol = "private"
        self.enabled = False
        self._mit_thread: Optional[threading.Thread] = None
        self._mit_stop = threading.Event()
        self._mit_lock = threading.Lock()
        self._mit_target = (0.0, 0.0, 0.0, 0.0, 0.0)
        self._param_condition = threading.Condition()
        self._param_responses: dict[int, bytes] = {}

    # -------------------------- CAN interface --------------------------
    def connect(self):
        self.bus = can.Bus(
            interface="slcan",
            channel=self.port,
            bitrate=self.bitrate,
            sleep_after_open=2,
        )
        CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = CSV_PATH.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp", "elapsed_s", "protocol", "can_id", "frame_type",
            "position_rad", "velocity_rad_s", "torque_Nm", "temperature_C",
            "mode", "faults", "feedback_hz"
        ])
        self.start_time = time.monotonic()
        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.rx_thread.start()
        print(f"Connected: {self.port} @ {self.bitrate} bit/s")
        print("Telemetry CSV:", CSV_PATH.resolve())
        if self.protocol == "private":
            time.sleep(0.15)
            self.enable_active_reporting()

    def send(self, arbitration_id: int, data: bytes | list[int], extended: bool):
        if self.bus is None:
            raise RuntimeError("CAN bus is not connected")
        payload = bytes(data)
        if len(payload) != 8:
            raise ValueError("CAN payload must be exactly 8 bytes")
        msg = can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=extended,
            data=payload,
            check=True,
        )
        self.bus.send(msg, timeout=0.2)

    def send_private(self, comm_type: int, data: bytes | list[int] = bytes(8)):
        self.send(private_id(comm_type), data, extended=True)

    def write_u8(self, parameter: int, value: int):
        self.send_private(TYPE_WRITE_PARAMETER, pack_private_u8(parameter, value))
        time.sleep(0.03)

    def write_float(self, parameter: int, value: float):
        self.send_private(TYPE_WRITE_PARAMETER, pack_private_float(parameter, value))
        time.sleep(0.03)

    def read_parameter_raw(self, parameter: int, timeout: float = 0.5) -> bytes:
        """Read one private-protocol parameter and return its 8-byte response data."""
        with self._param_condition:
            self._param_responses.pop(parameter, None)
        request = struct.pack("<HH4x", parameter, 0)
        self.send_private(17, request)
        deadline = time.monotonic() + timeout
        with self._param_condition:
            while time.monotonic() < deadline:
                raw = self._param_responses.pop(parameter, None)
                if raw is not None:
                    return raw
                self._param_condition.wait(max(0.0, deadline - time.monotonic()))
        raise TimeoutError(f"No readback for parameter 0x{parameter:04X}; check motor response / CAN ID")

    def read_parameter_float(self, parameter: int) -> float:
        raw = self.read_parameter_raw(parameter)
        return struct.unpack("<f", raw[4:8])[0]

    def read_parameter_u8(self, parameter: int) -> int:
        raw = self.read_parameter_raw(parameter)
        return raw[4]

    def enable_active_reporting(self):
        # Type 24: 01 02 03 04 05 06 F_CMD, F_CMD=01 enables reporting.
        self.send_private(TYPE_ACTIVE_REPORT, bytes([1, 2, 3, 4, 5, 6, 1, 0]))
        print("Requested private-protocol active reporting.")

    # ----------------------------- RX decode ----------------------------
    def _decode_private(self, msg: can.Message) -> Optional[Telemetry]:
        if not msg.is_extended_id or msg.dlc != 8:
            return None
        cid = msg.arbitration_id
        comm_type = (cid >> 24) & 0x1F
        motor = (cid >> 8) & 0xFF
        host = cid & 0xFF
        if comm_type not in (2, 24) or motor != MOTOR_ID or host != HOST_ID:
            return None
        d = msg.data
        rp = int.from_bytes(d[0:2], "big")
        rv = int.from_bytes(d[2:4], "big")
        rt = int.from_bytes(d[4:6], "big")
        temp = int.from_bytes(d[6:8], "big")
        mode = (cid >> 22) & 0x03
        faults = (cid >> 16) & 0x3F
        return Telemetry(
            timestamp=datetime.now().isoformat(timespec="milliseconds"),
            position=uint_to_float(rp, P_MIN, P_MAX, 16),
            velocity=uint_to_float(rv, V_MIN, V_MAX, 16),
            torque=uint_to_float(rt, T_MIN, T_MAX, 16),
            temperature=temp / 10.0,
            mode=mode,
            faults=faults,
            can_id=cid,
            frame_type=f"private-type-{comm_type}",
        )

    def _decode_mit(self, msg: can.Message) -> Optional[Telemetry]:
        # MIT feedback is commonly a standard frame addressed to host ID;
        # confirm exact feedback-ID semantics against the RS05 firmware manual.
        if msg.is_extended_id or msg.dlc != 8 or msg.arbitration_id != HOST_ID:
            return None
        d = msg.data
        if d[0] != MOTOR_ID:
            return None
        p = (d[1] << 8) | d[2]
        v = (d[3] << 4) | (d[4] >> 4)
        t = ((d[4] & 0x0F) << 8) | d[5]
        temp = (d[6] << 8) | d[7]
        return Telemetry(
            timestamp=datetime.now().isoformat(timespec="milliseconds"),
            position=uint_to_float(p, P_MIN, P_MAX, 16),
            velocity=uint_to_float(v, V_MIN, V_MAX, 12),
            torque=uint_to_float(t, T_MIN, T_MAX, 12),
            temperature=temp / 10.0,
            can_id=msg.arbitration_id,
            frame_type="mit-feedback",
        )

    def _receive_loop(self):
        while not self.stop_event.is_set():
            try:
                msg = self.bus.recv(timeout=0.1) if self.bus else None
                if msg is None:
                    continue
                # Parameter-read responses (communication type 17) are routed
                # to synchronous readback waiters, not the telemetry decoder.
                if self.protocol == "private" and msg.is_extended_id and msg.dlc == 8:
                    cid = msg.arbitration_id
                    comm_type = (cid >> 24) & 0x1F
                    motor = (cid >> 8) & 0xFF
                    host = cid & 0xFF
                    if comm_type == 17 and motor == MOTOR_ID and host == HOST_ID:
                        parameter = int.from_bytes(msg.data[0:2], "little")
                        with self._param_condition:
                            self._param_responses[parameter] = bytes(msg.data)
                            self._param_condition.notify_all()
                        continue
                telem = self._decode_private(msg) if self.protocol == "private" else self._decode_mit(msg)
                if telem is None:
                    continue
                now = time.monotonic()
                with self.lock:
                    self.latest = telem
                    self.timestamps.append(now)
                    self.frame_count += 1
                    hz = 0.0
                    if len(self.timestamps) >= 2:
                        dt = self.timestamps[-1] - self.timestamps[0]
                        if dt > 0:
                            hz = (len(self.timestamps) - 1) / dt
                    elapsed = now - self.start_time
                    if self.csv_writer:
                        self.csv_writer.writerow([
                            telem.timestamp, f"{elapsed:.4f}", self.protocol,
                            f"0x{telem.can_id:X}", telem.frame_type,
                            f"{telem.position:.7f}", f"{telem.velocity:.7f}",
                            f"{telem.torque:.7f}", f"{telem.temperature:.2f}",
                            telem.mode, telem.faults, f"{hz:.2f}"
                        ])
                        if self.frame_count % 25 == 0:
                            self.csv_file.flush()
            except (can.CanError, OSError):
                self.rx_errors += 1
                time.sleep(0.05)

    def telemetry_snapshot(self):
        with self.lock:
            t = self.latest
            hz = 0.0
            if len(self.timestamps) >= 2:
                dt = self.timestamps[-1] - self.timestamps[0]
                if dt > 0:
                    hz = (len(self.timestamps) - 1) / dt
            return t, hz, self.frame_count

    def monitor_snapshot(self):
        t, hz, count = self.telemetry_snapshot()
        print("\n--- TELEMETRY SNAPSHOT ---")
        print(f"Frames: {count} | Feedback: {hz:.1f} Hz | Protocol: {self.protocol}")
        if t is None:
            print("No decoded feedback yet.")
            return
        print(f"Position    : {t.position:+.5f} rad")
        print(f"Velocity    : {t.velocity:+.5f} rad/s")
        print(f"Torque      : {t.torque:+.5f} Nm")
        print(f"Temperature : {t.temperature:.1f} C")
        if self.protocol == "private":
            print(f"Mode bits   : {t.mode} (0=reset, 1=calibration, 2=run)")
            print(f"Fault bits  : 0x{t.faults:02X}")

    def monitor_live(self):
        """Continuously refresh telemetry; press Q to return to the menu."""
        print("\nLIVE TELEMETRY — press Q to return")
        time.sleep(0.25)
        while True:
            t, hz, count = self.telemetry_snapshot()
            print("\033[2J\033[H", end="")
            print("========== ROBSTRIDE RS05 LIVE TELEMETRY ==========")
            print(f"Protocol : {self.protocol.upper()}    CAN : {self.port} @ {self.bitrate}")
            print(f"Frames   : {count:<10} Feedback rate: {hz:7.2f} Hz")
            print("---------------------------------------------------")
            if t is None:
                print("Waiting for decoded feedback...")
            else:
                print(f"Position    : {t.position:+.6f} rad")
                print(f"Velocity    : {t.velocity:+.6f} rad/s")
                print(f"Torque      : {t.torque:+.5f} Nm")
                print(f"Temperature : {t.temperature:.1f} C")
                if self.protocol == "private":
                    print(f"Mode bits   : {t.mode} (0=reset, 2=run)")
                    print(f"Fault bits  : 0x{t.faults:02X}")
            print("---------------------------------------------------")
            print("CSV: logs/rs05_telemetry.csv")
            print("Press Q to return to menu.")
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key.lower() == "q":
                    break
            time.sleep(0.1)
        print("\033[2J\033[H", end="")

    # -------------------------- Private control -------------------------
    def private_disable(self):
        self.send_private(TYPE_DISABLE, bytes(8))
        self.enabled = False
        time.sleep(0.05)
        print("Private-protocol disable/stop frame sent.")

    def private_enable(self):
        self.send_private(TYPE_ENABLE, bytes(8))
        self.enabled = True
        time.sleep(0.05)
        print("Private-protocol enable frame sent.")

    def private_speed(self, speed: float, current_limit: float, acceleration: float):
        speed = clamp(speed, -SAFE_SPEED_RAD_S, SAFE_SPEED_RAD_S)
        current_limit = clamp(current_limit, 0.05, SAFE_CURRENT_A)
        acceleration = clamp(acceleration, 0.05, SAFE_ACCEL_RAD_S2)
        self.private_disable()
        self.write_u8(PARAM_RUN_MODE, 2)
        self.private_enable()
        self.write_float(PARAM_CURRENT_LIMIT, current_limit)
        self.write_float(PARAM_VELOCITY_ACCEL, acceleration)
        self.write_float(PARAM_SPEED_REF, speed)
        print(f"Speed target set: {speed:+.3f} rad/s | I-limit {current_limit:.2f} A | accel {acceleration:.2f} rad/s^2")

    def private_position(self, target: float, speed_limit: float,
                         acceleration: float, current_limit: float):
        # Use the motor's float32 mechPos parameter for the safety delta check,
        # rather than relying on a potentially unvalidated packed-feedback scale.
        current_position = self.read_parameter_float(0x7019)
        delta = target - current_position
        if abs(delta) > SAFE_POSITION_STEP_RAD:
            raise ValueError(
                f"Target is {delta:+.3f} rad from motor mechPos "
                f"({current_position:+.3f} rad); first-test limit is "
                f"+/-{SAFE_POSITION_STEP_RAD:.2f} rad"
            )
        target = clamp(target, P_MIN, P_MAX)
        speed_limit = clamp(speed_limit, 0.05, SAFE_SPEED_RAD_S)
        acceleration = clamp(acceleration, 0.05, SAFE_ACCEL_RAD_S2)
        current_limit = clamp(current_limit, 0.05, SAFE_CURRENT_A)

        # Manual sequence: disable -> set run_mode=PP -> enable -> set PP limits -> target.
        self.private_disable()
        self.write_u8(PARAM_RUN_MODE, 1)
        self.private_enable()
        self.write_float(PARAM_CURRENT_LIMIT, current_limit)
        self.write_float(PARAM_PP_VELOCITY_MAX, speed_limit)
        self.write_float(PARAM_PP_ACCEL, acceleration)
        self.write_float(PARAM_POSITION_REF, target)
        time.sleep(0.1)

        # Read back the key settings so a rejected/incorrect write is visible.
        try:
            actual_mode = self.read_parameter_u8(PARAM_RUN_MODE)
            actual_target = self.read_parameter_float(PARAM_POSITION_REF)
            actual_vmax = self.read_parameter_float(PARAM_PP_VELOCITY_MAX)
            actual_acc = self.read_parameter_float(PARAM_PP_ACCEL)
            actual_current = self.read_parameter_float(PARAM_CURRENT_LIMIT)
            print("\nPP parameter readback:")
            print(f" run_mode  : {actual_mode} (expected 1)")
            print(f" loc_ref   : {actual_target:+.5f} rad")
            print(f" vel_max   : {actual_vmax:.3f} rad/s")
            print(f" acc_set   : {actual_acc:.3f} rad/s^2")
            print(f" limit_cur : {actual_current:.3f} A")
            if actual_mode != 1 or abs(actual_target - target) > 1e-3:
                raise RuntimeError("PP parameter readback mismatch")
        except Exception:
            self.private_disable()
            raise RuntimeError("PP readback failed/mismatched; motor disable sent. Do not assume motion command was accepted.")
        print(f"PP target accepted: {target:+.4f} rad (delta {delta:+.4f} rad)")

    # --------------------------- MIT control -----------------------------
    def switch_to_mit(self):
        print("\nThis changes the motor protocol and requires a motor power cycle.")
        print("After power cycle, restart this program and select MIT protocol at startup.")
        confirm = input("Type SWITCH-MIT to transmit protocol change: ").strip()
        if confirm != "SWITCH-MIT":
            print("Cancelled.")
            return
        payload = bytes([1, 2, 3, 4, 5, 6, 2, 0])
        self.send_private(TYPE_SET_PROTOCOL, payload)
        print("MIT protocol selection frame sent. Power OFF motor, wait, then power ON.")

    def mit_enable(self):
        # MIT protocol: standard 11-bit ID, data tail FC enables.
        self.send(MOTOR_ID, bytes([0xFF] * 7 + [0xFC]), extended=False)
        self.enabled = True
        time.sleep(0.05)
        print("MIT enable frame sent.")

    def mit_disable(self):
        self._stop_mit_stream()
        self.send(MOTOR_ID, bytes([0xFF] * 7 + [0xFD]), extended=False)
        self.enabled = False
        print("MIT disable frame sent.")

    def _start_mit_stream(self, target: tuple[float, float, float, float, float]):
        with self._mit_lock:
            self._mit_target = target
        self._mit_stop.clear()

        def worker():
            period = 1.0 / MIT_STREAM_HZ
            next_t = time.monotonic()
            while not self._mit_stop.is_set():
                with self._mit_lock:
                    p, v, kp, kd, torque = self._mit_target
                payload = pack_mit_command(p, v, kp, kd, torque)
                try:
                    # MIT mode type 0 in bits 10..8; low byte is motor ID.
                    self.send(MOTOR_ID, payload, extended=False)
                except Exception as exc:
                    print("MIT TX error:", exc)
                    self._mit_stop.set()
                    break
                next_t += period
                self._mit_stop.wait(max(0.0, next_t - time.monotonic()))

        self._mit_thread = threading.Thread(target=worker, daemon=True)
        self._mit_thread.start()

    def _stop_mit_stream(self):
        self._mit_stop.set()
        if self._mit_thread and self._mit_thread.is_alive():
            self._mit_thread.join(timeout=0.5)
        self._mit_thread = None

    def mit_control_once_session(self):
        # In MIT protocol, telemetry is normally returned in response to MIT
        # frames. Start with a zero-gain, zero-torque neutral exchange so we
        # can obtain a position sample before asking for a small target step.
        t, _, _ = self.telemetry_snapshot()
        if t is None:
            print("No MIT feedback yet. A neutral handshake will be attempted.")
            if input("Type MIT-NEUTRAL-START to enable and send zero-gain frames: ").strip() != "MIT-NEUTRAL-START":
                print("Cancelled; no MIT frame sent.")
                return
            self.mit_enable()
            self._start_mit_stream((0.0, 0.0, 0.0, 0.0, 0.0))
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                t, _, _ = self.telemetry_snapshot()
                if t is not None:
                    break
                time.sleep(0.05)
            self._stop_mit_stream()
            if t is None:
                self.mit_disable()
                raise RuntimeError("No valid MIT feedback after neutral handshake. Motor disabled; verify MIT feedback ID/format and bitrate.")
            print(f"MIT feedback received. Current position: {t.position:+.5f} rad")

        p = float(input(f"Target position rad [current {t.position:+.3f}]: "))
        v = float(input("Target velocity rad/s [max +/-0.5]: "))
        kp = float(input("Kp [max 20]: "))
        kd = float(input("Kd [max 1.0]: "))
        torque = float(input("Feedforward torque Nm [max +/-0.5]: "))
        if abs(p - t.position) > SAFE_POSITION_STEP_RAD:
            raise ValueError(f"MIT target must stay within +/-{SAFE_POSITION_STEP_RAD:.2f} rad of current position")
        v = clamp(v, -SAFE_MIT_SPEED_RAD_S, SAFE_MIT_SPEED_RAD_S)
        kp = clamp(kp, 0.0, SAFE_MIT_KP)
        kd = clamp(kd, 0.0, SAFE_MIT_KD)
        torque = clamp(torque, -SAFE_MIT_TORQUE_NM, SAFE_MIT_TORQUE_NM)
        print("\nMIT command preview:")
        print(f"p={p:+.4f} rad, v={v:+.3f} rad/s, kp={kp:.2f}, kd={kd:.2f}, torque={torque:+.3f} Nm")
        if input("Type ARM to stream this target at 50 Hz: ").strip() != "ARM":
            print("Cancelled; motor will be disabled.")
            self.mit_disable()
            return
        if not self.enabled:
            self.mit_enable()
        self._start_mit_stream((p, v, kp, kd, torque))
        print("MIT command streaming. Press ENTER to stop and disable.")
        input()
        self.mit_disable()

    # ---------------------------- Baud rate ------------------------------
    def change_bitrate(self):
        print("\nMotor CAN bitrate options (change takes effect after power cycle):")
        for key, (_, _, label) in BAUD_OPTIONS.items():
            print(f" {key}. {label}")
        choice = input("Select new motor bitrate: ").strip()
        if choice not in BAUD_OPTIONS:
            print("Invalid selection.")
            return
        rate, code, label = BAUD_OPTIONS[choice]
        print(f"\nMotor will be configured for {label}.")
        print("After power cycle, CANable and this script must use the same bitrate.")
        if input("Type SET-BITRATE to transmit: ").strip() != "SET-BITRATE":
            print("Cancelled.")
            return
        payload = bytes([1, 2, 3, 4, 5, 6, code, 0])
        self.send_private(TYPE_SET_BAUDRATE, payload)
        print(f"Bitrate command sent for {label}.")
        print("Power-cycle the motor. Then edit START_BITRATE (or choose at startup) and reconnect.")

    # ------------------------------- UI ---------------------------------
    def private_menu(self):
        while True:
            print("\n========== RS05 PRIVATE PROTOCOL ==========")
            print("1. Speed control")
            print("2. Position control (PP)")
            print("3. Switch motor protocol to MIT (power cycle required)")
            print("4. Change motor CAN bitrate (power cycle required)")
            print("5. Live telemetry (continuous; press Q to exit)")
            print("6. Disable / stop motor")
            print("7. Exit")
            choice = input("Select: ").strip()
            try:
                if choice == "1":
                    speed = float(input("Target speed rad/s [-1..+1]: "))
                    current = float(input("Current limit A [max 1.0]: "))
                    accel = float(input("Acceleration rad/s^2 [max 1.0]: "))
                    print("Output shaft must be clear; motor will be enabled.")
                    if input("Type ARM to execute: ").strip() == "ARM":
                        self.private_speed(speed, current, accel)
                        print("Live telemetry active during motion. Press Q in dashboard to stop.")
                        self.monitor_live()
                        self.write_float(PARAM_SPEED_REF, 0.0)
                        time.sleep(0.2)
                        self.private_disable()
                elif choice == "2":
                    t, _, _ = self.telemetry_snapshot()
                    if t is None:
                        print("No feedback yet; wait for telemetry first.")
                        continue
                    print(f"Current position: {t.position:+.5f} rad")
                    target = float(input("Target absolute position rad (within +/-0.25 rad): "))
                    speed = float(input("Max speed rad/s [max 1.0]: "))
                    accel = float(input("Acceleration rad/s^2 [max 1.0]: "))
                    current = float(input("Current limit A [max 1.0]: "))
                    if input("Type ARM to execute: ").strip() == "ARM":
                        self.private_position(target, speed, accel, current)
                        print("Live telemetry active during PP motion. Press Q to disable and return.")
                        self.monitor_live()
                        self.private_disable()
                elif choice == "3":
                    self.switch_to_mit()
                elif choice == "4":
                    self.change_bitrate()
                elif choice == "5":
                    self.monitor_live()
                elif choice == "6":
                    self.private_disable()
                elif choice == "7":
                    break
                else:
                    print("Unknown option.")
            except (ValueError, RuntimeError, can.CanError, OSError) as exc:
                print("Command rejected/error:", exc)
                print("If the motor is enabled, use option 6 to disable.")

    def mit_menu(self):
        while True:
            print("\n=============== RS05 MIT PROTOCOL ===============")
            print("1. MIT control session (50 Hz stream)")
            print("2. Live telemetry (continuous; press Q to exit)")
            print("3. Disable motor")
            print("4. Exit")
            choice = input("Select: ").strip()
            try:
                if choice == "1":
                    self.mit_control_once_session()
                elif choice == "2":
                    self.monitor_live()
                elif choice == "3":
                    self.mit_disable()
                elif choice == "4":
                    break
                else:
                    print("Unknown option.")
            except (ValueError, RuntimeError, can.CanError, OSError) as exc:
                print("Command rejected/error:", exc)

    def close(self):
        try:
            if self.protocol == "private" and self.enabled:
                self.private_disable()
            elif self.protocol == "mit" and self.enabled:
                self.mit_disable()
        except Exception as exc:
            print("Shutdown command warning:", exc)
        self._stop_mit_stream()
        self.stop_event.set()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.5)
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
        if self.bus:
            self.bus.shutdown()
        print("CAN closed. CSV:", CSV_PATH.resolve())


def main():
    print("====================================================")
    print(" ROBSTRIDE RS05 | CANABLE V2 | PYTHON CONSOLE")
    print("====================================================")
    print("Default: COM9 | Motor 0x7F | Host 0xFD | 1 Mbps")
    print("48 V nominal supply is within RS05 15-60 V input range.")
    print("Software first-test caps: 1 A phase-current limit, 1 rad/s, +/-0.25 rad.")
    print("This is bench software, not a safety-rated motion controller.")

    port = input(f"SLCAN COM port [{PORT}]: ").strip() or PORT
    print("\nSelect CAN bitrate for this connection:")
    for key, (rate, _, label) in BAUD_OPTIONS.items():
        default_mark = " (default)" if rate == START_BITRATE else ""
        print(f" {key}. {label}{default_mark}")
    bchoice = input("Bitrate [1]: ").strip() or "1"
    if bchoice not in BAUD_OPTIONS:
        print("Invalid bitrate selection.")
        return
    bitrate = BAUD_OPTIONS[bchoice][0]

    proto = input("Motor protocol: [P]rivate or [M]IT? [P]: ").strip().lower() or "p"
    if proto not in ("p", "m"):
        print("Invalid protocol selection.")
        return

    print("\nSAFETY CHECK BEFORE CONNECTING:")
    print("- Motor securely mounted; output shaft and coupling area clear.")
    print("- 48 V supply polarity correct; set PSU current limit conservatively (start around 1-2 A input limit).")
    print("- Software phase-current limit is separately capped at 1.0 A.")
    print("- Have a physical way to remove motor power. Keyboard stop is not safety-rated.")
    if input("Type READY when the bench is physically safe: ").strip() != "READY":
        print("Aborted.")
        return

    app = RS05Console(port, bitrate)
    app.protocol = "private" if proto == "p" else "mit"
    try:
        app.connect()
        time.sleep(0.5)
        app.monitor_snapshot()
        if app.protocol == "private":
            app.private_menu()
        else:
            print("MIT mode selected. If no feedback appears, do not arm; verify MIT protocol/feedback ID.")
            app.mit_menu()
    except KeyboardInterrupt:
        print("\nCtrl+C received.")
    except Exception as exc:
        print("Fatal error:", repr(exc))
    finally:
        app.close()


if __name__ == "__main__":
    main()
