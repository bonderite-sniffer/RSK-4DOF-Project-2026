#!/usr/bin/env python3
"""rs05_protocol.py - which protocol is an RS05 speaking: PRIVATE, MIT or CANopen? And switch it back.

An RS05 speaks exactly one protocol, chosen in flash (private type 25 / MIT command 8, applied after a
power cycle). The tools in this folder only speak PRIVATE, so a motor left in MIT or CANopen mode looks
"dead". This probe finds out, without making the motor move:

  phase      what is sent                                       a reply means
  listen     nothing (1 s)                                       anything chatty (active report, heartbeat)
  private    type 0 'get ID' to IDs 0..255 (29-bit)              PRIVATE, motor ID = reply bits 15..8
  mit        MIT 'stop' FF FF FF FF FF FF FF FD to 11-bit IDs    MIT, motor ID = reply byte 0
             1..254 (stop only disables)                         (reply = feedback on 11-bit ID = host ID)
  canopen    SDO read of object 0x1000 to 0x600+node, 1..127     CANopen, node = reply ID - 0x580
  boot       (optional) listen while you power-cycle the motor   CANopen boot-up 0x700+node, data 00

Switch back to PRIVATE (takes effect after a power cycle, motor answers with its 64-bit UID):
  from MIT     11-bit ID = motor ID, data FF FF FF FF FF FF 00 FD   (RS05 manual, MIT command 8)
  from CANopen 29-bit ID 0xFFF, data 01 02 03 04 05 06 00 00        (RS05 manual 5.10 - VERIFY)
If that does not work, use the RobStride PC software (protocol_1, param 0x2022).

Examples (PowerShell, from python/canable):
  python rs05_protocol.py -p COM9                       detect at 1 Mbit/s
  python rs05_protocol.py -p COM9 --all-bitrates        detect at 1M / 500k / 250k / 125k
  python rs05_protocol.py -p COM9 --boot 15             also listen 15 s while you power-cycle the motor
  python rs05_protocol.py -p COM9 --to-private mit --id 0x7F
  python rs05_protocol.py -p COM9 --to-private canopen
  python rs05_protocol.py --sim mit                     try it on a simulated motor stuck in MIT mode
"""
from __future__ import annotations

import argparse
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import can

from rs05_can import (P_MAX, T_FAULT, T_FEEDBACK, T_GET_ID, T_MAX, T_READ, T_ACTIVE_REPORT, V_MAX,
                      open_bus, u2f)

PRIVATE, MIT, CANOPEN = "private", "mit", "canopen"
_PRIV_REPLY = {T_GET_ID: "get-ID reply", T_FEEDBACK: "feedback", T_READ: "param reply",
               T_FAULT: "fault frame", T_ACTIVE_REPORT: "active report"}
MIT_STOP = bytes([0xFF] * 7 + [0xFD])
CANOPEN_SWITCH_ID = 0xFFF                         # RS05 manual 5.10, extended frame - VERIFY
SDO_READ_1000 = bytes([0x40, 0x00, 0x10, 0x00, 0, 0, 0, 0])
NMT_STATES = {0x00: "boot-up", 0x04: "stopped", 0x05: "operational", 0x7F: "pre-operational"}


def fmt_frame(m: can.Message) -> str:
    aid = f"{m.arbitration_id:08X}" if m.is_extended_id else f"{m.arbitration_id:03X}"
    return f"{aid} [{len(m.data)}] {bytes(m.data).hex(' ')}"


def decode_mit_feedback(d: bytes) -> str:
    """MIT 'response command 1': id, p16, v12, t12, temp*10."""
    p = u2f((d[1] << 8) | d[2], -P_MAX, P_MAX)
    v = u2f((d[3] << 4) | (d[4] >> 4), -V_MAX, V_MAX, 12)
    t = u2f(((d[4] & 0x0F) << 8) | d[5], -T_MAX, T_MAX, 12)
    temp = ((d[6] << 8) | d[7]) / 10
    return f"pos {p:+.3f} rad, vel {v:+.2f} rad/s, torque {t:+.2f} Nm, temp {temp:.1f} C"


# ====================================================================== result
@dataclass
class Finding:
    protocol: str
    motor_id: int
    evidence: str
    raw: str = ""


@dataclass
class ProbeReport:
    bitrate: Optional[int] = None
    findings: list = field(default_factory=list)
    other: dict = field(default_factory=dict)           # frame id text -> count
    error: str = ""

    def add(self, proto, mid, evidence, raw=""):
        if not any(f.protocol == proto and f.motor_id == mid for f in self.findings):
            self.findings.append(Finding(proto, mid, evidence, raw))

    def protocols(self) -> set:
        return {f.protocol for f in self.findings}

    def text(self) -> str:
        br = f"@{self.bitrate} bit/s" if self.bitrate else "@bus"
        out = []
        for f in sorted(self.findings, key=lambda f: (f.protocol, f.motor_id)):
            out.append(f"{br}  {f.protocol.upper():8s} ID 0x{f.motor_id:02X} ({f.motor_id}): {f.evidence}"
                       + (f"\n{'':14s}raw {f.raw}" if f.raw else ""))
        if self.other:
            top = sorted(self.other.items(), key=lambda kv: -kv[1])[:10]
            out.append(f"{br}  unexplained frames: " + ", ".join(f"{k} x{n}" for k, n in top))
        if self.error and not self.findings:
            out.append(f"{br}  error: {self.error}")
        return "\n".join(out) or f"{br}  nothing answered"

    def advice(self) -> str:
        p = self.protocols()
        if not p:
            return ("No motor answered in any protocol. Check 48 V, CAN H/L, 120 ohm termination, CANable port;"
                    " try --all-bitrates and --boot.")
        lines = []
        for f in self.findings:
            if f.protocol == PRIVATE:
                lines.append(f"ID 0x{f.motor_id:02X}: PRIVATE - OK, usable with rs05_gui.py / rs05_canable_test.py.")
            elif f.protocol == MIT:
                lines.append(f"ID 0x{f.motor_id:02X}: stuck in MIT protocol -> "
                             f"python rs05_protocol.py --to-private mit --id 0x{f.motor_id:02X}, then power-cycle.")
            else:
                lines.append(f"node {f.motor_id}: CANopen -> python rs05_protocol.py --to-private canopen, "
                             f"then power-cycle (VERIFY; else RobStride PC software).")
        return "\n".join(lines)


# ====================================================================== probe
class _Collector:
    """Own receive loop: every frame goes into the list of the current phase."""

    def __init__(self, bus):
        self.bus = bus
        self.frames: list[tuple[float, can.Message]] = []
        self._lock = threading.Lock()
        self._done = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def _run(self):
        while not self._done.is_set():
            try:
                m = self.bus.recv(timeout=0.05)
            except (can.CanError, OSError, ValueError):
                time.sleep(0.02)
                continue
            if m is not None and not m.is_error_frame:
                with self._lock:
                    self.frames.append((time.monotonic(), m))

    def take(self) -> list[can.Message]:
        with self._lock:
            out, self.frames = [m for _, m in self.frames], []
        return out

    def close(self):
        self._done.set()
        self._th.join(timeout=0.5)


def _send(bus, arb, data, ext, rep: ProbeReport):
    try:
        bus.send(can.Message(arbitration_id=arb, is_extended_id=ext, data=data), timeout=0.1)
    except can.CanError as e:
        rep.error = rep.error or f"send failed ({e}) - no ACK: no device at this bitrate?"


def _is_own(m: can.Message, host: int) -> bool:
    """Frames we sent ourselves (if the interface echoes them)."""
    if m.is_extended_id:
        return ((m.arbitration_id >> 8) & 0xFF) == host and m.arbitration_id != CANOPEN_SWITCH_ID
    return bytes(m.data) == MIT_STOP or bytes(m.data) == SDO_READ_1000


def _classify_passive(m: can.Message, rep: ProbeReport, host: int) -> bool:
    """Frames that identify a protocol by themselves. True if explained."""
    a = m.arbitration_id
    if m.is_extended_id:
        typ, src = (a >> 24) & 0x1F, (a >> 8) & 0xFF
        if typ in _PRIV_REPLY and src != host:
            rep.add(PRIVATE, src, _PRIV_REPLY[typ], fmt_frame(m))
            return True
        return False
    if 0x701 <= a <= 0x77F and len(m.data) == 1:
        rep.add(CANOPEN, a - 0x700, f"NMT {NMT_STATES.get(m.data[0], hex(m.data[0]))} (heartbeat 0x{a:03X})",
                fmt_frame(m))
        return True
    if 0x581 <= a <= 0x5FF and len(m.data) == 8:
        node = a - 0x580
        if m.data[0] == 0x80:
            code = struct.unpack_from("<I", bytes(m.data), 4)[0]
            rep.add(CANOPEN, node, f"SDO abort 0x{code:08X} (still a CANopen node)", fmt_frame(m))
        else:
            val = struct.unpack_from("<I", bytes(m.data), 4)[0]
            rep.add(CANOPEN, node, f"SDO reply, device type (0x1000) = 0x{val:08X}", fmt_frame(m))
        return True
    return False


def probe(bus: can.BusABC, host: int = 0xFD, bitrate: Optional[int] = None, listen: float = 1.0,
          boot: float = 0.0, progress=None, boot_prompt=None) -> ProbeReport:
    """Detect PRIVATE / MIT / CANopen motors on `bus`. Nothing else may read the bus meanwhile."""
    rep = ProbeReport(bitrate=bitrate)
    col = _Collector(bus)
    step = lambda name, f: progress and progress(name, f)  # noqa: E731

    def leftovers(frames, phase):
        for m in frames:
            if not _is_own(m, host) and not _classify_passive(m, rep, host):
                k = f"{fmt_frame(m).split(' [')[0]}({phase})"
                rep.other[k] = rep.other.get(k, 0) + 1

    try:
        # -- passive
        step("listen", 0.0)
        time.sleep(listen)
        leftovers(col.take(), "listen")

        # -- private: get ID to every 8-bit ID
        for i, cid in enumerate(range(256)):
            _send(bus, (T_GET_ID << 24) | (host << 8) | cid, bytes(8), True, rep)
            time.sleep(0.002)
            if i % 32 == 0:
                step("private", i / 256)
        time.sleep(0.3)
        leftovers(col.take(), "private")

        # -- MIT: 'stop' to every 11-bit ID with mode type 0 (IDs 1..254). A reply is the MIT feedback
        #    frame (11-bit ID = host ID, byte 0 = motor ID).
        mit_frames = []
        for i, cid in enumerate(range(1, 255)):
            _send(bus, cid, MIT_STOP, False, rep)
            time.sleep(0.003)
            if i % 32 == 0:
                step("mit", i / 254)
        time.sleep(0.3)
        for m in col.take():
            if (not m.is_extended_id and len(m.data) == 8 and bytes(m.data) != MIT_STOP
                    and 1 <= m.data[0] <= 254 and not (0x581 <= m.arbitration_id <= 0x5FF)):
                mit_frames.append(m)
            else:
                leftovers([m], "mit")
        for cid in sorted({m.data[0] for m in mit_frames}):     # confirm each candidate on its own
            col.take()
            _send(bus, cid, MIT_STOP, False, rep)
            time.sleep(0.15)
            ok = [m for m in col.take() if not m.is_extended_id and len(m.data) == 8 and m.data[0] == cid]
            if ok:
                m = ok[0]
                rep.add(MIT, cid, f"answered MIT stop on reply ID 0x{m.arbitration_id:03X} "
                                  f"(= host ID); {decode_mit_feedback(bytes(m.data))}", fmt_frame(m))
            else:
                leftovers([m for m in mit_frames if m.data[0] == cid], "mit-unconfirmed")

        # -- CANopen: SDO upload of 0x1000 (device type, mandatory object) to nodes 1..127
        for node in range(1, 128):
            _send(bus, 0x600 + node, SDO_READ_1000, False, rep)
            time.sleep(0.003)
            if node % 16 == 0:
                step("canopen", node / 127)
        time.sleep(0.3)
        leftovers(col.take(), "canopen")

        # -- optional: listen while the motor is power-cycled (CANopen boot-up, anything at boot)
        if boot > 0:
            if boot_prompt:
                boot_prompt(boot)
            t_end = time.monotonic() + boot
            while time.monotonic() < t_end:
                step("boot", 1 - (t_end - time.monotonic()) / boot)
                time.sleep(0.2)
            leftovers(col.take(), "boot")
    finally:
        col.close()
    if rep.findings:
        rep.error = ""
    return rep


def probe_bitrates(interface, channel, host=0xFD, bitrates=(1_000_000, 500_000, 250_000, 125_000),
                   **kw) -> list[ProbeReport]:
    out = []
    for br in bitrates:
        try:
            bus = open_bus(interface, channel, br)
        except Exception as e:  # noqa: BLE001
            out.append(ProbeReport(bitrate=br, error=f"cannot open: {e}"))
            continue
        try:
            out.append(probe(bus, host, bitrate=br, **kw))
        finally:
            bus.shutdown()
    return out


# ====================================================================== switch back to private
def switch_to_private(bus: can.BusABC, source: str, motor_id: int = 0x7F, wait: float = 0.5) -> tuple[bool, str]:
    """Ask a motor in MIT or CANopen mode to use the PRIVATE protocol after the next power cycle.
    Returns (acknowledged, text). The motor answers with its 64-bit MCU UID on 11-bit ID = motor ID."""
    if source == MIT:
        if not 1 <= motor_id <= 254:
            raise ValueError("MIT motor ID must be 1..254")
        arb, ext, data = motor_id, False, bytes([0xFF] * 6 + [0x00, 0xFD])        # MIT command 8, F_CMD 0
    elif source == CANOPEN:
        arb, ext, data = CANOPEN_SWITCH_ID, True, bytes([1, 2, 3, 4, 5, 6, 0, 0])  # manual 5.10, VERIFY
    else:
        raise ValueError("source must be 'mit' or 'canopen'")
    col = _Collector(bus)
    try:
        bus.send(can.Message(arbitration_id=arb, is_extended_id=ext, data=data), timeout=0.2)
        time.sleep(wait)
        frames = [m for m in col.take() if not m.is_extended_id and len(m.data) == 8 and bytes(m.data) != data]
    finally:
        col.close()
    sent = f"sent {('%08X' if ext else '%03X') % arb} [8] {data.hex(' ')}"
    acks = [m for m in frames if source == CANOPEN or m.arbitration_id == motor_id]
    if acks:
        m = acks[0]
        return True, (f"{sent}\nACK from 11-bit ID 0x{m.arbitration_id:03X}, UID {bytes(m.data).hex()}.\n"
                      "Now POWER-CYCLE the motor, then run: python rs05_protocol.py (should say PRIVATE).")
    return False, (f"{sent}\nNo acknowledgement. The frame may still have been accepted - power-cycle and probe "
                   "again.\nIf the motor is still not PRIVATE, switch it with the RobStride PC software.")


# ====================================================================== CLI
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="COM9")
    ap.add_argument("--iface", default="slcan", help="slcan | gs_usb | stm (STM32 gateway, 1 Mbit/s only)")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--all-bitrates", action="store_true", help="probe at 1M, 500k, 250k and 125k")
    ap.add_argument("--host", type=lambda x: int(x, 0), default=0xFD)
    ap.add_argument("--boot", type=float, default=0.0, help="extra seconds of listening while you power-cycle")
    ap.add_argument("--to-private", choices=(MIT, CANOPEN), help="send the 'switch to private protocol' frame")
    ap.add_argument("--id", type=lambda x: int(x, 0), default=0x7F, help="motor ID for --to-private mit")
    ap.add_argument("-y", "--yes", action="store_true", help="skip confirmation")
    ap.add_argument("--sim", choices=(PRIVATE, MIT, CANOPEN), help="test against a simulated motor in this protocol")
    ap.add_argument("--sim-id", type=lambda x: int(x, 0), default=0x7F)
    a = ap.parse_args()

    sim = None
    if a.sim:
        from rs05_sim import FakeRS05
        sim = FakeRS05("protosim", a.sim_id, a.host, protocol=a.sim)
        open_ = lambda br=None: can.Bus(interface="virtual", channel="protosim")  # noqa: E731
    else:
        open_ = lambda br=None: open_bus(a.iface, a.port, br or a.bitrate)  # noqa: E731

    try:
        if a.to_private:
            what = f"motor ID 0x{a.id:02X} (MIT)" if a.to_private == MIT else "every CANopen motor on the bus"
            print(f"Switch {what} to the PRIVATE protocol (takes effect after a power cycle).")
            if not a.yes and input("Type SWITCH to send: ").strip() != "SWITCH":
                print("cancelled")
                return
            bus = open_()
            try:
                ok, txt = switch_to_private(bus, a.to_private, a.id)
            finally:
                bus.shutdown()
            print(txt)
            if sim and ok:
                sim.power_cycle()
                print("(sim: motor power-cycled)")
            return

        print("Probing PRIVATE / MIT / CANopen. MIT probe sends 'stop' (disables a running motor); "
              "nothing is enabled or moved.")

        def prog(name, f):
            print(f"  {name:8s} {100 * f:5.1f} %", end="\r")

        def boot_prompt(sec):
            print(f"\n>>> POWER-CYCLE THE MOTOR NOW (listening {sec:.0f} s) <<<")
            if sim:
                threading.Timer(1.0, sim.power_cycle).start()

        kw = dict(host=a.host, progress=prog, boot=a.boot, boot_prompt=boot_prompt)
        if a.all_bitrates and not sim:
            reports = probe_bitrates(a.iface, a.port, **kw)
        else:
            bus = open_()
            try:
                reports = [probe(bus, bitrate=None if sim else a.bitrate, **kw)]
            finally:
                bus.shutdown()
        print(" " * 40)
        for r in reports:
            print(r.text())
        found = [r for r in reports if r.findings]
        print("\n" + (found[0].advice() if found else reports[0].advice()))
        if any(r.bitrate and r.bitrate != 1_000_000 for r in found):
            print("note: found at a bitrate other than 1 Mbit/s - the motor's baud rate was changed.")
    finally:
        if sim:
            sim.close()


if __name__ == "__main__":
    main()
