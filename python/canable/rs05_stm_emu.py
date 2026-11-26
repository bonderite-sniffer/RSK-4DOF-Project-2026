"""rs05_stm_emu.py - a software stand-in for the STM32 gateway firmware (protocol v3).

It speaks the same USB protocol as Firmware_Gateway (PING, CMD_GATEWAY, CMD_CAN_TX, EVT_CAN_RX,
EVT_GW_STATS, EVT_HEALTH, ESTOP, watchdog) and puts simulated RS05 motors (rs05_sim.FakeRS05) on two
python-can 'virtual' channels = CAN1 and CAN2. Used for:

    python rs05_gui.py          Interface "stm-sim"
    python rs05_multi_gui.py    Interface "stm-sim"   (first half of the plan IDs on CAN1, rest on CAN2)
    tests of rs05_stm.StmBus without hardware

    from rs05_stm_emu import open_stm_sim
    bus, emu = open_stm_sim({0: [1, 2], 1: [3, 4]})      # StmBus + emulator with 4 fake motors
"""
from __future__ import annotations

import queue
import struct
import threading
import time

import can

from rs05_stm import (CMD_CAN_TX, CMD_ESTOP, CMD_GATEWAY, CMD_GET_HEALTH, CMD_HEARTBEAT, CMD_PING,
                      EVT_ACK, EVT_CAN_RX, EVT_EVENT, EVT_GW_STATS, EVT_HEALTH, EVT_PONG, GW_FLAG_EXT, GW_RX,
                      GW_STATS, GW_TX, PONG, FrameParser, StmBus, encode_frame)

FW_VERSION, PROTO = 0x00030000, 3
RX_BATCH = 6
ST_OK, ST_BAD_LEN, ST_BUSY, ST_BAD_ARG, ST_UNKNOWN = 0, -1, -2, -4, -6


class _Pipe:
    """One direction of a fake serial line."""

    def __init__(self):
        self.q: queue.Queue = queue.Queue()
        self.closed = False


class EmuSerial:
    """pyserial-like end point used by StmBus(serial_port=...)."""

    def __init__(self, rx: _Pipe, tx: _Pipe, timeout=0.02):
        self._rx, self._tx, self.timeout = rx, tx, timeout
        self._buf = bytearray()

    @property
    def in_waiting(self):
        return len(self._buf) + self._rx.q.qsize()

    def read(self, n=1):
        if not self._buf:
            try:
                self._buf += self._rx.q.get(timeout=self.timeout)
            except queue.Empty:
                if self._rx.closed:
                    raise OSError("port closed")
                return b""
        while not self._rx.q.empty() and len(self._buf) < n:
            self._buf += self._rx.q.get_nowait()
        out, self._buf = bytes(self._buf[:n]), self._buf[n:]
        return out

    def write(self, data: bytes):
        if self._tx.closed:
            raise OSError("port closed")
        self._tx.q.put(bytes(data))
        return len(data)

    def close(self):
        self._tx.closed = self._rx.closed = True


class StmEmulator:
    _count = 0

    def __init__(self, channels=None, host_id=0xFD):
        StmEmulator._count += 1
        if channels is None:                        # unique per instance (virtual buses are shared by name)
            channels = (f"stmemu{StmEmulator._count}_can1", f"stmemu{StmEmulator._count}_can2")
        self.connect()
        self.can = [can.Bus(interface="virtual", channel=c, receive_own_messages=False) for c in channels]
        self.channels = channels
        self.host_id = host_id
        self.enabled, self.wd_ms, self.wd_tripped, self.wd_trips = False, 0, False, 0
        self.tx_ok, self.tx_drop, self.rx_fwd = [0, 0], [0, 0], [0, 0]
        self.seen = [set(), set()]
        self.last_host = time.monotonic()
        self.fakes = []
        self._batch: list[bytes] = []
        self._t0 = time.monotonic()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def connect(self) -> EmuSerial:
        """New 'USB cable': fresh serial pair (a closed StmBus closes its port, like unplugging)."""
        h2d, d2h = _Pipe(), _Pipe()
        self.host_port = EmuSerial(d2h, h2d)
        self.dev_port = EmuSerial(h2d, d2h, timeout=0.001)
        self._parser = FrameParser()
        self.enabled = False                         # firmware: a new host must enable the gateway again
        return self.host_port

    def add_motor(self, bus: int, motor_id: int, **kw):
        from rs05_sim import FakeRS05
        f = FakeRS05(self.channels[bus], motor_id, self.host_id, **kw)
        self.fakes.append(f)
        return f

    # -------------------------------------------------------------- device side
    def _send(self, ftype, payload=b""):
        try:
            self.dev_port.write(encode_frame(ftype, payload))
        except OSError:
            pass

    def _ack(self, cmd, st):
        self._send(EVT_ACK, struct.pack("<Bb", cmd, st))

    def _stop_all(self):
        for b in range(2):
            for mid in sorted(self.seen[b]):
                aid = (4 << 24) | (self.host_id << 8) | mid
                self.can[b].send(can.Message(arbitration_id=aid, is_extended_id=True, data=bytes(8)))

    def _command(self, ftype, p):
        self.last_host = time.monotonic()
        if ftype == CMD_CAN_TX:
            if len(p) == 0 or len(p) % GW_TX.size:
                return self._ack(ftype, ST_BAD_LEN)
            if not self.enabled:
                return self._ack(ftype, ST_BUSY)
            for k in range(len(p) // GW_TX.size):
                bus, flags, dlc, aid, data = GW_TX.unpack_from(p, k * GW_TX.size)
                if bus > 1 or dlc > 8:
                    self._ack(ftype, ST_BAD_ARG)
                    continue
                self.can[bus].send(can.Message(arbitration_id=aid, is_extended_id=bool(flags & GW_FLAG_EXT),
                                               dlc=dlc, data=data[:dlc]))
                self.tx_ok[bus] += 1
        elif ftype == CMD_PING:
            self._send(EVT_PONG, PONG.pack(FW_VERSION, 500, 4, 2, PROTO, 0))
        elif ftype == CMD_GATEWAY:
            if len(p) != 3:
                return self._ack(ftype, ST_BAD_LEN)
            en, wd = struct.unpack("<BH", p)
            self.enabled, self.wd_ms, self.wd_tripped = bool(en), wd, False
            self._ack(ftype, ST_OK)
        elif ftype == CMD_ESTOP:
            if self.enabled:
                self._stop_all()
            self._ack(ftype, ST_OK)
        elif ftype == CMD_GET_HEALTH:
            self._health()
        elif ftype == CMD_HEARTBEAT:
            pass
        else:
            self._ack(ftype, ST_UNKNOWN)

    def _health(self):
        p = struct.pack("<I", int(1000 * (time.monotonic() - self._t0)))
        for b in range(2):
            p += struct.pack("<BBBB7I", 1, 0, 0, 0, self.tx_ok[b], 0, 0, self.rx_fwd[b], 0, 0, 0)
        p += struct.pack("<4f4H", 48, 48, 48, 48, 0, 0, 0, 0)
        self._send(EVT_HEALTH, p)

    def _flush(self):
        if self._batch:
            self._send(EVT_CAN_RX, b"".join(self._batch))
            self._batch = []

    def _run(self):
        t_stats = 0.0
        while not self._stop.is_set():
            try:
                data = self.dev_port.read(256)
            except OSError:                          # host closed the port: wait for connect()
                time.sleep(0.01)
                data = b""
            for ftype, p in self._parser.feed(data):
                self._command(ftype, p)
            for b in range(2):
                while True:
                    m = self.can[b].recv(0)
                    if m is None:
                        break
                    if not self.enabled:
                        continue
                    if m.is_extended_id:
                        src = (m.arbitration_id >> 8) & 0xFF
                        if src != self.host_id:
                            self.seen[b].add(src)
                    t_us = int(1e6 * (time.monotonic() - self._t0)) & 0xFFFFFFFF
                    self._batch.append(GW_RX.pack(b, GW_FLAG_EXT if m.is_extended_id else 0, m.dlc,
                                                  m.arbitration_id, t_us, bytes(m.data).ljust(8, b"\0")))
                    self.rx_fwd[b] += 1
                    if len(self._batch) >= RX_BATCH:
                        self._flush()
            self._flush()
            now = time.monotonic()
            if self.enabled and self.wd_ms:
                silent = 1000 * (now - self.last_host)
                if silent > self.wd_ms:
                    if not self.wd_tripped:
                        self.wd_tripped, self.wd_trips = True, self.wd_trips + 1
                        self._stop_all()
                        self._send(EVT_EVENT, struct.pack("<BBf", 0xFF, 7, silent))
                else:
                    self.wd_tripped = False
            if now - t_stats > 0.2:
                t_stats = now
                self._health()
                if self.enabled:
                    self._send(EVT_GW_STATS, GW_STATS.pack(int(1000 * (now - self._t0)), *self.tx_ok,
                                                           *self.tx_drop, *self.rx_fwd, 0, self.wd_trips,
                                                           0, 0, 0, 0, 1, int(self.wd_tripped)))

    def close(self):
        self._stop.set()
        self._th.join(timeout=0.5)
        self.dev_port.close()
        for f in self.fakes:
            f.close()
        for c in self.can:
            c.shutdown()


def open_stm_sim(motors: dict, wd_ms: int = 300, **kw) -> tuple[StmBus, StmEmulator]:
    """motors = {0: [ids on CAN1], 1: [ids on CAN2]} -> (StmBus, emulator). Close both when done."""
    emu = StmEmulator()
    for bus, ids in motors.items():
        for mid in ids:
            emu.add_motor(bus, mid)
    try:
        return StmBus("stm-sim", serial_port=emu.connect(), wd_ms=wd_ms, **kw), emu
    except Exception:
        emu.close()
        raise
