"""rs05_stm.py - use the STM32H723 (Firmware_Gateway, protocol v3) as the CAN interface.

The STM32 drives the two MCP2515 CAN channels (CAN1, CAN2; 2 motors each). Python keeps doing
everything it already does (control, safety, GUIs, tests) and only swaps the "CANable" for the STM:

    from rs05_can import open_bus
    bus = open_bus("stm", "COM8")             # instead of open_bus("slcan", "COM9")

    python rs05_gui.py           -> Interface "stm", Port = the STM's USB COM port
    python rs05_multi_gui.py     -> same
    python rs05_canable_test.py --iface stm -p COM8 scan
    python rs05_multi.py --iface stm -p COM8 --check-ids --ids 1 2 3 4

How it works (see Firmware_Gateway/Core/Inc/gateway.h):
    send(msg)   -> CMD_CAN_TX (bus, id, data) -> STM queue -> MCP2515 -> CAN1 / CAN2 (order kept)
    recv()      <- EVT_CAN_RX: every frame the STM receives on CAN1 / CAN2 (msg.channel = 0 / 1)
    watchdog    -> no frame from Python for wd_ms (default 300) -> the STM sends STOP to every motor
                   it has seen. A heartbeat thread keeps it fed while the bus is open.

Which CAN channel a frame goes to:
    can_channel="auto" (default): learned from the frames each motor sends (ID -> channel). A frame for
        an ID that has not been seen yet (scans, first STOP) goes to BOTH channels. An ID seen on both
        channels is a conflict (two motors with the same ID on different channels): frames go to both
        and replies from both arrive, exactly like a duplicate ID on one bus.
    can_channel=0 or 1: only that channel (RX from the other one is ignored).
    routes={1: 0, 2: 0, 3: 1, 4: 1}: fixed ID -> channel table (wins over learning).

The bitrate is fixed by the firmware (1 Mbit/s); asking for another one raises an error.
"""
from __future__ import annotations

import queue
import struct
import threading
import time
from typing import Optional

import can

# ------------------------------------------------------------------ link framing (same as python/rs05_host.py)
CMD_PING, CMD_ESTOP, CMD_HEARTBEAT, CMD_GET_HEALTH = 0x01, 0x02, 0x03, 0x07
CMD_GATEWAY, CMD_CAN_TX = 0x50, 0x51
EVT_ACK, EVT_EVENT, EVT_PONG, EVT_HEALTH, EVT_CAN_RX, EVT_GW_STATS = 0x80, 0x84, 0x85, 0x86, 0x87, 0x88

GW_TX = struct.Struct("<BBBI8s")             # 15 bytes, host -> MCU
GW_RX = struct.Struct("<BBBII8s")            # 19 bytes, MCU -> host
GW_FLAG_EXT = 0x01
GW_STATS = struct.Struct("<I2I2I2IIIBBBBBB2x")   # 44 bytes (gw_stats_t)
HEALTH_BUS = struct.Struct("<BBBB7I")        # 32 bytes (health_bus_t)
HEALTH_TAIL = struct.Struct("<4f4H")         # vbus[4], fb_age_ms[4]
PONG = struct.Struct("<IIBBBB")              # 12 bytes
MAX_TX_PER_MSG = 8                           # 8 x 15 = 120 <= LINK_MAX_PAYLOAD
ACK_STATUS = {0: "OK", -1: "BAD_LEN", -2: "BUSY", -4: "BAD_ARG", -5: "CAN_ERR", -6: "UNKNOWN_CMD"}
EVENT_NAMES = {7: "WATCHDOG (STM stopped all motors)", 8: "ESTOP", 9: "CAN_ERROR"}
FW_BITRATE = 1_000_000

assert GW_TX.size == 15 and GW_RX.size == 19 and GW_STATS.size == 44 and HEALTH_BUS.size == 32


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


def frame_target(msg: can.Message) -> Optional[int]:
    """CAN ID of the motor a host frame is meant for (None = broadcast / unknown)."""
    aid = msg.arbitration_id
    if msg.is_extended_id:
        return aid & 0xFF                          # private protocol: target in bits 7..0
    if 0x600 <= aid < 0x680:
        return aid - 0x600                         # CANopen SDO request
    if aid > 0 and (aid >> 8) <= 7:
        return aid & 0xFF                          # MIT protocol: bits 7..0 = motor ID
    return None


def frame_source(msg: can.Message) -> Optional[int]:
    """CAN ID of the motor that sent a frame (None = unknown)."""
    aid = msg.arbitration_id
    if msg.is_extended_id:
        return (aid >> 8) & 0xFF                   # private protocol: sender in bits 15..8
    if 0x580 <= aid < 0x600:
        return aid - 0x580                         # CANopen SDO reply
    if 0x700 < aid < 0x780:
        return aid - 0x700                         # CANopen heartbeat / boot-up
    return None


class StmBus(can.BusABC):
    """python-can bus on top of the STM32 gateway. See the module docstring."""

    def __init__(self, channel: str = "COM8", can_channel="auto", routes: Optional[dict] = None,
                 wd_ms: int = 300, bitrate: int = FW_BITRATE, serial_port=None, heartbeat_s: float = 0.1,
                 **kwargs):
        if bitrate and int(bitrate) != FW_BITRATE:
            raise can.CanInitializationError(
                f"the STM firmware runs CAN at {FW_BITRATE // 1000} kbit/s only (asked {bitrate}); "
                f"change MCP2515_BITRATE in app_config.h and reflash for another bitrate")
        if can_channel not in ("auto", 0, 1):
            raise ValueError("can_channel must be 'auto', 0 (CAN1) or 1 (CAN2)")
        self.channel_info = f"STM32 gateway {channel}"
        self.can_channel = can_channel
        self.static_routes = dict(routes or {})
        self.learned: dict[int, object] = {}       # motor ID -> 0, 1 or "both"
        self.conflicts: set[int] = set()
        self.wd_ms = int(wd_ms)
        self.health: Optional[dict] = None
        self.gw_stats: Optional[dict] = None
        self.pong: Optional[dict] = None
        self.events: list[str] = []
        self.nacks: dict[str, int] = {}
        self.rx_count = [0, 0]
        self.tx_count = [0, 0]
        self._rxq: queue.Queue = queue.Queue(maxsize=50000)
        self.rx_dropped = 0
        self._parser = FrameParser()
        self._wlock = threading.Lock()
        self._cv = threading.Condition()
        self._acks: dict[int, int] = {}
        self._stop = threading.Event()
        if serial_port is None:
            import serial                            # pyserial
            serial_port = serial.Serial(channel, 115200, timeout=0.02, write_timeout=0.5)
        self.ser = serial_port
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        try:
            self._handshake()
        except Exception:
            self._close_port()
            raise
        self._hb = threading.Thread(target=self._heartbeat_loop, args=(heartbeat_s,), daemon=True)
        self._hb.start()
        super().__init__(channel=channel, **kwargs)

    # -------------------------------------------------------------- link
    def _write(self, data: bytes):
        try:
            with self._wlock:
                self.ser.write(data)
        except Exception as e:  # noqa: BLE001 - serial.SerialException, OSError ...
            raise can.CanOperationError(f"STM USB write failed: {e}") from e

    def _request(self, ftype: int, payload: bytes = b"", timeout: float = 1.0) -> int:
        """Send a command that answers with EVT_ACK; returns the status (0 = OK)."""
        with self._cv:
            self._acks.pop(ftype, None)
        self._write(encode_frame(ftype, payload))
        t_end = time.monotonic() + timeout
        with self._cv:
            while ftype not in self._acks:
                left = t_end - time.monotonic()
                if left <= 0:
                    raise can.CanOperationError(f"STM did not acknowledge command 0x{ftype:02X}")
                self._cv.wait(left)
            return self._acks.pop(ftype)

    def _handshake(self):
        for _ in range(3):
            self.pong = None
            self._write(encode_frame(CMD_PING))
            t_end = time.monotonic() + 0.7
            with self._cv:
                while self.pong is None and time.monotonic() < t_end:
                    self._cv.wait(0.05)
            if self.pong:
                break
        if not self.pong:
            raise can.CanInitializationError(
                f"no answer from the STM32 on {self.channel_info.split()[-1]} (wrong COM port? firmware running?)")
        if self.pong["proto"] < 3:
            raise can.CanInitializationError(
                f"STM firmware speaks protocol v{self.pong['proto']}; the gateway needs v3 - "
                f"flash rs05_hal/Firmware_Gateway")
        st = self._request(CMD_GATEWAY, struct.pack("<BH", 1, self.wd_ms))
        if st != 0:
            raise can.CanInitializationError(f"gateway enable refused: {ACK_STATUS.get(st, st)}")

    def _heartbeat_loop(self, period: float):
        while not self._stop.wait(period):
            try:
                self._write(encode_frame(CMD_HEARTBEAT))
            except can.CanError:
                pass

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                data = self.ser.read(max(1, getattr(self.ser, "in_waiting", 0) or 1))
            except Exception:  # noqa: BLE001 - port closed / unplugged
                if self._stop.is_set():
                    return
                time.sleep(0.05)
                continue
            if not data:
                continue
            for ftype, p in self._parser.feed(data):
                try:
                    self._dispatch(ftype, p)
                except (struct.error, IndexError):
                    pass

    def _dispatch(self, ftype: int, p: bytes):
        if ftype == EVT_CAN_RX:
            now = time.time()
            for k in range(len(p) // GW_RX.size):
                bus, flags, dlc, aid, _t_us, data = GW_RX.unpack_from(p, k * GW_RX.size)
                self._on_can(bus, flags, dlc, aid, data, now)
        elif ftype == EVT_ACK:
            cmd, st = p[0], struct.unpack("<b", p[1:2])[0]
            if cmd == CMD_CAN_TX:
                name = ACK_STATUS.get(st, str(st))
                self.nacks[name] = self.nacks.get(name, 0) + 1
            with self._cv:
                self._acks[cmd] = st
                self._cv.notify_all()
        elif ftype == EVT_GW_STATS:
            v = GW_STATS.unpack(p)
            self.gw_stats = dict(t_ms=v[0], tx_ok=list(v[1:3]), tx_drop=list(v[3:5]), rx_fwd=list(v[5:7]),
                                 usb_dropped=v[7], wd_trips=v[8], txq_peak=list(v[9:11]), txq_len=list(v[11:13]),
                                 enabled=v[13], wd_tripped=v[14])
        elif ftype == EVT_HEALTH:
            buses = []
            for b in range(2):
                v = HEALTH_BUS.unpack_from(p, 4 + 32 * b)
                buses.append(dict(init_ok=v[0], eflg=v[1], tec=v[2], rec=v[3], tx_sent=v[4], tx_fail=v[5],
                                  tx_busy=v[6], rx_count=v[7], rx_ovf=v[8], bus_off=v[9], spi_err=v[10]))
            self.health = dict(t_ms=struct.unpack_from("<I", p)[0], bus=buses)
        elif ftype == EVT_PONG:
            v = PONG.unpack(p)
            with self._cv:
                self.pong = dict(fw=v[0], ctrl_hz=v[1], n_motors=v[2], n_bus=v[3], proto=v[4])
                self._cv.notify_all()
        elif ftype == EVT_EVENT:
            motor, code = p[0], p[1]
            val = struct.unpack_from("<f", p, 2)[0]
            self.events.append(time.strftime("%H:%M:%S ") + f"STM event {EVENT_NAMES.get(code, code)} "
                               f"motor {'all' if motor == 0xFF else motor} value {val:.0f}")

    def _on_can(self, bus: int, flags: int, dlc: int, aid: int, data: bytes, now: float):
        if self.can_channel in (0, 1) and bus != self.can_channel:
            return
        dlc = min(dlc, 8)
        msg = can.Message(timestamp=now, arbitration_id=aid, is_extended_id=bool(flags & GW_FLAG_EXT),
                          dlc=dlc, data=data[:dlc], channel=bus, is_rx=True)
        self.rx_count[bus & 1] += 1
        src = frame_source(msg)
        if src is not None and src not in self.static_routes:
            prev = self.learned.get(src)
            if prev is None:
                self.learned[src] = bus
            elif prev != bus and prev != "both":
                self.learned[src] = "both"
                self.conflicts.add(src)
        try:
            self._rxq.put_nowait(msg)
        except queue.Full:
            self.rx_dropped += 1

    # -------------------------------------------------------------- python-can API
    def route(self, msg: can.Message) -> list[int]:
        if self.can_channel in (0, 1):
            return [self.can_channel]
        t = frame_target(msg)
        b = self.static_routes.get(t, self.learned.get(t)) if t is not None else None
        return [b] if b in (0, 1) else [0, 1]

    def send(self, msg: can.Message, timeout: Optional[float] = None) -> None:
        if self._stop.is_set():
            raise can.CanOperationError("STM bus is shut down")
        data = bytes(msg.data).ljust(8, b"\0")[:8]
        dlc = min(msg.dlc, 8)
        flags = GW_FLAG_EXT if msg.is_extended_id else 0
        payload = b""
        for b in self.route(msg):
            payload += GW_TX.pack(b, flags, dlc, msg.arbitration_id, data)
            self.tx_count[b] += 1
        self._write(encode_frame(CMD_CAN_TX, payload))

    def _recv_internal(self, timeout: Optional[float]):
        try:
            if timeout is None:
                return self._rxq.get(), False                 # python-can: None = block
            if timeout <= 0:
                return self._rxq.get_nowait(), False
            return self._rxq.get(timeout=timeout), False
        except queue.Empty:
            return None, False

    def _close_port(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass

    def shutdown(self) -> None:
        if not self._stop.is_set():
            try:
                self._request(CMD_GATEWAY, struct.pack("<BH", 0, 0), timeout=0.5)   # back to legacy mode
            except can.CanError:
                pass
            self._close_port()
            self._reader.join(timeout=0.5)
        super().shutdown()

    # -------------------------------------------------------------- extras
    def estop(self):
        """STM-side emergency stop: STOP to every motor it has seen on both channels."""
        self._write(encode_frame(CMD_ESTOP))

    def channel_of(self, motor_id: int):
        return self.static_routes.get(motor_id, self.learned.get(motor_id))

    def stats_text(self) -> str:
        L = [f"STM gateway: fw 0x{self.pong['fw']:06X} proto v{self.pong['proto']}" if self.pong else "STM gateway"]
        routes = {**self.learned, **self.static_routes}
        if routes:
            L.append("  ID -> channel: " + ", ".join(
                f"{i}:{'CAN' + str(b + 1) if b in (0, 1) else 'BOTH'}" for i, b in sorted(routes.items())))
        if self.conflicts:
            L.append("  CONFLICT: ID(s) " + ", ".join(map(str, sorted(self.conflicts)))
                     + " answer on BOTH channels (same ID used twice)")
        g, h = self.gw_stats, self.health
        for b in range(2):
            s = f"  CAN{b + 1}: host tx {self.tx_count[b]}  rx {self.rx_count[b]}"
            if g:
                s += f" | STM sent {g['tx_ok'][b]} dropped {g['tx_drop'][b]} queue peak {g['txq_peak'][b]}"
            if h:
                hb = h["bus"][b]
                s += (f" | no-ACK {hb['tx_fail']} rx-overflow {hb['rx_ovf']} TEC {hb['tec']} REC {hb['rec']} "
                      f"bus-off {hb['bus_off']} SPI err {hb['spi_err']}{'' if hb['init_ok'] else '  MCP2515 NOT UP'}")
            L.append(s)
        tail = f"  USB: crc errors {self._parser.crc_errors}, python rx queue drops {self.rx_dropped}"
        if g:
            tail += f", STM usb drops {g['usb_dropped']}, watchdog stops {g['wd_trips']}"
        if self.nacks:
            tail += f", CAN_TX refused {self.nacks}"
        L.append(tail)
        L += ["  " + e for e in self.events[-5:]]
        return "\n".join(L)


def open_stm(channel: str, bitrate: int = FW_BITRATE, **kw) -> StmBus:
    return StmBus(channel, bitrate=bitrate, **kw)
