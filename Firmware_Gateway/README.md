# Firmware_Gateway: STM32H723 + 2× MCP2515 as the CAN interface for the Python tools

**What the STM32 does with this firmware**

| Task | Who does it |
|---|---|
| Drives CAN1 (SPI1 → MCP2515) and CAN2 (SPI2 → MCP2515), 2 motors per channel, 1 Mbit/s | STM32 |
| Sends each frame Python asks for, **in order**, on the right channel | STM32 |
| Forwards every frame received on CAN1/CAN2 (feedback, parameter replies, ID replies) to Python | STM32 |
| Watchdog: Python silent for more than 300 ms → STOP to every motor it has seen | STM32 |
| CAN / SPI / USB health counters (no-ACK, RX overflow, TEC/REC, bus-off, drops) | STM32 → Python |
| Modes, setpoints, gains, limits, ramps, safety supervisor, tests, GUIs | Python (unchanged) |

Python decides everything; the STM only executes it. All the existing Python tools work through the STM
by picking the interface **`stm`** instead of `slcan`. The CANable path (`slcan`, `gs_usb`) is still there,
unchanged.

```
 Python (rs05_gui / rs05_multi_gui / CLI)            STM32H723 (this firmware)               motors
   RS05 + Controller  ──CMD_CAN_TX (bus,id,data)──►  per-bus queue ─► MCP2515 #1 ─ CAN1 ─►  J1, J2
   StmBus (rs05_stm.py) ◄──EVT_CAN_RX (bus,id,data,t)─ RX batch  ◄── MCP2515 #2 ─ CAN2 ─►  J3, J4
                         ──heartbeat 10 Hz──────────►  watchdog → STOP all
```

---

## 1. What is in this folder

```
Firmware_Gateway/
  Core/Src/main.c, Core/Inc/main.h    copied unchanged from D:\Projects\STM32CubeIDE\TA_4DOF (your CubeIDE project)
  USB_DEVICE/App/usbd_cdc_if.c        copied unchanged (already calls usb_link_rx_isr)
  Core/Src|Inc/gateway.c/.h           NEW: the USB <-> CAN bridge
  Core/Src|Inc/*                      the app from TA_4DOF, with these changes:
```

| File | Change |
|---|---|
| `protocol.h` | Protocol **v3**, an addition to v2 that leaves the v2 commands as they were: `CMD_GATEWAY` 0x50, `CMD_CAN_TX` 0x51, `EVT_CAN_RX` 0x87, `EVT_GW_STATS` 0x88, `gw_stats_t`. FW version 3.0.0 |
| `can_bus.c` | Every received frame, 29-bit **and** 11-bit, goes to `gw_on_rx()` |
| `app.c` | `gw_init()`, and `gw_poll()` every main-loop pass |
| `command.c` | New commands. While the gateway is on, the STM-side motion and parameter commands (`CMD_POS/VEL/MIT/CALIB/SCAN/READ/WRITE_PARAM`) answer BUSY. `CMD_ESTOP` also stops every motor the gateway has seen |
| `motor_ctrl.c` | Passive while the gateway is on. **Important:** in IDLE it normally sends STOP to each motor every 20 ms, which would disable the motors Python enables |
| `telemetry.c` | No 100 Hz `EVT_TELEM` while the gateway is on (Python gets the raw frames); health stays at 5 Hz |
| `robstride.c/.h` | ±5.5 Nm torque scale (the fix from `rs05_hal/Firmware`). TA_4DOF still had 6.0 / 120 |
| `app_config.h` | `APP_MOTOR_IDS {1,2,3,4}` (J1, J2 on CAN1, J3, J4 on CAN2), `APP_BOOT_GATEWAY`, `APP_GW_WD_MS`. Still **8 MHz** MCP2515 crystals, as in TA_4DOF |
| `mcp2515.c` | Initialised a buffer (`-Wmaybe-uninitialized`) |

## 2. Put it into the CubeIDE project

Safest: work on a copy of TA_4DOF (`File → Copy/Paste` the project, or copy the folder), then:

1. Copy `Firmware_Gateway/Core/Inc/*` → `<project>/Core/Inc/` and `Firmware_Gateway/Core/Src/*` →
   `<project>/Core/Src/`. This overwrites the app files and adds `gateway.c` / `gateway.h`.
   `main.c` / `main.h` are identical to TA_4DOF's, so overwriting them changes nothing.
2. `usbd_cdc_if.c` is identical too; nothing to do there.
3. Delete the old `mcp2515.c.bak` / `usb_link.c.bak` from `Core/Src` if CubeIDE tries to build them
   (normally it does not: `.bak` is not compiled).
4. Build (hammer) → Run (ST-Link).
5. If you regenerate code in CubeMX later: our code is only in our own files plus the USER CODE blocks
   (`app_init()` / `app_loop()` in `main.c`, `usb_link_rx_isr()` in `usbd_cdc_if.c`), so it survives.

**Verified here:** the folder compiles and links with arm-none-eabi-gcc 14.3 (the CubeIDE 2.2 toolchain)
against TA_4DOF's HAL, USB and startup files, with `-Wall -Wextra` and no warnings in these files.
Flash 43 KB, RAM 27 KB. It has **not run on the board yet**.

## 3. Use it from Python

```powershell
cd rs05_hal\python\canable
python rs05_gui.py                 # Interface: stm, Port: the STM's COM port (USB-C), Connect
python rs05_multi_gui.py           # Interface: stm, Port: STM COM port -> Connect -> Check IDs -> RUN
python rs05_canable_test.py --iface stm -p COM8 scan
python rs05_multi.py --iface stm -p COM8 --check-ids --ids 1 2 3 4
python rs05_protocol.py --iface stm -p COM8          # private / MIT / CANopen probe through the STM

# no hardware: simulated STM with simulated motors
python rs05_gui.py                 # Interface: stm-sim
python rs05_multi_gui.py           # Interface: stm-sim  (first half of the plan IDs on CAN1, rest on CAN2)
python rs05_multi.py --iface stm-sim --check-ids --ids 1 2 3 4
```

**Which channel a frame goes to.** `StmBus` learns the channel from the frames each motor sends
(ID → CAN1/CAN2). A frame for an ID not heard yet, such as scans or the first STOP, goes to **both**
channels. That is harmless, because only the motor with that ID reacts.

If the **same ID answers on both channels**, it is reported as a conflict:
- in the single-motor GUI, on the connection line;
- in the multi-motor GUI, in the event log and the report;
- by *Check IDs*, which shows DUPLICATE with 2 UIDs.

Give every motor a unique ID (1..4).

**Bitrate.** The firmware runs CAN at **1 Mbit/s only** (`MCP2515_BITRATE`), so "all bitrates" deep
scans are disabled for `stm`.

**Report / status.**
- The single-motor GUI's connection line shows the channel the motor answers on, plus warnings: ID
  conflict, watchdog stops, dropped frames, MCP2515 not up.
- Multi-motor test reports get an extra *STM gateway* block, per channel: frames sent, dropped, no-ACK,
  RX overflow, TEC/REC, bus-off, SPI errors. USB drops and watchdog stops are listed too.
- From your own script, call `bus.stats_text()` on an `StmBus`.

**Legacy STM-side control** (`python/rs05_host.py`, MATLAB `RS05Link.m`, protocol v2) still works, as
long as the gateway is off. The gateway is off after reset and after a Python `StmBus` closes cleanly.

## 4. Protocol v3 additions (little endian, same framing as v2: `A5 5A type len payload crc16`)

| Message | Dir | Payload |
|---|---|---|
| `CMD_GATEWAY` 0x50 | host→MCU | `u8 enable, u16 wd_ms` (0 = no watchdog) → `EVT_ACK` |
| `CMD_CAN_TX` 0x51 | host→MCU | 1..8 × `{u8 bus, u8 flags, u8 dlc, u32 id, u8 data[8]}` (15 B). ACK only on error: BAD_LEN, BUSY (gateway off), BAD_ARG, CAN_ERR (queue full / MCP2515 not up) |
| `EVT_CAN_RX` 0x87 | MCU→host | 1..6 × `{u8 bus, u8 flags, u8 dlc, u32 id, u32 t_us, u8 data[8]}` (19 B) |
| `EVT_GW_STATS` 0x88 | MCU→host | `gw_stats_t` (44 B) every 200 ms while the gateway is on |
| `EVT_EVENT` code 7 | MCU→host | watchdog fired (value = ms of silence) |

`flags` bit0 = 29-bit ID. RTR is rejected. `bus` 0 = CAN1, 1 = CAN2.

**Ordering.** A queued frame goes to the MCP2515 only when all 3 TX buffers are empty. Otherwise the chip
can send buffer 2 before buffer 0 and swap e.g. `run_mode` and `ENABLE`. With one frame in flight at a
time, each channel can still carry about 6000 frames/s, and 4 motors use about 600.

**Bandwidth (4 motors, active report 100 Hz, MIT 100 Hz, iq/VBUS polls).** About 1000 frames/s up
(≈21 KB/s) and about 400 frames/s down (≈8 KB/s), which is far below USB-FS CDC.

## 5. Live Expressions

| Expression | Meaning |
|---|---|
| `g_gw.enabled`, `g_gw.wd_trips`, `g_gw.wd_tripped` | gateway state, watchdog |
| `g_gw.tx_ok[2]`, `g_gw.tx_drop[2]`, `g_gw.rx_fwd[2]`, `g_gw.txq_peak[2]` | per channel |
| `g_gw.seen` | bitmap of IDs heard per channel (the IDs the watchdog will STOP) |
| `g_can[0].mcp.st`, `g_can[1].mcp.st` | MCP2515 health (tx_fail = no ACK, rx_ovf, tec/rec, bus_off) |
| `g_motor[i]`, `g_live` | still decoded from the frames Python triggers (IDs from `APP_MOTOR_IDS`) |
| `g_diag[0..1]` | MCP2515 self-test at boot (unchanged) |

## 6. Things to know

- **8 MHz MCP2515 crystals at 1 Mbit/s are out of spec.** The bit timing has only 4 time quanta (see
  `mcp2515.c`). It works on a short bench bus. If `no-ACK`, `rx_ovf` or TEC/REC climb under load, the
  cure is 16 MHz crystals plus `MCP2515_OSC_HZ 16000000u`.
- **The MCP2515 has only 2 RX buffers.** The main loop drains them; if `rx-overflow` grows in the report,
  the loop is too slow. Watch `g_app.loop_count`; it should be well above 10 000 per second.
- **Boot.** The legacy `mc_init()` still writes `zero_sta = 1` to the IDs in `APP_MOTOR_IDS` at power-up,
  so `mechPos` is reported in −π..π. Feedback position (±4π) is not affected.
- **Watchdog.** 300 ms. `StmBus` sends a heartbeat every 100 ms from its own thread. If Python crashes or
  the USB cable is pulled, the STM sends STOP to every motor it has seen within about 0.3 s.
- `APP_BOOT_GATEWAY 1` makes the STM start in gateway mode without a host command. Not needed with
  `StmBus`, which enables it on connect.
