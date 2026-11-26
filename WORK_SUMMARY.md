# RS05 4-DOF Motor Test: Work Summary

This document records the work done on `rs05_hal` and its current state.
It covers the firmware fixes, the CANable motor-test toolchain (CLI, GUI, simulator), joint-collision
protection, friction identification, CAN ID management and lost-ID recovery.

**System**

| Part | What it is |
|---|---|
| Motors | 4 × RobStride RS05 (EduLite 05), private CAN protocol, 1 Mbit/s |
| Final controller | STM32H723VGT6 (WeAct board) with 2 × MCP2515 on SPI1/SPI2. Firmware in `Firmware/` |
| Bench tester | CANable v2 (slcan firmware) connected straight from the PC to the motors, with no STM32 |

**Status at a glance**

| Area | State |
|---|---|
| Firmware torque-scale fix | Done. Not compiled on this PC (no gcc / CubeIDE here) |
| CANable CLI tests | Done. Tested on the simulator only |
| GUI: position, velocity, MIT, zero, limits, friction | Done. Tested on the simulator only |
| Joint-collision protection | Done. Tested on the simulator only |
| CAN ID setup per motor | Done. Tested on the simulator only |
| Deep scan for lost IDs | Done. Tested on the simulator only |
| Tests on real motors | **Not yet done.** This is the next step |

---

## 1. Project layout

```
rs05_hal/
├── README.md                 project overview (Indonesian)
├── WORK_SUMMARY.md           this file
├── Firmware/                 STM32H723 firmware (App/Inc, App/Src), PC simulation tests, CUBEMX_SETUP.md
├── matlab/                   RS05App.m, RS05Block.m, RS05Link.m, example_script.m (talk to the STM32)
├── python/
│   ├── rs05_host.py, rs05_tests.py, live_plot.py, plot_log.py   host tools for the STM32 pipeline
│   ├── requirements.txt      now includes python-can
│   ├── _archive/             old scripts, kept but no longer used
│   └── canable/              ← bench testing straight from the PC (everything new is here)
│       ├── rs05_can.py            protocol library (RS05 class, scan, deep scan, set ID)
│       ├── rs05_canable_test.py   CLI tests + interactive menu
│       ├── rs05_controller.py     safety supervisor (100 Hz) + friction identification
│       ├── rs05_gui.py            Tkinter GUI
│       ├── rs05_sim.py            simulated RS05 on a virtual CAN bus
│       ├── rs05_gui_config.json   joint limits and settings per motor ID (created by the GUI)
│       ├── README.md              usage (Indonesian)
│       ├── MIT_GUIDE.md           GUI and MIT/impedance test protocol
│       └── logs/                  CSV logs
└── .venv/                    Python env: python-can 4.6.1, matplotlib 3.11, numpy 2.5, pyserial 3.5
```

---

## 2. Firmware and host fixes (STM32 pipeline)

**Torque scale corrected to the RS05 value of ±5.5 Nm.** Before this fix, the firmware and tools mixed 5.5 and 6.0 Nm.

| File | Change |
|---|---|
| `Firmware/App/Inc/robstride.h` | `RS_T_MAX 5.5f` replaces `RS_T_FB_MAX 6.0` |
| `Firmware/App/Src/robstride.c` | `rs_t_cmd_max = RS_T_MAX` |
| `Firmware/App/Inc/protocol.h` | comment updated |
| `python/rs05_tests.py` | `--tscale` default is now 5.5 |
| `matlab/RS05App.m`, `RS05Block.m`, `example_script.m` | use 5.5 Nm |
| `Firmware/CUBEMX_SETUP.md` | adjusted to match |

**Still open for the STM32 step:**
- `APP_MOTOR_IDS` in `Firmware/App/Inc/app_config.h` is `{1, 2, 1, 2}`. That is two motors per bus, with IDs repeated across the two buses. If you give the motors unique IDs 1–4 with the CANable, change it to `{1, 2, 3, 4}`.
- The firmware drives CAN through 2 × MCP2515. If the board has no MCP2515 modules, the CAN layer would have to move to the H723's built-in FDCAN. That change has been suggested but is **not done**.
- The firmware type 7 (set ID) and type 22 (save) frames are marked **VERIFY** in `robstride.h`.

---

## 3. RS05 protocol reference (as implemented)

29-bit extended ID: `type << 24 | data2 << 8 | target_id`, 1 Mbit/s.

| Type | Meaning | Notes |
|---|---|---|
| 0 | get ID | reply has ID `(motor << 8) \| 0xFE`; data = 8-byte MCU UID |
| 1 | MIT / operation control | torque in ID bits 23..8; p, v, kp, kd as big-endian u16 in the data |
| 2 | feedback | p, v, τ, temp ×10; mode in bits 22..23; faults in bits 16..21 |
| 3 / 4 | enable / stop | stop with `data[0]=1` also clears faults |
| 6 | set mechanical zero | `data[0]=1` |
| 7 | set CAN ID | new ID in bits 23..16, host in bits 15..8 |
| 17 / 18 | read / write parameter | index and value are little-endian |
| 21 | fault frame | |
| 22 | save parameters to flash | |
| 23 | change baud rate | not used by the tools; this is why deep scan also tries other bitrates |
| 24 | active report on/off | |

- **Scaling:** P ±4π rad (±720°), V ±50 rad/s, T ±5.5 Nm, Kp 0–500, Kd 0–5, current 11 A max.
- **run_mode (0x7005):** 0 MIT, 1 PP, 2 speed, 3 current, 5 CSP.
- **Mode change order:** stop → write run_mode → enable → write limits → write target.
- **Key parameters:**

| Index | Name | Index | Name |
|---|---|---|---|
| 0x7006 | iq_ref | 0x701A | iqf (q-axis current) |
| 0x700A | spd_ref | 0x701C | VBUS |
| 0x7016 | loc_ref | 0x7022 | acc_rad |
| 0x7017 | limit_spd | 0x7024 | vel_max |
| 0x7018 | limit_cur | 0x7025 | acc_set |
| 0x7019 | mechPos | 0x7029 | zero_sta |

- The tools use **"MIT" = type 1 in the private protocol**. Do not switch the motor to the separate MIT protocol (type 25); the old scripts did that.

---

## 4. CANable toolchain (`python/canable/`)

### 4.1 `rs05_can.py`: protocol library

**`RS05` class** decodes everything on the bus in a background receive thread. Its methods:

| Group | Methods |
|---|---|
| Motor commands | `enable`, `stop(clear_fault)`, `set_zero`, `save_params`, `active_report` |
| Control | `mit(p, v, kp, kd, t)`, `set_mode` |
| Parameters | `write(name\|index, val)`, `request`, `read` (0.3 s timeout) |
| Feedback | `start_polling` (iqf and VBUS at 20 Hz), `wait_feedback`, `on_feedback` callback |
| IDs | `get_id`, `ping`, `scan(1..127)`, `set_can_id(new_id)` |

`set_can_id(new_id)` stops the motor, sends type 7, then checks that the motor answers on the new ID before switching to it.

**Standalone functions:**
- `deep_scan(bus, host, ids=0..255, listen, stop_probe)` finds lost IDs; see 4.7.
- `deep_scan_bitrates(...)` runs `deep_scan` at 1M, 500k, 250k and 125k.
- `open_bus(interface, channel, bitrate)` opens slcan, gs_usb or virtual.

### 4.2 `rs05_canable_test.py`: CLI tests

| Command | Tests |
|---|---|
| `scan` | IDs 1–127 |
| `scan --deep`, `scan --all-bitrates` | lost or undocumented ID |
| `monitor` | live position / speed / torque / iq / VBUS / temperature (motor stays off) |
| `params` | read the important parameters |
| `zero [--save]` | zeroing, checked against `mechPos` |
| `speed --vel …` | speed control, tracking error and current |
| `pos --mode pp\|csp --deg …` | position control |
| `mit --deg … / --spring` | MIT steps with target ramp, or spring hold |
| `current [--iq …]` | idle current draw; current mode → Kt (shaft must be locked) |
| `stop [--clear]` | stop and clear faults |
| `setid --new N` | change CAN ID |

- With no command, it opens an interactive menu.
- Every test writes a CSV to `logs/`.
- Default safety caps: 2 A, 20 rad/s, 360° step, MIT kp ≤ 50, tff ≤ 1 Nm, 70 °C.
- **The CLI has no joint-limit protection.**

### 4.3 `rs05_gui.py`: GUI

Run `python rs05_gui.py --sim` for the simulator, or `python rs05_gui.py` for real hardware.

- **Top bar:** interface (slcan / gs_usb / sim), port, bitrate, Motor ID, host ID, Connect, Scan IDs.
- **Left panel:** STOP (also `Esc`), Clear faults, Record CSV, live readouts, joint range bar, event log.
- **Bottom:** live plot of position / velocity / torque / iq.

| Tab | Contents |
|---|---|
| 1 Setup | A. zero (option to save to flash) · B. teach MIN/MAX by hand with an inset, typed limits, free shaft · C. current limit and max temperature · D. CAN ID change, scan, **deep scan** |
| 2 Position | PP / CSP; slider clamped to the limits; jog ±1/5/10°; go to 0 or the middle of the range |
| 3 Velocity | slider, presets, hold-to-jog; speed is cut before a limit |
| 4 MIT / Impedance | Kp, Kd, τff, target and ramp sliders; presets (Transparent, Damping only, Soft spring, Medium, Stiff) |
| 5 Friction | sweep parameters, run/abort, results, fit plot |

Window size adapts to the screen. Settings are saved per motor in `rs05_gui_config.json` (key `motor_<id>`).

### 4.4 `rs05_controller.py`: safety supervisor (100 Hz)

Collision protection runs in every mode:

| Protection | Value |
|---|---|
| Motion refused until limits are taught | or until *free shaft* is ticked (bare motor only, uses ±700° = feedback range ±720° minus margin) |
| Position mode | target clamped to the limits; STOP if more than 5° past a limit |
| Velocity mode | speed forced to 0 when the braking distance v²/2a plus 2° reaches a limit |
| MIT: damping zone | within 3° + v²/(2·30) of a limit, moving toward it: Kd ≥ 1.5 and outward τff removed |
| MIT: virtual wall | past a limit: spring back with Kp ≥ 40, Kd ≥ 1.5 |
| MIT: overspeed brake | \|ω\| > 8 rad/s adds damping |
| MIT: hard trip | more than 12° past a limit → STOP |
| Always | feedback lost for more than 0.3 s, motor fault, or over-temperature → STOP |

- Zeroing shifts the taught limits so they stay at the same physical place.
- Starting position mode cross-checks `mechPos` against the feedback position.
- **This is not a certified safety system.** Keep the PSU switch within reach.

### 4.5 Friction identification (tab 5, `FrictionTest`)

1. Back-and-forth velocity sweep between the joint limits.
2. Least-squares fit of `τ = Fc·sign(ω) + B·ω + offset`.
3. Breakaway test: CSP moves the joint to mid-range, then the MIT feed-forward torque is ramped up until the joint moves.

The tab checks the joint has enough range for the requested speeds (`required_range_deg`). The sweep has its own acceleration setting (default 10 rad/s²).

### 4.6 CAN ID setup per motor

New motors all ship as 0x7F (127). For each motor:
1. Put **one** motor with ID 127 on the bus and connect.
2. Go to Setup → D, enter the new ID (J1=1, J2=2, J3=3, J4=4) and press **Change ID**.
3. Repeat for the next motor.
4. Power-cycle all motors, then Scan. The result should be 1, 2, 3, 4.

The GUI checks before and after the change:
- it refuses while the motor is moving, or if the target ID is already in use;
- after sending, it checks the motor answers on the new ID;
- it reconnects automatically and moves the joint limits and settings to the new ID.

CLI equivalent: `setid --new N`.

### 4.7 Deep scan (lost / undocumented ID)

The normal scan only covers IDs 1–127 at one bitrate. Deep scan does more:
1. listens passively for 1 s;
2. sends *get ID* to every **ID 0–255**;
3. sends *stop* to the IDs that did not answer (a motor always replies to stop with feedback; this also switches off any running motor);
4. optionally repeats all of this at 1M, 500k, 250k and 125k bit/s.

For each motor found it reports the ID, the bitrate, the 8-byte MCU UID (which tells physical motors apart) and any other traffic it saw. Short 11-bit frames point to a motor switched to the MIT or CANopen protocol, which needs the RobStride software to switch back.

- **GUI:** Setup → D → **Deep scan (lost ID)…**. It fills in Motor ID and bitrate; then Connect → Change ID.
- **CLI:** `scan --deep` or `scan --all-bitrates`.

### 4.7b Protocol detection and switch-back (`rs05_protocol.py`)

An RS05 speaks only one protocol, stored in flash and applied after a power cycle. The old archived console sent type 25 with F_CMD=2 (MIT protocol), so a motor can be "stuck" in MIT mode and look dead to the private-protocol tools. The probe does not move the motor:
1. passive listen;
2. private get-ID to IDs 0–255;
3. MIT *stop* `FF×7 FD` to 11-bit IDs 1–254. The reply is MIT feedback on 11-bit ID = host ID, byte 0 = motor ID, decoded to position, velocity, torque and temperature.
4. CANopen SDO read of object 0x1000 to nodes 1–127;
5. optionally, listening during a power cycle to catch the CANopen boot-up frame 0x700+node.

**Switching back to private:**
- from MIT: command 8, 11-bit ID = motor ID, data `FF FF FF FF FF FF 00 FD`;
- from CANopen: extended frame ID 0xFFF, data `01 02 03 04 05 06 00 00`. This frame is marked VERIFY.

The motor acknowledges with its UID; a power cycle is then required. Available in the CLI (`--to-private`) and the GUI (Setup → D → *Detect protocol…*). Source: the RS05 instruction manual, sections 4.1.13, 5.10 and 6.

### 4.7c Four motors at once (`rs05_multi.py`, `rs05_multi_gui.py`)

**`BusHub`** (in `rs05_can.py`) lets several `RS05` objects share one CANable:
- one receive thread hands each frame to the motor with that ID;
- one transmit lock serialises all sends;
- it counts frames per ID and keeps every get-ID reply with its UID.

**Per-motor plan:**

| Field | Meaning |
|---|---|
| mode | pos-pp, pos-csp, vel or mit |
| steps | `value[:s]` list: deg for position/MIT, rad/s for velocity |
| dwell | seconds for steps without their own time |
| cycles | number of repeats (0 = until the time limit) |
| start delay | seconds before this motor starts |
| current limit, vmax, acc | limits for position and velocity modes |
| Kp, Kd, ramp | MIT gains and target ramp speed |
| relative | steps are relative to the start position |
| end | hold the last step, or stop |

**Check IDs:** every expected ID answers with exactly one UID (two UIDs means a duplicate ID). A scan of 1–127 flags unexpected IDs, and an addressed parameter read is done per ID. **Identify** wiggles a motor ±5° so you can see which joint has that ID.

**Test runner:**
- runs at 50 Hz with all single-motor safety kept;
- stops everything at the time limit, when VBUS drops below a threshold, or (optional) when any motor trips;
- records per-motor feedback gaps, iq peak/RMS, torque peak, temperature, VBUS min/max, transmit errors, faults and end-of-step error;
- writes `logs/multi_*.csv` and `_report.txt`.

### 4.8 `rs05_sim.py`: simulated motor

`FakeRS05` answers the private protocol on a python-can virtual bus.

| Physics | Value |
|---|---|
| Inertia J | 0.002 kg·m² |
| Coulomb friction Fc | 0.06 Nm |
| Static friction Fs | 0.09 Nm |
| Viscous friction B | 0.012 Nm·s/rad |
| Torque constant Kt | 0.3 Nm/A |
| Mechanical hard stops | ±120° |

It runs internal speed, position (PP/CSP), MIT and current loops. It handles get ID, set ID (only while disabled), zero, read/write parameters and active report.

### 4.9 STM32 as the CAN interface (`Firmware_Gateway/`, `rs05_stm.py`)

The STM32H723 (2× MCP2515, CAN1/CAN2, 2 motors each) can replace the CANable for every Python tool
(interface `stm`).
- **Firmware.** Based on the CubeIDE project `D:\Projects\STM32CubeIDE\TA_4DOF`. `main.c`, `main.h`
  and `usbd_cdc_if.c` are copied unchanged. Protocol v3 adds a gateway mode with ordered per-bus TX
  queues, batched RX forwarding (11- and 29-bit frames), a 300 ms host watchdog that STOPs every motor
  it has seen, and stats. While the gateway is on, the STM-side controller is passive. The legacy v2
  STM-side control still works when it is off. Merged in the ±5.5 Nm fix.
- **Python.** `StmBus` (a python-can bus) does handshake, gateway enable, heartbeat and ID→channel
  routing. Routing is learned from traffic; IDs not heard yet go to both channels; an ID heard on both
  channels is reported as a conflict.
- `rs05_stm_emu.py` emulates the firmware on two virtual channels, which is what `stm-sim` uses.
- **Verified:**
  - the firmware compiles and links with the CubeIDE 2.2 GCC, no warnings in our files;
  - through the emulator: ID check (both channels), routing, duplicate ID across channels, a 4-motor
    test run, the watchdog stop, and every feature of both GUIs.
- **Not yet run on the board.**

---

## 5. Problems found and fixed

| Problem | Fix |
|---|---|
| Firmware and tools mixed 5.5 and 6.0 Nm torque scales | All set to ±5.5 Nm (section 2) |
| MIT virtual wall too soft: joint reached 106–110° with a 90° limit | Stiffer wall (kp 40, kd 1.5), speed-dependent damping zone, overspeed brake. Retest held the joint at 89.55° |
| Friction range check failed for a 3 rad/s top speed | Separate sweep acceleration (10 rad/s²) and `required_range_deg` |
| `current --max-iq` validated too late | Check moved before the idle measurement |
| Startup could print NaN VBUS | Waits for the first VBUS reading |
| Normal scan could not find a motor with an undocumented ID | Deep scan (4.7) |
| A motor at rest read -0.0008 rad/s and -0.011° instead of 0. The firmware truncates when encoding; we decoded without a half-step correction (confirmed in the real log) | `u2f_fb()` takes the middle of each step. The sim now encodes by truncation, like the motor |
| Velocity plot looked like heavy jitter: autoscale zoomed in on ±0.085 rad/s spikes, which are the motor's speed estimate jumping when the encoder moves one count | Median-of-5 velocity line plus the raw line, a minimum y-span on every plot, and the median also used in the readouts |
| MIT near limits: wall damping was decided from the sign of the noisy speed, so Kd toggled between 0.5 and 1.5 (chatter) | `VEL_DEADBAND` 0.2 rad/s |
| The RS02 used RS05 scaling (its torque read ~3× too low, MIT v_des 12 % off) | `MODELS` table, model chosen per motor in `JointConfig.model` and in the multi plan *Model* column |

---

## 6. Verification so far (simulator only)

| Check | Result |
|---|---|
| Motion before limits are taught | refused |
| Position target beyond a limit | clamped to the limit |
| Velocity toward a limit | stopped at the limit |
| MIT push past a 90° limit | held at 89.55° |
| Friction fit | Fc 0.0599 (true 0.06), B 0.0121 (true 0.012), breakaway 0.093 (true 0.09) |
| CAN ID change | duplicate ID refused; 0x7F → 3 worked; limits moved to `motor_3`; scan showed 2 and 3 |
| Deep scan | motors at IDs 0, 127 and 200: normal scan found only 127, deep scan found all three (≈3 s); 11-bit traffic reported |
| GUI lost-ID flow | motor at ID 200 found → connected → re-IDed to 3 |
| Protocol probe | sim motors in private / MIT / CANopen each identified; mixed bus (MIT 0x7F + CANopen 2 + private 3) all identified; both switch-backs acknowledged, all private after power cycle; CANopen boot-up caught |
| 4-motor run | 4 sim motors (pos-csp ×2, vel, MIT) ran together; ID check PASS; report and CSV written; no feedback gaps |
| 4-motor fault cases | duplicate ID 2 → 2 UIDs reported, leftover 0x7F flagged → FAIL; one motor unplugged mid-test → "feedback lost" trip in 0.3 s, all motors stopped |
| GUI protocol flow | motor stuck in MIT detected → switched → power-cycled → connected in private |

**Nothing has been run on a real RS05 yet.** Thresholds and default gains are starting points.

---

## 7. Documentation written

| File | Contents |
|---|---|
| `README.md` (root) | project overview, updated for the CANable workflow |
| `python/canable/README.md` | setup, GUI tabs, CAN ID setup, deep scan, CLI test order, protocol notes (Indonesian) |
| `python/canable/MIT_GUIDE.md` | GUI walkthrough and MIT test protocol T1–T8 with a results table |
| `WORK_SUMMARY.md` | this file |

Background on questions answered along the way:
- **The 700° limit** is `FREE_LIMIT_DEG` in `rs05_controller.py`. It only applies in *free shaft* mode and keeps the motor inside the ±720° range the feedback can report. It is not a joint limit for the robot.
- **How to run:** activate `.venv`, `cd python\canable`, then `python rs05_gui.py --sim` to practise or `python rs05_gui.py` for real hardware.

---

## 8. Next steps

1. **Real-motor bring-up, one motor at a time, on the CANable**, with a 1–2 A PSU current limit:
   1. Scan (deep scan if needed).
   2. Monitor.
   3. Zero, then check it survives a power-cycle.
   4. Teach limits.
   5. Position, then velocity, then MIT tests T1–T8 from `MIT_GUIDE.md`.
   6. Friction test.
2. **Assign IDs 1–4** and confirm they persist after a power-cycle.
3. **Record the real results:** Kt, friction (Fc, B, breakaway), good Kp/Kd per joint, gravity torque per link.
4. **Move to the STM32:**
   - set `APP_MOTOR_IDS`;
   - decide MCP2515 vs native FDCAN;
   - check the type 7 / type 22 frames marked VERIFY;
   - compile and flash (needs STM32CubeIDE or arm-none-eabi-gcc on the PC).
