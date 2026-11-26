# RS05 GUI & MIT / Impedance Test Guide

How to use `rs05_gui.py` with a CANable v2, and a step-by-step protocol for testing the RS05 in
MIT (impedance) mode before moving to the STM32 firmware.

---

## 1. Start the GUI

```powershell
cd D:\College\Semester_7\Semprol\4DOF\Motor_Test\rs05_hal
.venv\Scripts\Activate.ps1
cd python\canable

python rs05_gui.py --sim     # simulated motor (no hardware) - practise here first
python rs05_gui.py           # real motor via CANable
```

Top bar: **Interface** `slcan` (or `gs_usb`, port `0`, for candleLight firmware) · **Port** e.g. `COM9`
· **Bitrate** `1000000` · **Motor ID** (`0x7F` from the factory) · **Connect**. Use **Scan IDs** to see
which motors answer.

### Window layout

| Area | What it does |
|---|---|
| Left panel | **STOP** (also `Esc`), *Clear faults*, *Record CSV*, live readouts (pos, vel, torque, iq, VBUS, temp, mode, faults), joint range bar (green = allowed, red = forbidden, marker = joint), event log |
| Tab 1 Setup | Zero, teach joint limits, current / temperature limits, change CAN ID |
| Tab 2 Position | PP / CSP position control |
| Tab 3 Velocity | Speed control with automatic stop before limits |
| Tab 4 MIT / Impedance | Kp, Kd, τff, target, ramp sliders + presets |
| Tab 5 Friction | Friction identification (Coulomb + viscous + breakaway) |
| Bottom | Live plot of position / velocity / torque / iq |

---

## 2. Before any motion (do once per motor)

1. **Power**: 48 V PSU, current limit 1-2 A for the first tests. Keep a hand on the PSU switch -
   the GUI STOP is software only.
2. **CAN**: 120 Ω termination at both ends, 1 Mbit/s.
3. **CAN ID** (multi-motor only): tab 1 → *D. Motor CAN ID*, one motor at a time. See `README.md`.
4. **Zero**: move the joint to the reference pose by hand (motor off) → tab 1 → *Set zero HERE*.
   Tick *also save in motor flash* if you want it stored in the motor; check it survives a power-cycle.
5. **Teach limits**: motor off, move the joint by hand to one mechanical end → *◀ Set MIN here*,
   other end → *Set MAX here ▶*. The limit is stored `inset` degrees before the touch point.
   **Every mode refuses to move until limits are taught** (or *Free shaft* is ticked for a bare motor).
6. Click *Record CSV* whenever you want a log in `logs/`.

---

## 3. MIT / impedance mode - what the motor does

Every control cycle the motor computes

```
τ = Kp·(θd − θ) + Kd·(ωd − ω) + τff
```

| Symbol | Slider | Unit | GUI range | Meaning |
|---|---|---|---|---|
| Kp | *Kp stiffness* | Nm/rad | 0 - 60 (config `mit_kp_max`) | spring stiffness. 1 Nm/rad ≈ 0.0175 Nm/° |
| Kd | *Kd damping* | Nm·s/rad | 0 - 5 | viscous damping |
| τff | *τff feed-forward* | Nm | ±1 (config `mit_tff_max`) | constant torque added (gravity comp, friction tests) |
| θd | *θd target* | deg | clamped to the joint limits | equilibrium point of the spring |
| ramp | *target ramp* | rad/s | 0.1 - 10 | θd moves to the slider value at this speed (no step torque spike); ωd = ramp while moving, 0 when arrived |

The GUI sends this as a type-1 frame at **100 Hz** from the supervisor loop
(`rs05_controller.py`). All sliders act live once MIT is started.

### Buttons
- **START MIT (hold here)** - switches the motor to run_mode 0 and sets θd = current position, so
  nothing jumps on start.
- **Target = here** - moves θd to where the joint is now (useful after pushing it).
- **Stop** / `Esc` - disables the motor (it goes limp).

### Presets

| Preset | Kp | Kd | Use |
|---|---|---|---|
| Transparent | 0 | 0.02 | joint is (almost) free - feel friction and cogging |
| Damping only | 0 | 0.3 | feels like moving through honey; no return to target |
| Soft spring | 3 | 0.15 | gentle return; safe first spring test |
| Medium | 10 | 0.5 | normal holding |
| Stiff | 40 | 1.0 | firm hold; only after the softer tests behave |

### Live readout (tab 4)
`θd`, `θ`, *τ model* = Kp·(θd−θ)+Kd·(0−ω)+τff computed by the GUI, *τ meas* = motor torque feedback,
*stiffness* in Nm/°, plus **VIRTUAL WALL** / **OVERSPEED BRAKE** when those protections act.

### MIT safety layers (always on in the GUI)

| Layer | Trigger | Action |
|---|---|---|
| Target clamp | θd outside the limits | θd clamped to the limit |
| Damping zone | moving toward a limit and within 3° + v²/(2·30 rad/s²) of it | Kd raised to ≥ 1.5, τff pushing toward the limit removed |
| Virtual wall | joint past a limit | spring to the limit with Kp ≥ 40, Kd ≥ 1.5, τff = 0 |
| Overspeed brake | \|ω\| > 8 rad/s (`mit_vel_max`) | same extra damping as the zone |
| Hard trip | joint > 12° past a limit | **STOP** (motor disabled) |
| Feedback lost > 0.3 s, motor fault, temp > `temp_max` | | **STOP** |

> The CLI `rs05_canable_test.py mit` has **no** joint-limit protection - only kp/tff caps.
> Use the GUI whenever a link is attached.

---

## 4. MIT test protocol

Do the tests in order. Pass each one before going to the next. Run everything once on `--sim`
first, then on the real motor with the **bare shaft**, then with the link attached.
Start *Record CSV* before each test and note the file name.

### T1 - Transparent (friction feel)
1. Preset **Transparent**, **START MIT**.
2. Turn the joint slowly by hand across the range.

Pass: joint moves freely, *τ meas* stays small (≈ friction level, roughly < 0.1 Nm on a bare shaft),
no faults. Near each limit you should feel the damping zone, then the wall.

### T2 - Damping
1. Preset **Damping only** (Kd 0.3). Then try Kd 0.1 and 1.0.
2. Move the joint at different speeds.

Pass: resistance grows with speed, joint stays where you leave it. *τ meas* ≈ −Kd·ω
(check on the plot: torque opposes velocity).

### T3 - Stiffness check (static)
1. Preset **Soft spring** (Kp 3), **START MIT** at a mid position.
2. Push the joint a known angle Δθ (e.g. 10°) and hold still.
3. Compare *τ meas* with *τ model* = Kp·Δθ (10° = 0.175 rad → 0.52 Nm at Kp 3).
4. Repeat for Kp 10 and Kp 20.

Pass: τ meas within ~10-15 % of τ model (difference is friction + torque estimate error). Plot
τ vs Δθ for the three Kp values: slopes should equal Kp. Record the numbers in the results table.

### T4 - Release / return (dynamic)
1. Kp 10, Kd 0.5 (Medium). Push the joint ~20°, release.
2. Repeat with Kd 0.1, 0.3, 1.0.

Pass: joint returns to θd. Low Kd → overshoot/oscillation, high Kd → slow creep, no overshoot.
Pick the smallest Kd without visible overshoot for each Kp. Theory: critical damping
Kd ≈ 2·√(Kp·J), so if you double Kp, Kd should go up by about √2.

### T5 - Target tracking (step via ramp)
1. Kp 10, Kd 0.5, ramp 1 rad/s.
2. Move the *θd target* slider by +30°, then −30°, then back to 0.
3. Repeat with ramp 2 and 4 rad/s, then with Kp 20.

Measure from the CSV: tracking lag during the ramp, overshoot, steady-state error at the end.
Pass: no overshoot past the limits, steady-state error ≈ friction/Kp
(e.g. 0.06 Nm / 10 = 0.006 rad ≈ 0.3°).

### T6 - Feed-forward / gravity compensation (link attached)
1. Kp 0, Kd 0.3. Slowly increase *τff* until the link just holds against gravity at a horizontal pose.
2. That τff ≈ m·g·r (gravity torque at horizontal). Check at other poses: needed τff ≈ m·g·r·cos θ.

Pass: the measured hold torque follows a cosine. This value is what the STM32 controller will need for
gravity compensation. On a bare shaft, use τff instead to find the breakaway torque (the joint starts
moving when τff exceeds static friction - tab 5 does this automatically).

### T7 - Protection check (do on --sim first, then real motor at low Kp)
1. Limits e.g. ±60°, Kp 3, Kd 0.15.
2. Drag the target slider to the limit → θd stops at the limit.
3. Push the joint by hand toward a limit → damping zone, then **VIRTUAL WALL** in the readout
   and event log, joint pushed back.
4. Press `Esc` while moving → motor goes limp immediately.

Pass: all three appear in the event log, no trip, joint never more than a few degrees past the limit.

### T8 - Stiff hold (last)
Preset **Stiff** (Kp 40, Kd 1.0), push hard by hand. Watch temperature and iq.
Pass: holds without oscillation or audible buzz. If it buzzes, lower Kd first, then Kp.

---

## 5. Results table (copy per motor)

| Test | Motor ID | Kp | Kd | τff | Measured | Expected | OK? | CSV file |
|---|---|---|---|---|---|---|---|---|
| T1 friction feel | | 0 | 0.02 | 0 | τ ≈ | < 0.1 Nm | | |
| T3 stiffness | | 3 | 0.15 | 0 | τ @10° = | 0.52 Nm | | |
| T3 stiffness | | 10 | 0.5 | 0 | τ @10° = | 1.75 Nm | | |
| T4 best Kd | | 10 | | 0 | overshoot = | none | | |
| T5 steady error | | 10 | 0.5 | 0 | err = | < 1° | | |
| T6 gravity torque | | 0 | 0.3 | | τff hold = | m·g·r | | |
| T7 wall | | 3 | 0.15 | 0 | max overshoot past limit = | < 5° | | |

---

## 6. CLI equivalents (scripted, repeatable logs)

```powershell
# steps with ramp; prints rise time, error, torque, iq per target
python rs05_canable_test.py -p COM9 mit --deg 30 -30 0 --kp 10 --kd 0.5 --vmax 2 --hold 2
# relative steps from the current position
python rs05_canable_test.py -p COM9 mit --rel --deg 10 -10 0 --kp 5 --kd 0.2
# spring: hold here for 15 s, push by hand
python rs05_canable_test.py -p COM9 mit --spring --kp 5 --kd 0.2 --time 15
```
Options: `--tff`, `--rate` (Hz, default 100), `--max-kp` (default 50), `--max-tff` (default 1 Nm),
`-y` to skip the confirm prompt. **No joint-limit protection in the CLI** - bare shaft or wide-open
range only.

---

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| "Teach the joint limits first" | Tab 1, teach MIN/MAX (or tick *Free shaft* with nothing attached) |
| START MIT refused, "outside the limits" | Joint is > 5° outside the taught range - move it back by hand, or re-zero/re-teach |
| STOP "feedback lost" | CAN wiring / termination, wrong COM port, wrong motor ID, PSU off |
| Motor fault in readout | Read the fault name in the log, fix cause, *Clear faults* |
| Buzzing / oscillation | Kd too high for the loop (noise) or Kp too high - lower Kd, then Kp |
| Joint creeps, does not reach θd | Friction > Kp·error - normal at low Kp; raise Kp or add τff |
| No feedback at all, scan finds nothing | Motor may be switched to the "MIT protocol" (type 25) by old scripts - switch back to private protocol with RobStride software |
| τ meas ≠ τ model a lot | Check zero / sign; at standstill friction alone can be ±0.06-0.1 Nm |
