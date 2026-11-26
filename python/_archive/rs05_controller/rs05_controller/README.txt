ROBSTRIDE RS05 PYTHON TERMINAL CONTROLLER
========================================

FILES
-----
rs05_console.py : single-file terminal controller and telemetry logger

INSTALL (Windows PowerShell)
----------------------------
1) Activate your venv.
2) Install python-can:
       py -m pip install python-can
3) Confirm CANable V2 is running SLCAN firmware and appears as COM9.
4) Run:
       py rs05_console.py

DEFAULTS
--------
CAN adapter : CANable V2 / SLCAN
COM port    : COM9
CAN bitrate : 1 Mbps
Motor ID    : 0x7F
Host ID     : 0xFD
Supply      : 48 V nominal (RS05 published input range 15-60 V)

SUPPORTED PRIVATE-PROTOCOL OPERATIONS
--------------------------------------
- Speed mode (run_mode=2; spd_ref 0x700A)
- Position PP mode (run_mode=1; loc_ref 0x7016)
- Active reporting (type 24)
- Disable/stop (type 4)
- Change CAN bitrate (type 23: 1M/500k/250k/125k; power cycle required)
- Switch to MIT protocol (type 25; power cycle required)

MIT
---
The program includes standard 8-byte MIT command packing and a 50 Hz command
stream. MIT support is firmware/protocol-revision sensitive. Verify the exact
RS05 manual's MIT frame-ID and feedback-ID rules before using the MIT menu.
Do not arm if the program does not decode valid live feedback first.

IMPORTANT BITRATE NOTE
----------------------
Changing the motor bitrate is not just a software adapter setting. The motor
is configured by communication type 23 and the new setting takes effect after
power cycle. The CANable bitrate must then match the motor. If communication is
lost, reconnect at the previous known bitrate or restore the motor's known
configuration using a verified procedure.

SAFETY
------
- This is experimental bench software, not a safety-rated controller.
- The first-motion caps are deliberately conservative:
    private speed <= 1.0 rad/s
    position target within +/-0.25 rad of measured position
    phase current limit <= 1.0 A
    acceleration <= 1.0 rad/s^2
    MIT speed <= 0.5 rad/s, torque <= 0.5 Nm, Kp <= 20, Kd <= 1
- These are software command caps, not guarantees of physical torque/speed.
- Set PSU input current limit conservatively (e.g. begin around 1-2 A),
  recognizing PSU input current differs from motor phase-current limit.
- Secure the motor, clear the shaft/coupling area, and have a physical power
  disconnect. Ctrl+C and software disable are not emergency-stop systems.
- Never change bitrate/protocol while the motor is moving or enabled.
- Do not change mechanical zero or save parameters during initial tests.

CSV
---
Telemetry is written to logs/rs05_telemetry.csv relative to the working folder.

PROTOCOL SOURCES / CAVEATS
--------------------------
Private protocol communication types, parameter IDs, mode sequencing and
RS05 model ranges are based on RobStride protocol/sample references. The exact
firmware/manual revision shipped with your RS05 remains authoritative.
The decoder uses the RS05-specific mapping listed in RobStride's EDULITE_A3
protocol reference: P=+/-12.57 rad, V=+/-50 rad/s, T=+/-5.5 Nm, Kp=0..500,
Kd=0..5. Do not substitute scaling tables for other RS models. Revalidate
telemetry against parameter reads/manual before closed-loop use.
