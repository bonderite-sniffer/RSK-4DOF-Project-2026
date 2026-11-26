
import can
import time

PORT = "COM9"
BITRATE = 1_000_000

MOTOR_ID = 0x7F
HOST_ID = 0xFD

# Communication Type 6: Set Zero Position
comm_type = 6

can_id = (
    (comm_type << 24)
    | (HOST_ID << 8)
    | MOTOR_ID
)

msg = can.Message(
    arbitration_id=can_id,
    is_extended_id=True,
    data=[0x01, 0x02, 0x03, 0x04,
          0x05, 0x06, 0x07, 0x08]
)

bus = None

try:
    bus = can.Bus(
        interface="slcan",
        channel=PORT,
        bitrate=BITRATE,
        sleep_after_open=2
    )

    print("RS05 ZERO POSITION")
    print(f"CAN ID: 0x{can_id:X}")
    print("WARNING: This changes the position reference.")

    confirm = input(
        "Ensure motor is stationary. Type ZERO to continue: "
    )

    if confirm == "ZERO":
        bus.send(msg)
        print("Zero-position command transmitted.")
        time.sleep(0.2)
    else:
        print("Cancelled.")

finally:
    if bus:
        bus.shutdown()
