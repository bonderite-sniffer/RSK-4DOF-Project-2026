
import can
import struct
import time

PORT = "COM9"
BITRATE = 1_000_000

MOTOR_ID = 0x7F
HOST_ID = 0xFD

PARAMETERS = {
     "run_mode": 0x7005,
    "limit_spd": 0x7017,
    "limit_cur": 0x7018,

    "mechPos": 0x7019,
    "iqf": 0x701A,
    "mechVel": 0x701B,
    "VBUS": 0x701C,
}

def make_read_id():
    return (0x11 << 24) | (HOST_ID << 8) | MOTOR_ID

def main():
    bus = None

    try:
        bus = can.Bus(
            interface="slcan",
            channel=PORT,
            bitrate=BITRATE,
            sleep_after_open=2
        )

        print("ROBSTRIDE PARAMETER READER")
        print(f"Motor ID: 0x{MOTOR_ID:02X}")
        print(f"Host ID: 0x{HOST_ID:02X}")

        for name, address in PARAMETERS.items():

            payload = struct.pack("<H", address) + bytes(6)

            request = can.Message(
                arbitration_id=make_read_id(),
                is_extended_id=True,
                data=payload
            )

            print(f"\nReading {name} (0x{address:04X})")

            bus.send(request, timeout=1.0)

            deadline = time.monotonic() + 1.0
            response = None

            while time.monotonic() < deadline:
                msg = bus.recv(timeout=0.1)

                if msg is None:
                    continue

                expected_id = (
                    (0x11 << 24) |
                    (MOTOR_ID << 8) |
                    HOST_ID
                )

                if (
                    msg.arbitration_id == expected_id
                    and msg.is_extended_id
                    and msg.dlc == 8
                    and msg.data[0:2] == struct.pack("<H", address)
                ):
                    response = msg
                    break

                print(
                    f"Other frame: ID=0x{msg.arbitration_id:X}, "
                    f"DATA={msg.data.hex(' ')}"
                )

            if response:
                raw = response.data[4:8]
                value = struct.unpack("<f", raw)[0]

                print(f"Raw: {response.data.hex(' ').upper()}")
                print(f"Value: {value}")

            else:
                print("No matching parameter response.")

        print("\nRead test completed.")

    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        if bus:
            bus.shutdown()

if __name__ == "__main__":
    main()
