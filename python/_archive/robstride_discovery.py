
import can
import time

PORT = "COM9"
BITRATE = 1_000_000

# RobStride private protocol:
# Communication type = 0
# Host ID = 0xFD
# Destination = 0x7F (discovery)

DISCOVERY_ID = 0x00FD7F

def main():
    bus = None

    try:
        print("Connecting to MKS CANable 2.0...")

        bus = can.Bus(
            interface="slcan",
            channel=PORT,
            bitrate=BITRATE,
            sleep_after_open=2
        )

        print("Connected.")
        print("Sending read-only device discovery request...")

        request = can.Message(
            arbitration_id=DISCOVERY_ID,
            is_extended_id=True,
            data=bytes(8)
        )

        bus.send(request, timeout=1.0)

        print(f"TX ID: 0x{DISCOVERY_ID:08X}")
        print("TX DATA: 00 00 00 00 00 00 00 00")
        print("\nWaiting for motor response...\n")

        deadline = time.monotonic() + 3.0
        found = False

        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.2)

            if msg is None:
                continue

            found = True

            print(
                f"RX ID: 0x{msg.arbitration_id:08X} | "
                f"{'EXT' if msg.is_extended_id else 'STD'} | "
                f"DATA: {msg.data.hex(' ').upper()}"
            )

        if not found:
            print("\nNO RESPONSE.")
            print("Check motor power, CAN wiring, termination")
            print("and adapter transmit capability.")

        else:
            print("\nCAN response detected!")

    except Exception as error:
        print(f"ERROR: {error}")

    finally:
        if bus is not None:
            bus.shutdown()


if __name__ == "__main__":
    main()
