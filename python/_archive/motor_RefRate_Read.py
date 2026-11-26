
import can
import time

bus = None

try:
    bus = can.Bus(
        interface="slcan",
        channel="COM9",
        bitrate=1_000_000,
        sleep_after_open=2
    )

    # Type 24: Enable active reporting
    msg = can.Message(
        arbitration_id=0x1800FD7F,
        is_extended_id=True,
        data=[0x01, 0x02, 0x03, 0x04,
              0x05, 0x06, 0x01]
    )

    bus.send(msg)
    print("Type 24 active reporting command sent.")

    print("\nListening for motor feedback...")

    start = time.monotonic()
    count = 0

    while time.monotonic() - start < 5:
        rx = bus.recv(timeout=0.1)

        if rx is not None:
            count += 1
            print(
                f"ID=0x{rx.arbitration_id:X} "
                f"EXT={rx.is_extended_id} "
                f"DATA={rx.data.hex(' ').upper()}"
            )

    print(f"\nTotal received: {count}")

except Exception as e:
    print("ERROR:", repr(e))

finally:
    if bus:
        bus.shutdown()
