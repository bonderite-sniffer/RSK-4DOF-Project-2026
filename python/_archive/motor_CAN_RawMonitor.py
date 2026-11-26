
import can
import time
from collections import Counter

PORT = "COM9"
BITRATE = 1_000_000

MOTOR_ID = 0x7F
HOST_ID = 0xFD

def main():
    bus = None

    try:
        bus = can.Bus(
            interface="slcan",
            channel=PORT,
            bitrate=BITRATE,
            sleep_after_open=2
        )

        print("Connected to COM9")
        print("Raw CAN monitor - 10 seconds")
        print("No motor commands will be transmitted.\n")

        start = time.monotonic()
        frames = []
        ids = Counter()

        while time.monotonic() - start < 10:

            msg = bus.recv(timeout=0.2)

            if msg is None:
                continue

            now = time.monotonic()
            frames.append((now, msg))
            ids[msg.arbitration_id] += 1

            comm_type = (msg.arbitration_id >> 24) & 0x1F
            motor = (msg.arbitration_id >> 8) & 0xFF
            host = msg.arbitration_id & 0xFF

            print(
                f"ID=0x{msg.arbitration_id:08X} "
                f"TYPE={comm_type} "
                f"MOTOR=0x{motor:02X} "
                f"HOST=0x{host:02X} "
                f"EXT={msg.is_extended_id} "
                f"DATA={msg.data.hex(' ').upper()}"
            )

        print("\n========== SUMMARY ==========")
        print(f"Total frames: {len(frames)}")
        print("\nFrames by CAN ID:")

        for can_id, count in ids.items():
            print(f"0x{can_id:08X}: {count}")

        if len(frames) > 1:
            intervals = [
                (frames[i][0] - frames[i-1][0]) * 1000
                for i in range(1, len(frames))
            ]

            print(f"\nMean interval: {sum(intervals)/len(intervals):.3f} ms")
            print(f"Observed rate: {len(frames)/10:.2f} frames/s")

    except Exception as e:
        print("ERROR:", e)

    finally:
        if bus:
            bus.shutdown()

if __name__ == "__main__":
    main()
