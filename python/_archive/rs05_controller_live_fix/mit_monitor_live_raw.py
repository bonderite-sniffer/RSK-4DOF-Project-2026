
import can
import time
from collections import Counter

PORT = "COM9"
BITRATE = 1_000_000
DURATION = 10

bus = None

try:
    bus = can.Bus(
        interface="slcan",
        channel=PORT,
        bitrate=BITRATE,
        sleep_after_open=2
    )

    print("===== RS05 MIT RAW MONITOR =====")
    print("Passive receive only.")
    print("No motor commands transmitted.")
    print("Monitoring for 10 seconds...\n")

    start = time.monotonic()
    count = 0
    ids = Counter()

    while time.monotonic() - start < DURATION:

        msg = bus.recv(timeout=0.1)

        if msg is None:
            continue

        count += 1
        ids[(msg.arbitration_id, msg.is_extended_id)] += 1

        print(
            f"ID=0x{msg.arbitration_id:X} "
            f"EXT={msg.is_extended_id} "
            f"DLC={msg.dlc} "
            f"DATA={msg.data.hex(' ').upper()}"
        )

    print("\n========== SUMMARY ==========")
    print(f"Total frames: {count}")

    for (cid, extended), total in ids.most_common():
        print(
            f"ID=0x{cid:X} "
            f"EXT={extended}: {total} frames"
        )

except Exception as e:
    print("ERROR:", repr(e))

finally:
    if bus:
        bus.shutdown()

    print("\nCAN closed.")
