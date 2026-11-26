
import can
import time

PORT = "COM9"
BITRATE = 1_000_000

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

        print(f"Connected: {PORT}")
        print(f"CAN bitrate: {BITRATE} bps")
        print("Listening for CAN frames...")
        print("Press CTRL+C to stop.\n")

        count = 0

        while True:
            msg = bus.recv(timeout=1.0)

            if msg is None:
                print("No CAN traffic...")
                continue

            count += 1

            print(
                f"[{count:05d}] "
                f"ID=0x{msg.arbitration_id:08X} "
                f"{'EXT' if msg.is_extended_id else 'STD'} "
                f"DLC={msg.dlc} "
                f"DATA={msg.data.hex(' ').upper()}"
            )

    except KeyboardInterrupt:
        print("\nStopped.")

    except Exception as e:
        print(f"\nConnection error: {e}")

    finally:
        if bus is not None:
            bus.shutdown()

if __name__ == "__main__":
    main()
