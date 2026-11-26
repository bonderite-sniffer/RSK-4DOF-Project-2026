
import can
import serial.tools.list_ports
import time

BITRATE = 1_000_000

def list_ports():
    print("\nAvailable serial ports:")
    for p in serial.tools.list_ports.comports():
        print(f"{p.device}: {p.description}")

def main():
    list_ports()

    port = input("\nEnter CANable COM port (e.g. COM7): ").strip()

    try:
        bus = can.Bus(
            interface="slcan",
            channel=port,
            bitrate=BITRATE
        )

        print(f"\nConnected to {port}")
        print(f"Bitrate: {BITRATE}")
        print("Listening for CAN frames...")
        print("Press CTRL+C to exit.\n")

        while True:
            msg = bus.recv(timeout=1.0)

            if msg is not None:
                frame_type = (
                    "EXT" if msg.is_extended_id else "STD"
                )

                data = " ".join(
                    f"{b:02X}" for b in msg.data
                )

                print(
                    f"ID: 0x{msg.arbitration_id:X} | "
                    f"{frame_type} | "
                    f"DLC: {msg.dlc} | "
                    f"DATA: {data}"
                )

    except KeyboardInterrupt:
        print("\nMonitoring stopped.")

    except Exception as e:
        print(f"\nCAN Error: {e}")

    finally:
        if "bus" in locals():
            bus.shutdown()

if __name__ == "__main__":
    main()
