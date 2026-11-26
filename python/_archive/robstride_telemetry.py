
import can
import time
import math
import struct

# ==========================================
# ROBSTRIDE EDULITE 05 - LIVE TELEMETRY
# MKS CANable 2.0 / Windows SLCAN
# ==========================================

PORT = "COM9"
BITRATE = 1_000_000

MOTOR_ID = 0x7F
HOST_ID = 0xFD

# RS05 feedback ranges
POS_MIN = -4.0 * math.pi
POS_MAX =  4.0 * math.pi

VEL_MIN = -33.0
VEL_MAX =  33.0

TORQUE_MIN = -17.0
TORQUE_MAX =  17.0


def make_can_id(comm_type, host, motor):
    return (
        (comm_type << 24)
        | (host << 8)
        | motor
    )


def uint16_to_float(raw, minimum, maximum):
    return minimum + (raw / 65535.0) * (maximum - minimum)


def decode_feedback(msg):

    can_id = msg.arbitration_id

    comm_type = (can_id >> 24) & 0x1F
    mode = (can_id >> 22) & 0x03
    fault = (can_id >> 16) & 0x3F

    motor_id = (can_id >> 8) & 0xFF
    host_id = can_id & 0xFF

    if comm_type != 0x02:
        return None

    if motor_id != MOTOR_ID or host_id != HOST_ID:
        return None

    if msg.dlc != 8:
        return None

    data = msg.data

    raw_pos = int.from_bytes(data[0:2], "big")
    raw_vel = int.from_bytes(data[2:4], "big")
    raw_torque = int.from_bytes(data[4:6], "big")
    raw_temp = int.from_bytes(data[6:8], "big")

    position = uint16_to_float(
        raw_pos, POS_MIN, POS_MAX
    )

    velocity = uint16_to_float(
        raw_vel, VEL_MIN, VEL_MAX
    )

    torque = uint16_to_float(
        raw_torque, TORQUE_MIN, TORQUE_MAX
    )

    temperature = raw_temp / 10.0

    return {
        "position": position,
        "velocity": velocity,
        "torque": torque,
        "temperature": temperature,
        "mode": mode,
        "fault": fault
    }


def enable_reporting(bus, enable):

    can_id = make_can_id(
        0x18,
        HOST_ID,
        MOTOR_ID
    )

    # Type 24:
    # 01 02 03 04 05 06 F_CMD
    payload = bytes([
        0x01, 0x02, 0x03,
        0x04, 0x05, 0x06,
        0x01 if enable else 0x00,
        0x00
    ])

    msg = can.Message(
        arbitration_id=can_id,
        is_extended_id=True,
        data=payload
    )

    bus.send(msg, timeout=1.0)

    print(
        "Active reporting:",
        "ENABLED" if enable else "DISABLED"
    )


def main():

    bus = None

    try:
        print("=" * 55)
        print("ROBSTRIDE EDULITE 05 - TELEMETRY TEST")
        print("=" * 55)

        bus = can.Bus(
            interface="slcan",
            channel=PORT,
            bitrate=BITRATE,
            sleep_after_open=2
        )

        print(f"Connected: {PORT}")
        print(f"Bitrate: {BITRATE}")
        print(f"Motor ID: 0x{MOTOR_ID:02X}")

        enable_reporting(bus, True)

        print("\nReceiving feedback...")
        print("Press CTRL+C to stop.\n")

        print(
            f"{'POS(rad)':>10} "
            f"{'VEL(rad/s)':>12} "
            f"{'TORQUE(Nm)':>12} "
            f"{'TEMP(C)':>10} "
            f"{'MODE':>6} "
            f"{'FAULT':>8}"
        )

        print("-" * 70)

        count = 0
        last_message = time.monotonic()

        while True:

            msg = bus.recv(timeout=1.0)

            if msg is None:
                print("Waiting for feedback...")
                continue

            result = decode_feedback(msg)

            if result is None:
                continue

            count += 1
            last_message = time.monotonic()

            print(
                f"{result['position']:10.4f} "
                f"{result['velocity']:12.4f} "
                f"{result['torque']:12.4f} "
                f"{result['temperature']:10.1f} "
                f"{result['mode']:6d} "
                f"0x{result['fault']:02X}"
            )

    except KeyboardInterrupt:
        print("\nStopping telemetry...")

    except Exception as error:
        print(f"\nERROR: {error}")

    finally:

        if bus is not None:

            try:
                enable_reporting(bus, False)
            except Exception as error:
                print("Could not disable reporting:", error)

            bus.shutdown()

        print("CAN connection closed.")


if __name__ == "__main__":
    main()
