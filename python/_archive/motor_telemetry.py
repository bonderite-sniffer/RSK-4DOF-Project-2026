
import can
import time
import csv
import statistics
from datetime import datetime
from collections import deque

# ==========================================
# CONFIGURATION
# ==========================================

PORT = "COM9"
BITRATE = 1_000_000

MOTOR_ID = 0x7F
HOST_ID = 0xFD

P_MIN = -12.57
P_MAX =  12.57

V_MIN = -50.0
V_MAX =  50.0

T_MIN = -5.5
T_MAX =  5.5

CSV_FILE = "rs05_telemetry.csv"

# ==========================================
# CAN DECODER
# ==========================================

def uint_to_float(x, x_min, x_max):
    return x_min + (x / 65535.0) * (x_max - x_min)


def decode_feedback(msg):

    if not msg.is_extended_id or msg.dlc != 8:
        return None

    cid = msg.arbitration_id

    comm_type = (cid >> 24) & 0x1F
    motor_id = (cid >> 8) & 0xFF
    host_id = cid & 0xFF

    if comm_type != 24:
        return None

    if motor_id != MOTOR_ID or host_id != HOST_ID:
        return None

    data = msg.data

    raw_p = int.from_bytes(data[0:2], "big")
    raw_v = int.from_bytes(data[2:4], "big")
    raw_t = int.from_bytes(data[4:6], "big")
    raw_temp = int.from_bytes(data[6:8], "big")

    position = uint_to_float(raw_p, P_MIN, P_MAX)
    velocity = uint_to_float(raw_v, V_MIN, V_MAX)
    torque = uint_to_float(raw_t, T_MIN, T_MAX)
    temperature = raw_temp / 10.0

    return {
        "position": position,
        "velocity": velocity,
        "torque": torque,
        "temperature": temperature,
        "raw_p": raw_p,
        "raw_v": raw_v,
        "raw_t": raw_t,
        "raw_temp": raw_temp
    }


# ==========================================
# MAIN
# ==========================================

def main():

    bus = None
    timestamps = deque(maxlen=100)

    total_frames = 0
    last_display = 0

    try:

        bus = can.Bus(
            interface="slcan",
            channel=PORT,
            bitrate=BITRATE,
            sleep_after_open=2
        )

        print("===================================")
        print(" ROBSTRIDE RS05 LIVE TELEMETRY")
        print("===================================")
        print(f"Port       : {PORT}")
        print(f"Bitrate    : {BITRATE}")
        print(f"Motor ID   : 0x{MOTOR_ID:02X}")
        print(f"Host ID    : 0x{HOST_ID:02X}")
        print(f"CSV Output : {CSV_FILE}")
        print("\nPassive monitoring only.")
        print("Press CTRL+C to stop.\n")

        with open(CSV_FILE, "w", newline="") as file:

            writer = csv.writer(file)

            writer.writerow([
                "timestamp",
                "elapsed_s",
                "position_rad",
                "velocity_rad_s",
                "torque_Nm",
                "temperature_C",
                "feedback_hz",
                "raw_p",
                "raw_v",
                "raw_t",
                "raw_temp"
            ])

            start = time.monotonic()

            while True:

                msg = bus.recv(timeout=0.1)

                if msg is None:
                    continue

                feedback = decode_feedback(msg)

                if feedback is None:
                    continue

                now = time.monotonic()
                elapsed = now - start

                timestamps.append(now)
                total_frames += 1

                frequency = 0.0

                if len(timestamps) >= 2:
                    dt = timestamps[-1] - timestamps[0]

                    if dt > 0:
                        frequency = (
                            len(timestamps) - 1
                        ) / dt

                timestamp = datetime.now().isoformat(
                    timespec="milliseconds"
                )

                writer.writerow([
                    timestamp,
                    round(elapsed, 4),
                    feedback["position"],
                    feedback["velocity"],
                    feedback["torque"],
                    feedback["temperature"],
                    frequency,
                    feedback["raw_p"],
                    feedback["raw_v"],
                    feedback["raw_t"],
                    feedback["raw_temp"]
                ])

                # Update display every 0.2 seconds
                if now - last_display >= 0.2:

                    print("\033[2J\033[H", end="")

                    print("======= RS05 LIVE TELEMETRY =======")
                    print(f"Elapsed     : {elapsed:.2f} s")
                    print(f"Frames      : {total_frames}")
                    print(f"Feedback    : {frequency:.2f} Hz")

                    print("-----------------------------------")

                    print(
                        f"Position    : "
                        f"{feedback['position']:.5f} rad"
                    )

                    print(
                        f"Velocity    : "
                        f"{feedback['velocity']:.5f} rad/s"
                    )

                    print(
                        f"Torque      : "
                        f"{feedback['torque']:.5f} Nm"
                    )

                    print(
                        f"Temperature : "
                        f"{feedback['temperature']:.1f} C"
                    )

                    print("-----------------------------------")
                    print("Logging to CSV...")
                    print("Press CTRL+C to stop.")

                    last_display = now

                    file.flush()

    except KeyboardInterrupt:

        print("\n\nMonitoring stopped by user.")

    except Exception as e:

        print("\nERROR:", repr(e))

    finally:

        if bus:
            bus.shutdown()

        print("\nCAN closed.")
        print(f"Total frames received: {total_frames}")
        print(f"CSV saved: {CSV_FILE}")


if __name__ == "__main__":
    main()
