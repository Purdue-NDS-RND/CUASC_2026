"""
Batch ArduPilot .BIN to CSV Exporter
Finds all .BIN files in the directory and extracts GPS/Attitude telemetry to CSVs.
"""

import csv
import glob
import os
from datetime import datetime, timedelta, timezone

from pymavlink import mavutil

# --- CONFIGURATION ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# FIX 1: Point directly to the logs_csv folder
LOGS_CSV_DIR = os.path.join(SCRIPT_DIR, "logs_csv")


def gps_time_to_datetime(gps_week, gps_ms):
    """Converts atomic GPS time to a standard UTC datetime."""
    # FIX 2: Explicitly assign UTC timezone to prevent local machine offsets
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
    leap_seconds = 18
    return gps_epoch + timedelta(
        weeks=gps_week, milliseconds=gps_ms, seconds=-leap_seconds
    )


def process_log(bin_path, csv_path):
    """Parses a single .BIN file and writes the telemetry to a CSV."""
    print(f"📖 Opening DataFlash Log: {os.path.basename(bin_path)}...")

    mlog = mavutil.mavlink_connection(bin_path, robust_parsing=True)

    current_roll = 0.0
    current_pitch = 0.0
    current_yaw = 0.0
    time_offset = None
    row_count = 0

    with open(csv_path, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "Unix_Timestamp",
                "Real_World_Time",
                "Latitude",
                "Longitude",
                "Altitude_m",
                "Roll_deg",
                "Pitch_deg",
                "Yaw_deg",
            ]
        )

        while True:
            msg = mlog.recv_match(type=["GPS", "ATT", "POS"], blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == "ATT":
                current_roll = msg.Roll
                current_pitch = msg.Pitch
                current_yaw = msg.Yaw

            elif msg_type == "GPS" and msg.Status >= 3:
                if time_offset is None:
                    dt = gps_time_to_datetime(msg.GWk, msg.GMS)
                    time_offset = dt.timestamp() - (msg.TimeUS / 1e6)

            elif msg_type == "POS" and time_offset is not None:
                unix_time = (msg.TimeUS / 1e6) + time_offset

                # FIX 2 (Cont.): Force the string output to format using UTC
                dt_str = datetime.fromtimestamp(unix_time, timezone.utc).strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )[:-3]

                writer.writerow(
                    [
                        f"{unix_time:.6f}",
                        dt_str,
                        msg.Lat,
                        msg.Lng,
                        msg.Alt,
                        current_roll,
                        current_pitch,
                        current_yaw,
                    ]
                )
                row_count += 1

    print(f"  ✅ Extracted {row_count} rows -> {os.path.basename(csv_path)}")


def main():
    # Ensure the output directory actually exists before writing
    os.makedirs(LOGS_CSV_DIR, exist_ok=True)

    # 1. Find all .BIN files in the LOGS directory
    search_pattern = os.path.join(SCRIPT_DIR, "LOGS", "*.BIN")
    bin_files = glob.glob(search_pattern)

    # FIX 3: Look for lowercase .bin files in the LOGS folder, not root
    bin_files.extend(glob.glob(os.path.join(SCRIPT_DIR, "LOGS", "*.bin")))

    if not bin_files:
        print(f"❌ No .BIN files found in LOGS folder")
        return

    print(f"🔍 Found {len(bin_files)} flight logs to process.\n")

    # 2. Loop through every file and process it
    for i, bin_file in enumerate(bin_files, 1):
        print(f"--- Processing Log {i} of {len(bin_files)} ---")

        base_name = os.path.splitext(os.path.basename(bin_file))[0]

        # FIX 1 (Cont.): Route the output directly to the logs_csv folder
        csv_output = os.path.join(LOGS_CSV_DIR, f"flight_{base_name}_telemetry.csv")

        process_log(bin_file, csv_output)

    print("\n🎉 All logs processed successfully!")


if __name__ == "__main__":
    main()
