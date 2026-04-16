"""
Offline Photogrammetry & Target Geolocation
Parses ArduPilot .BIN logs, syncs with image timestamps, runs YOLO, and raycasts targets.
"""

import csv
import glob
import math
import os
from datetime import datetime, timedelta

import cv2
import numpy as np
import yaml
from pymavlink import mavutil
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO

# --- CONFIGURATION PATHS ---
# 1. Dynamically find the exact folder THIS script is sitting in
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 2. Anchor all your config files to this directory
BIN_LOG_PATH = os.path.join(SCRIPT_DIR, "00000032.BIN")
IMAGE_DIR = os.path.join(SCRIPT_DIR, "camera_captures")
YOLO_MODEL_PATH = os.path.join(SCRIPT_DIR, "yolo26n_v1.0.engine")
FLIGHT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "offline_flight_config.yaml")
CAMERA_INFO_PATH = os.path.join(SCRIPT_DIR, "arducam_info.yaml")
OUTPUT_CSV = os.path.join(SCRIPT_DIR, "mission_results.csv")
# Earth radius in meters
R_EARTH = 6378137.0


def load_yaml(filepath):
    with open(filepath, "r") as f:
        return yaml.safe_load(f)


def gps_time_to_datetime(gps_week, gps_ms):
    """Converts atomic GPS time (Week + Milliseconds) to a standard UTC datetime."""
    # GPS epoch started on Jan 6, 1980
    gps_epoch = datetime(1980, 1, 6, 0, 0, 0)
    # GPS time is ahead of UTC by 18 leap seconds (as of recent years)
    leap_seconds = 18

    elapsed = timedelta(weeks=gps_week, milliseconds=gps_ms, seconds=-leap_seconds)
    return gps_epoch + elapsed


def parse_flight_log(bin_path):
    """
    Cracks open the DataFlash .BIN log and builds an interpolated timeline
    of the drone's telemetry so we can query it by Unix timestamp.
    """
    print(f"📖 Parsing Flight Log: {bin_path}...")
    mlog = mavutil.mavlink_connection(bin_path)

    time_us_to_unix = {}
    telemetry = []

    while True:
        msg = mlog.recv_match(type=["GPS", "ATT", "POS"], blocking=False)
        if msg is None:
            break

        msg_type = msg.get_type()
        time_us = msg.TimeUS

        # Sync the internal boot clock (TimeUS) to Real-World GPS Time
        if msg_type == "GPS" and msg.Status >= 3:  # 3 = 3D Fix
            dt = gps_time_to_datetime(msg.GWk, msg.GMS)
            time_us_to_unix[time_us] = dt.timestamp()

        # Grab Attitude (Roll/Pitch/Yaw)
        elif msg_type == "ATT":
            telemetry.append(
                {
                    "time_us": time_us,
                    "roll": math.radians(msg.Roll),
                    "pitch": math.radians(msg.Pitch),
                    "yaw": math.radians(msg.Yaw),
                    "type": "ATT",
                }
            )

        # Grab Position (Lat/Lon/Alt)
        elif msg_type == "POS":
            telemetry.append(
                {
                    "time_us": time_us,
                    "lat": msg.Lat,
                    "lon": msg.Lng,
                    "alt": msg.Alt,  # Altitude in meters
                    "type": "POS",
                }
            )

    print("⏳ Synchronizing clocks and building interpolation tables...")

    # We need a continuous time reference. We will match TimeUS to the closest mapped Unix time.
    # (Assuming clock drift is negligible over a single flight)
    us_keys = sorted(list(time_us_to_unix.keys()))
    if not us_keys:
        raise ValueError("No GPS 3D Fix found in log. Cannot sync time!")

    # Separate data into arrays for SciPy interpolation
    pos_data = [t for t in telemetry if t["type"] == "POS"]
    att_data = [t for t in telemetry if t["type"] == "ATT"]

    # Map TimeUS to Unix Epoch based on the first GPS lock offset
    time_offset = time_us_to_unix[us_keys[0]] - (us_keys[0] / 1e6)

    # Create Interpolation Functions
    get_lat = interp1d(
        [(p["time_us"] / 1e6) + time_offset for p in pos_data],
        [p["lat"] for p in pos_data],
        fill_value="extrapolate",
    )
    get_lon = interp1d(
        [(p["time_us"] / 1e6) + time_offset for p in pos_data],
        [p["lon"] for p in pos_data],
        fill_value="extrapolate",
    )
    get_alt = interp1d(
        [(p["time_us"] / 1e6) + time_offset for p in pos_data],
        [p["alt"] for p in pos_data],
        fill_value="extrapolate",
    )

    get_roll = interp1d(
        [(a["time_us"] / 1e6) + time_offset for a in att_data],
        [a["roll"] for a in att_data],
        fill_value="extrapolate",
    )
    get_pitch = interp1d(
        [(a["time_us"] / 1e6) + time_offset for a in att_data],
        [a["pitch"] for a in att_data],
        fill_value="extrapolate",
    )
    get_yaw = interp1d(
        [(a["time_us"] / 1e6) + time_offset for a in att_data],
        [a["yaw"] for a in att_data],
        fill_value="extrapolate",
    )

    return lambda t: (
        get_lat(t),
        get_lon(t),
        get_alt(t),
        get_roll(t),
        get_pitch(t),
        get_yaw(t),
    )


def parse_image_time(filename):
    """Extracts the datetime from: img_YYYYMMDD_HHMM_count_HHMMSS_mmm.webp"""
    base = os.path.basename(filename)
    parts = base.split("_")
    # Parts: ['img', '20260415', '1200', '0001', '120005', '123.webp']
    date_str = parts[1]
    time_str = parts[4]
    ms_str = parts[5].split(".")[0]

    dt_str = f"{date_str}_{time_str}_{ms_str}"
    dt = datetime.strptime(dt_str, "%Y%m%d_%H%M%S_%f")
    return dt.timestamp()


def main():
    # 1. Load Configurations
    flight_cfg = load_yaml(FLIGHT_CONFIG_PATH)
    cam_cfg = load_yaml(CAMERA_INFO_PATH)

    ground_z = flight_cfg["flight_settings"]["ground_altitude_m"]
    mount_x = flight_cfg["camera_mount"]["offset_x_m"]
    mount_y = flight_cfg["camera_mount"]["offset_y_m"]
    mount_z = flight_cfg["camera_mount"]["offset_z_m"]

    mount_r = R.from_euler(
        "xyz",
        [
            flight_cfg["camera_mount"]["roll_deg"],
            flight_cfg["camera_mount"]["pitch_deg"],
            flight_cfg["camera_mount"]["yaw_deg"],
        ],
        degrees=True,
    )

    # Note: Images are already undistorted by your capture script!
    # We only need the focal lengths and optical centers to project the ray.
    K = np.array(cam_cfg["camera_matrix"]["data"]).reshape((3, 3))
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # 2. Parse Flight Log & Load YOLO
    get_telemetry_at_time = parse_flight_log(BIN_LOG_PATH)

    print(f"🧠 Loading YOLO Engine: {YOLO_MODEL_PATH}...")
    model = YOLO(YOLO_MODEL_PATH, task="detect")

    images = glob.glob(os.path.join(IMAGE_DIR, "*.webp"))
    images.sort()

    print(f"🔍 Found {len(images)} images to process. Beginning Photogrammetry...")

    # 3. Process Images
    with open(OUTPUT_CSV, mode="w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            ["Image", "Target_Class", "Confidence", "Lat", "Lon", "Drone_Alt"]
        )

        for img_path in images:
            img_time = parse_image_time(img_path)

            # Query the flight log for this exact millisecond
            lat, lon, alt, roll, pitch, yaw = get_telemetry_at_time(img_time)
            drone_r = R.from_euler("xyz", [roll, pitch, yaw], degrees=False)

            frame = cv2.imread(img_path)
            results = model(frame, conf=0.50, verbose=False)

            for box in results[0].boxes:
                # Get Pixel Coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u = x1 + (x2 - x1) / 2.0
                v = y1 + (y2 - y1) / 2.0
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())

                # Step A: Pixel to Optical Ray (Z-forward)
                ray_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                ray_opt /= np.linalg.norm(ray_opt)

                # Step B: Rotate through Mount -> Drone -> Earth Frame
                ray_body = mount_r.apply(ray_opt)
                ray_world = drone_r.apply(ray_body)

                # Step C: Calculate physical lens location in the air
                cam_world_offset = drone_r.apply(np.array([mount_x, mount_y, mount_z]))
                cam_z = alt + cam_world_offset[2]

                # Step D: Ground Intersection
                if abs(ray_world[2]) < 1e-6:
                    continue  # Ray is perfectly horizontal to the ground

                t = (ground_z - cam_z) / ray_world[2]
                if t < 0:
                    continue  # Looking at the sky

                # Offset in meters from the drone's center
                target_offset_x = cam_world_offset[0] + (t * ray_world[0])
                target_offset_y = cam_world_offset[1] + (t * ray_world[1])

                # Step E: Geolocation (Equirectangular projection)
                lat_offset = (target_offset_y / R_EARTH) * (180.0 / math.pi)
                lon_scale = math.cos(math.radians(lat))
                lon_offset = (target_offset_x / (R_EARTH * lon_scale)) * (
                    180.0 / math.pi
                )

                final_lat = lat + lat_offset
                final_lon = lon + lon_offset

                # Write to CSV
                writer.writerow(
                    [
                        os.path.basename(img_path),
                        cls_id,
                        f"{conf:.2f}",
                        f"{final_lat:.7f}",
                        f"{final_lon:.7f}",
                        f"{alt:.2f}",
                    ]
                )

                # Draw on the image and save for human review
                cv2.circle(frame, (int(u), int(v)), 30, (0, 0, 255), 4)
                cv2.putText(
                    frame,
                    f"Lat: {final_lat:.6f} Lon: {final_lon:.6f}",
                    (int(x1), int(y1) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 255, 0),
                    3,
                )

            # Overwrite or save to a new "processed" directory
            cv2.imwrite(img_path.replace(".webp", "_processed.webp"), frame)

    print(f"✅ Mission Complete. Results saved to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
