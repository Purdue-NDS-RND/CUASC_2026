"""
Offline Photogrammetry & Target Geolocation
Parses ArduPilot .BIN logs, syncs with image timestamps, runs YOLO, and raycasts targets.
"""

import csv
import glob
import math

# --- CONFIGURATION PATHS ---
import os
from datetime import datetime, timedelta

import cv2
import numpy as np
import yaml
from pymavlink import mavutil
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO

# 1. Find the script's directory (.../src/vision_pipeline/vision_pipeline)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 2. Go UP one level to the package root (.../src/vision_pipeline)
PACKAGE_ROOT = os.path.dirname(SCRIPT_DIR)
# 3. Hardcode the absolute paths to your data folders
IMAGE_DIR = "/home/nds2/camera_captures_calibrated"
OUTPUT_DIR = "/home/nds2/camera_captures_processed"
# 4. Anchor your files
# Route the path into your new LOGS folder
BIN_LOG_PATH = os.path.join(SCRIPT_DIR, "LOGS", "00000032.BIN")
YOLO_MODEL_PATH = os.path.join(PACKAGE_ROOT, "models", "yolo26n_v1.0.engine")
# point to the config directory!
FLIGHT_CONFIG_PATH = os.path.join(PACKAGE_ROOT, "config", "offline_flight_config.yaml")
CAMERA_INFO_PATH = os.path.join(PACKAGE_ROOT, "config", "arducam_info.yaml")
# 5. Save the final CSV directly into the NEW output folder
OUTPUT_CSV = os.path.join(OUTPUT_DIR, "mission_results.csv")
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
    # ✅ CORRECT: Initialize it once before the loop starts
    home_alt = None
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
            # The first time we see a POS message, lock in the ground altitude
            if home_alt is None:
                home_alt = msg.Alt
            telemetry.append(
                {
                    "time_us": time_us,
                    "lat": msg.Lat,
                    "lon": msg.Lng,
                    # Subtract home_alt to convert AMSL to Height Above Ground!
                    "alt": msg.Alt - home_alt,
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
    os.makedirs(OUTPUT_DIR, exist_ok=True)
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
    print(f"🔍 Found {len(images)} images to process. Beginning Photogrammetry...\n")

    # Running totals for the end-of-run summary
    total_detections = 0
    images_with_detections = 0
    images_skipped = 0

    # 3. Process Images
    with open(OUTPUT_CSV, mode="w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            ["Image", "Target_Class", "Confidence", "Lat", "Lon", "Drone_Alt"]
        )
        for img_idx, img_path in enumerate(images, start=1):
            base_filename = os.path.basename(img_path)
            print(f"[{img_idx:04d}/{len(images):04d}] 📷 {base_filename}")

            # --- Timestamp Parsing (with error guard) ---
            try:
                img_time = parse_image_time(img_path)
            except (IndexError, ValueError) as e:
                print(f"           ⚠️  Skipping — bad filename format: {e}\n")
                images_skipped += 1
                continue

            # --- Image Loading (with error guard) ---
            frame = cv2.imread(img_path)
            if frame is None:
                print(
                    f"           ⚠️  Skipping — cv2 could not read file (bad/corrupt webp?)\n"
                )
                images_skipped += 1
                continue

            # --- Telemetry Lookup ---
            lat, lon, alt, roll, pitch, yaw = get_telemetry_at_time(img_time)
            print(
                f"           📡 Telemetry → lat={float(lat):.5f}, lon={float(lon):.5f}, "
                f"alt={float(alt):.1f}m, yaw={math.degrees(float(yaw)):.1f}°"
            )

            drone_r = R.from_euler("xyz", [roll, pitch, yaw], degrees=False)

            # --- YOLO Inference ---
            results = model(frame, conf=0.50, imgsz=1280, verbose=False, device=0)
            box_count = len(results[0].boxes)
            print(f"           🎯 YOLO → {box_count} detection(s) at conf=0.50")

            if box_count == 0:
                print(f"           ⏭️  No detections — image will NOT be saved.\n")
                continue

            # --- Process Each Detection ---
            images_with_detections += 1
            rows_written = 0
            for box in results[0].boxes:
                # Get Pixel Coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u = x1 + (x2 - x1) / 2.0
                v = y1 + (y2 - y1) / 2.0
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())

                print(
                    f"              ↳ Box #{rows_written + 1}: cls={cls_id}, conf={conf:.3f}, "
                    f"center=({u:.0f}, {v:.0f})"
                )

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
                    print(
                        f"              ↳ ⚠️  Box #{rows_written + 1} dropped — ray is horizontal (ray_world[2]≈0)"
                    )
                    continue
                t = (ground_z - cam_z) / ray_world[2]
                if t < 0:
                    print(
                        f"              ↳ ⚠️  Box #{rows_written + 1} dropped — ray points skyward "
                        f"(t={t:.2f}, cam_z={float(cam_z):.2f}, ground_z={ground_z})"
                    )
                    continue

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

                print(
                    f"              ↳ 📍 Geolocated → lat={float(final_lat):.6f}, lon={float(final_lon):.6f}"
                )

                # Write to CSV
                writer.writerow(
                    [
                        base_filename,
                        cls_id,
                        f"{conf:.2f}",
                        f"{float(final_lat):.7f}",
                        f"{float(final_lon):.7f}",
                        f"{float(alt):.2f}",
                    ]
                )
                rows_written += 1
                total_detections += 1

                # Draw on the image
                cv2.circle(frame, (int(u), int(v)), 30, (0, 0, 255), 4)
                cv2.putText(
                    frame,
                    f"Lat: {float(final_lat):.6f} Lon: {float(final_lon):.6f}",
                    (int(x1), int(y1) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 255, 0),
                    3,
                )

            # Only save the image if at least one box survived the raycast checks
            if rows_written > 0:
                new_filename = base_filename.replace(".webp", "_processed.webp")
                processed_img_path = os.path.join(OUTPUT_DIR, new_filename)
                cv2.imwrite(processed_img_path, frame)
                print(
                    f"           💾 Saved → {new_filename} ({rows_written} box(es) written to CSV)\n"
                )
            else:
                print(
                    f"           ⚠️  {box_count} detection(s) found but all dropped by raycast checks — image NOT saved.\n"
                )

    # --- Final Summary ---
    print("=" * 60)
    print(f"✅ Mission Complete.")
    print(f"   Images processed : {len(images) - images_skipped}")
    print(f"   Images skipped   : {images_skipped}")
    print(f"   Images with hits : {images_with_detections}")
    print(f"   Total detections : {total_detections}")
    print(f"   Results CSV      : {OUTPUT_CSV}")
    print("=" * 60)


if __name__ == "__main__":
    main()
