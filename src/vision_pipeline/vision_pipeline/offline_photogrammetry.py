"""
Offline Photogrammetry & Target Geolocation (Bulletproof Time Sync)
Automatically scans the logs_csv directory, corrects timezone offsets, and matches images.
"""

import csv
import glob
import math
import os
from datetime import datetime, timedelta, timezone

import cv2
import numpy as np
import yaml
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO

# --- DIRECTORY ANCHORS ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.dirname(SCRIPT_DIR)
IMAGE_DIR = "/home/nds2/camera_captures_calibrated"
OUTPUT_DIR = "/home/nds2/camera_captures_processed"

LOGS_CSV_DIR = os.path.join(SCRIPT_DIR, "logs_csv")

YOLO_MODEL_PATH = os.path.join(PACKAGE_ROOT, "models", "yolo26n_v1.0.engine")
FLIGHT_CONFIG_PATH = os.path.join(PACKAGE_ROOT, "config", "offline_flight_config.yaml")
CAMERA_INFO_PATH = os.path.join(PACKAGE_ROOT, "config", "arducam_info.yaml")
OUTPUT_CSV = os.path.join(OUTPUT_DIR, "mission_results.csv")

R_EARTH = 6378137.0


def load_yaml(filepath):
    with open(filepath, "r") as f:
        return yaml.safe_load(f)


def parse_image_time(filename):
    """Extracts datetime from filename, assumes EDT (UTC-4), returns true UTC Unix Time."""
    base = os.path.basename(filename)
    parts = base.split("_")
    date_str, time_str, ms_str = parts[1], parts[4], parts[5].split(".")[0]

    # Parse the string into a naive datetime
    dt_naive = datetime.strptime(f"{date_str}_{time_str}_{ms_str}", "%Y%m%d_%H%M%S_%f")

    # Force it to EDT (Indiana Time) and convert to Unix Timestamp
    edt_tz = timezone(timedelta(hours=-4))
    return dt_naive.replace(tzinfo=edt_tz).timestamp()


def parse_csv_time(time_str):
    """Parses '2026-04-09 21:15:19.662' (UTC) into a true UTC Unix Timestamp."""
    dt_naive = datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S.%f")
    # Force it to recognize this string as UTC to bypass local machine offsets
    return dt_naive.replace(tzinfo=timezone.utc).timestamp()


class CSVLogManager:
    """Scans all telemetry CSVs and intelligently loads the correct one for a given timestamp."""

    def __init__(self, logs_directory):
        self.logs_dir = logs_directory
        self.log_index = []
        self.active_log = None
        self.active_telemetry_fn = None
        self.active_home_alt = None

        print("🗄️  Scanning logs_csv directory to build master time index...")
        self._build_index()

    def _build_index(self):
        csv_files = glob.glob(os.path.join(self.logs_dir, "*.csv"))

        for csv_path in csv_files:
            with open(csv_path, "r") as f:
                reader = csv.DictReader(f)
                try:
                    first_row = next(reader)
                    # 🎯 Fix: Parse the human readable string as UTC instead of using the broken Unix column
                    start_time = parse_csv_time(first_row["Real_World_Time"])

                    last_row = first_row
                    for row in reader:
                        last_row = row
                    end_time = parse_csv_time(last_row["Real_World_Time"])

                    self.log_index.append(
                        {
                            "path": csv_path,
                            "start": start_time,
                            "end": end_time,
                            "filename": os.path.basename(csv_path),
                        }
                    )
                except StopIteration:
                    continue

        print(f"✅ Indexed {len(self.log_index)} flight logs.\n")

    def get_telemetry(self, image_timestamp):
        """Finds the correct log for the image, loads it if necessary, and returns the telemetry."""
        if self.active_log and (
            self.active_log["start"] <= image_timestamp <= self.active_log["end"]
        ):
            return self.active_telemetry_fn(image_timestamp), self.active_home_alt

        for log in self.log_index:
            if log["start"] <= image_timestamp <= log["end"]:
                print(f"🔄 Image requires log switch. Loading {log['filename']}...")
                self._load_log(log)
                return self.active_telemetry_fn(image_timestamp), self.active_home_alt

        return None, None

    def _load_log(self, log_meta):
        """Loads the CSV into RAM and builds the SciPy interpolation arrays."""
        unix_times, lats, lons, alts, rolls, pitches, yaws = [], [], [], [], [], [], []
        home_alt = None

        with open(log_meta["path"], "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                # 🎯 Fix: Parse the human readable string as UTC
                t = parse_csv_time(row["Real_World_Time"])
                lat = float(row["Latitude"])
                lon = float(row["Longitude"])
                alt_amsl = float(row["Altitude_m"])

                roll = math.radians(float(row["Roll_deg"]))
                pitch = math.radians(float(row["Pitch_deg"]))
                yaw = math.radians(float(row["Yaw_deg"]))

                if home_alt is None:
                    home_alt = alt_amsl

                unix_times.append(t)
                lats.append(lat)
                lons.append(lon)
                alts.append(alt_amsl - home_alt)
                rolls.append(roll)
                pitches.append(pitch)
                yaws.append(yaw)

        get_lat = interp1d(unix_times, lats, fill_value="extrapolate")
        get_lon = interp1d(unix_times, lons, fill_value="extrapolate")
        get_alt = interp1d(unix_times, alts, fill_value="extrapolate")
        get_roll = interp1d(unix_times, rolls, fill_value="extrapolate")
        get_pitch = interp1d(unix_times, pitches, fill_value="extrapolate")
        get_yaw = interp1d(unix_times, yaws, fill_value="extrapolate")

        self.active_log = log_meta
        self.active_home_alt = home_alt
        self.active_telemetry_fn = lambda t: (
            get_lat(t),
            get_lon(t),
            get_alt(t),
            get_roll(t),
            get_pitch(t),
            get_yaw(t),
        )


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

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

    K = np.array(cam_cfg["camera_matrix"]["data"]).reshape((3, 3))
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    log_manager = CSVLogManager(LOGS_CSV_DIR)

    print(f"🧠 Loading YOLO Engine: {YOLO_MODEL_PATH}...")
    model = YOLO(YOLO_MODEL_PATH, task="detect")

    images = glob.glob(os.path.join(IMAGE_DIR, "*.webp"))
    images.sort()
    print(f"\n🔍 Found {len(images)} images to process.\n")

    total_detections, images_with_hits, images_skipped, out_of_range_count = 0, 0, 0, 0

    with open(OUTPUT_CSV, mode="w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            ["Image", "Target_Class", "Confidence", "Lat", "Lon", "Drone_Alt"]
        )

        for img_idx, img_path in enumerate(images, start=1):
            base_filename = os.path.basename(img_path)
            print(f"[{img_idx:04d}/{len(images):04d}] 📷 {base_filename}")

            try:
                img_time = parse_image_time(img_path)
            except Exception as e:
                print(f"           ⚠️  Skipping — bad filename: {e}\n")
                images_skipped += 1
                continue

            frame = cv2.imread(img_path)
            if frame is None:
                images_skipped += 1
                continue

            telemetry_data, home_alt = log_manager.get_telemetry(img_time)

            if telemetry_data is None:
                out_of_range_count += 1
                images_skipped += 1
                img_utc = datetime.utcfromtimestamp(img_time).strftime(
                    "%Y-%m-%d %H:%M:%S UTC"
                )
                print(
                    f"           🚨 No matching CSV log found for {img_utc}. Skipping.\n"
                )
                continue

            lat, lon, alt, roll, pitch, yaw = telemetry_data
            print(
                f"           📡 Telemetry → lat={float(lat):.5f}, lon={float(lon):.5f}, alt={float(alt):.1f}m"
            )

            drone_r = R.from_euler("xyz", [roll, pitch, yaw], degrees=False)

            # Using your correct inference settings!
            results = model(frame, conf=0.50, imgsz=1280, verbose=False, device=0)
            box_count = len(results[0].boxes)

            if box_count == 0:
                print(f"           ⏭️  No detections.\n")
                continue

            rows_written = 0

            # Grab the height and width of the full image for boundary checking
            img_h, img_w = frame.shape[:2]

            for box_idx, box in enumerate(results[0].boxes, start=1):
                # Get raw bounding box limits
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                u, v = x1 + (x2 - x1) / 2.0, y1 + (y2 - y1) / 2.0
                conf, cls_id = (
                    float(box.conf[0].cpu().numpy()),
                    int(box.cls[0].cpu().numpy()),
                )

                # --- NEW: CROP AND SAVE THE TARGET CHIP ---
                PADDING = 100  # How many extra pixels to include around the target

                # Calculate padded crop boundaries, making sure we don't slice outside the image
                crop_y1 = max(0, int(y1) - PADDING)
                crop_y2 = min(img_h, int(y2) + PADDING)
                crop_x1 = max(0, int(x1) - PADDING)
                crop_x2 = min(img_w, int(x2) + PADDING)

                # Slice the numpy array to crop the image
                target_crop = frame[crop_y1:crop_y2, crop_x1:crop_x2]

                # Save the cropped chip with a unique filename
                chip_filename = base_filename.replace(".webp", f"_CHIP_{box_idx}.webp")
                chip_path = os.path.join(OUTPUT_DIR, chip_filename)
                cv2.imwrite(chip_path, target_crop)
                # ------------------------------------------

                ray_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
                ray_opt /= np.linalg.norm(ray_opt)

                # Step B: Map OpenCV to Drone's NED Body Frame
                # Assuming camera is mounted flat, pointing down, top of image facing forward
                # Opt X (Right) -> NED Y (East)
                # Opt Y (Down)  -> NED -X (South)
                # Opt Z (Fwd)   -> NED Z (Down)
                ray_body_ned = np.array([-ray_opt[1], ray_opt[0], ray_opt[2]])

                # Apply YAML mount rotations (Does nothing if yaml is 0,0,0)
                ray_body_ned = mount_r.apply(ray_body_ned)

                # Step C: Rotate Drone Body to World NED
                # ArduPilot uses intrinsic ZYX (Yaw, Pitch, Roll)
                drone_r_ned = R.from_euler("ZYX", [yaw, pitch, roll], degrees=False)
                ray_world_ned = drone_r_ned.apply(ray_body_ned)

                # Step D: Convert World NED to World ENU (East, North, Up) for Geography
                # NED (X=North, Y=East, Z=Down) -> ENU (X=East, Y=North, Z=Up)
                ray_world_enu = np.array(
                    [ray_world_ned[1], ray_world_ned[0], -ray_world_ned[2]]
                )

                # Map physical lens offset from the drone's center
                cam_world_offset_ned = drone_r_ned.apply(
                    np.array([mount_x, mount_y, mount_z])
                )
                cam_world_offset_enu = np.array(
                    [
                        cam_world_offset_ned[1],
                        cam_world_offset_ned[0],
                        -cam_world_offset_ned[2],
                    ]
                )

                cam_z = float(alt) + cam_world_offset_enu[2]

                # Step E: Ground Intersection & Geolocation
                if abs(ray_world_enu[2]) < 1e-6:
                    print(f"              ↳ ⚠️  Dropped — ray is horizontal")
                    continue

                t_ray = (ground_z - cam_z) / ray_world_enu[2]
                if t_ray < 0:
                    print(
                        f"              ↳ ⚠️  Dropped — ray points skyward (t={t_ray:.2f}, cam_z={cam_z:.2f})"
                    )
                    continue

                # X is East (Longitude), Y is North (Latitude)
                target_offset_x = cam_world_offset_enu[0] + (t_ray * ray_world_enu[0])
                target_offset_y = cam_world_offset_enu[1] + (t_ray * ray_world_enu[1])

                lat_offset = (target_offset_y / R_EARTH) * (180.0 / math.pi)
                lon_scale = math.cos(math.radians(float(lat)))
                lon_offset = (target_offset_x / (R_EARTH * lon_scale)) * (
                    180.0 / math.pi
                )

                final_lat = float(lat) + lat_offset
                final_lon = float(lon) + lon_offset
                writer.writerow(
                    [
                        base_filename,
                        cls_id,
                        f"{conf:.2f}",
                        f"{final_lat:.7f}",
                        f"{final_lon:.7f}",
                        f"{float(alt):.2f}",
                    ]
                )
                rows_written += 1
                total_detections += 1

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

            if rows_written > 0:
                images_with_hits += 1
                cv2.imwrite(
                    os.path.join(
                        OUTPUT_DIR, base_filename.replace(".webp", "_processed.webp")
                    ),
                    frame,
                )
                print(f"           💾 Saved {rows_written} target(s).\n")

    print(f"\n✅ Finished processing. Valid Detections: {total_detections}")


if __name__ == "__main__":
    main()

