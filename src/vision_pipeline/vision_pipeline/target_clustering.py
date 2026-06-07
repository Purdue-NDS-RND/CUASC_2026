# /// script
# dependencies = [
#   "numpy",
#   "scipy",
#   "opencv-python",
#   "scikit-learn",
# ]
# ///
"""
Target Clustering & Chip Sorter — Dual CSV Edition
Updated with command-line argument parser support to bypass manual hardcoding.

Usage:
    python3 target_clustering.py --flight_dir /path/to/Flight_YYYYMMDD_HHMMSS
"""

import argparse
import csv
import math
import os
import shutil
import sys

import cv2
import numpy as np
from sklearn.cluster import DBSCAN

# ---------------------------------------------------------------------------
# DEFAULT CONFIGURATION — used if no CLI arguments are supplied
# ---------------------------------------------------------------------------
DEFAULT_FLIGHT_DIR = "/home/samuel-yoon/training_cuasc_20260507/CUASC_Mission_Data/Flight_20260507_154051"
R_EARTH = 6378137.0


def load_csv(csv_path):
    if not os.path.exists(csv_path):
        print(f"  [Warning] CSV File not found: {csv_path}")
        return []

    detections = []
    with open(csv_path, mode="r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Strip trailing/leading spaces from dict keys & values
            clean_row = {k.strip(): v.strip() for k, v in row.items() if k is not None}
            detections.append(clean_row)
    return detections


def cluster_detections(
    detections,
    eps_meters=3.0,
    min_samples=3,
):
    if len(detections) == 0:
        return {}

    coords = detections_to_meters(detections)

    db = DBSCAN(
        eps=eps_meters,
        min_samples=min_samples,
    )

    labels = db.fit_predict(coords)

    clusters = {}

    valid_labels = sorted(set(labels) - {-1})

    label_map = {
        old_label: new_id for new_id, old_label in enumerate(valid_labels, start=1)
    }

    for det, label in zip(detections, labels):
        if label == -1:
            continue

        cluster_id = label_map[label]
        clusters.setdefault(cluster_id, []).append(det)
    return clusters


def detections_to_meters(detections):
    mean_lat = np.mean([float(d["Latitude"]) for d in detections])

    lat_scale = math.cos(math.radians(mean_lat))

    coords = []

    for d in detections:
        lat = float(d["Latitude"])
        lon = float(d["Longitude"])

        x = lon * R_EARTH * lat_scale * math.pi / 180.0
        y = lat * R_EARTH * math.pi / 180.0

        coords.append([x, y])

    return np.array(coords)


def save_cluster_outputs(clusters, output_root, img_dir, prefix_label):
    os.makedirs(output_root, exist_ok=True)
    summary_rows = []

    for cluster_id, dets in clusters.items():
        cluster_folder = os.path.join(output_root, f"cluster_{cluster_id:02d}")
        os.makedirs(cluster_folder, exist_ok=True)

        lats = [float(d["Latitude"]) for d in dets]
        lons = [float(d["Longitude"]) for d in dets]

        center_lat = np.mean(lats)
        center_lon = np.mean(lons)

        spread_m = metres_spread(
            dets,
            center_lat,
            center_lon,
        )

        best_det = max(dets, key=lambda d: float(d["YOLO_Confidence"]))

        summary_rows.append(
            {
                "Cluster_ID": f"{prefix_label}_cluster_{cluster_id:02d}",
                "Center_Lat": f"{center_lat:.7f}",
                "Center_Lon": f"{center_lon:.7f}",
                "Spread_m": f"{spread_m:.2f}",
                "Detections_Count": len(dets),
                "Mean_Confidence": (
                    f"{np.mean([float(d['YOLO_Confidence']) for d in dets]):.3f}"
                ),
                "Best_Detection_Confidence": best_det["YOLO_Confidence"],
                "Best_Detection_Image": best_det["Image_Name"],
            }
        )

        print(
            f"Cluster {cluster_id:02d} | "
            f"N={len(dets):3d} | "
            f"Spread={spread_m:6.2f}m | "
            f"Center=({center_lat:.7f}, {center_lon:.7f})"
        )

        # Copy high-confidence crop images into cluster groups for offline sorting
        for det in dets:
            src_path = os.path.join(img_dir, det["Image_Name"])
            if os.path.exists(src_path):
                dest_path = os.path.join(cluster_folder, det["Image_Name"])
                shutil.copy2(src_path, dest_path)

    # Save a cluster summary CSV report
    summary_csv = os.path.join(output_root, f"cluster_summary_{prefix_label}.csv")
    if summary_rows:
        with open(summary_csv, mode="w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=summary_rows[0].keys())
            writer.writeheader()
            writer.writerows(summary_rows)
        print(f"  📝 Saved cluster summary report: {summary_csv}")

    return summary_rows


def metres_spread(cluster_items, centre_lat, centre_lon):
    if len(cluster_items) < 2:
        return 0.0

    lat_scale = math.cos(math.radians(centre_lat))

    max_r = 0.0

    for d in cluster_items:
        lat = float(d["Latitude"])
        lon = float(d["Longitude"])

        dy = (lat - centre_lat) * R_EARTH * math.pi / 180.0

        dx = (lon - centre_lon) * R_EARTH * lat_scale * math.pi / 180.0

        max_r = max(max_r, math.hypot(dx, dy))

    return max_r


def main():
    parser = argparse.ArgumentParser(
        description="CUASC Post-Flight Target Clustering Pipeline"
    )
    parser.add_argument(
        "--flight_dir",
        type=str,
        default=DEFAULT_FLIGHT_DIR,
        help="Path to the standard Flight folder containing mission CSV files.",
    )
    parser.add_argument(
        "--eps",
        type=float,
        default=3.0,
        help="DBSCAN clustering radius in meters",
    )

    parser.add_argument(
        "--min_samples",
        type=int,
        default=3,
        help="Minimum detections required to form a cluster",
    )
    args = parser.parse_args()

    FLIGHT_DIR = os.path.abspath(os.path.normpath(args.flight_dir))
    CSV_FULL = os.path.join(
        FLIGHT_DIR, "mission_log_full_v2.csv"
    )  # Prefer singularity-free V2 logs
    CSV_PRIME = os.path.join(FLIGHT_DIR, "mission_log_prime_v2.csv")
    PROCESSED_IMG_DIR = os.path.join(FLIGHT_DIR, "targets")
    CLUSTERS_DIR = os.path.join(FLIGHT_DIR, "target_clusters")

    print(f"\n⚙️ Running Clustering Pipeline for: {FLIGHT_DIR}")

    if not os.path.exists(FLIGHT_DIR):
        print(f"❌ Error: Specified flight directory does not exist: {FLIGHT_DIR}")
        sys.exit(1)

    # Fallback to standard V1 logs if V2 does not exist
    if not os.path.exists(CSV_FULL) and os.path.exists(
        os.path.join(FLIGHT_DIR, "mission_log_full_v1.csv")
    ):
        CSV_FULL = os.path.join(FLIGHT_DIR, "mission_log_full_v1.csv")
        CSV_PRIME = os.path.join(FLIGHT_DIR, "mission_log_prime_v1.csv")

    full_rows = []
    if os.path.exists(CSV_FULL):
        print(f"\n📂 processing full datasets -> {CSV_FULL}")
        full_dets = load_csv(CSV_FULL)
        print(f"  Loaded {len(full_dets)} valid detections")
        if full_dets:
            full_clusters = cluster_detections(
                full_dets,
                eps_meters=args.eps,
                min_samples=args.min_samples,
            )
            full_rows = save_cluster_outputs(
                full_clusters,
                os.path.join(CLUSTERS_DIR, "full"),
                PROCESSED_IMG_DIR,
                "FULL",
            )

    prime_rows = []
    if os.path.exists(CSV_PRIME):
        print(f"\n📂 processing center-zone prime datasets -> {CSV_PRIME}")
        prime_dets = load_csv(CSV_PRIME)
        print(f"  Loaded {len(prime_dets)} prime center-zone detections")
        if prime_dets:
            prime_clusters = cluster_detections(
                prime_dets,
                eps_meters=args.eps,
                min_samples=args.min_samples,
            )
            prime_rows = save_cluster_outputs(
                prime_clusters,
                os.path.join(CLUSTERS_DIR, "prime"),
                PROCESSED_IMG_DIR,
                "PRIME",
            )

    if full_rows or prime_rows:
        print(
            "\n🎉 Target clustering finished! Output folders located at: target_clusters/"
        )


if __name__ == "__main__":
    main()
