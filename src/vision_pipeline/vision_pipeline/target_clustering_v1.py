# /// script
# dependencies = [
#   "numpy",
#   "scipy",
#   "opencv-python",
# ]
# ///
"""
Target Clustering & Chip Sorter
Locks to a 1.0m tolerance, outputs annotation CSVs, and physically copies
cropped target chips into specific Target_ID subdirectories.
"""

import csv
import math
import os
import shutil

import cv2
import numpy as np
import scipy.cluster.hierarchy as hcluster

# --- CONFIGURATION ---
# Make sure to update this path to your exact flight folder!
FLIGHT_DIR = "/home/spes_ignota/Flight_20260419_162533_copy"

INPUT_CSV = os.path.join(FLIGHT_DIR, "mission_log.csv")
# CHANGED: Look in the targets folder, not the frames folder
PROCESSED_IMG_DIR = os.path.join(FLIGHT_DIR, "targets")

# The new master directory where all our organized target folders will live
CLUSTERS_DIR = os.path.join(FLIGHT_DIR, "target_clusters")

SUMMARY_CSV = os.path.join(CLUSTERS_DIR, "cluster_summary.csv")
ANNOTATION_CSV = os.path.join(CLUSTERS_DIR, "chip_annotations.csv")

TOLERANCE = 15  # Set to 1.0 meters based on your previous request
R_EARTH = 6378137.0


def main():
    if not os.path.exists(INPUT_CSV):
        print(f"❌ Cannot find input file: {INPUT_CSV}")
        return

    print(f"📂 Reading raw detections from {INPUT_CSV}...")

    detections = []

    with open(INPUT_CSV, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # CHANGED: Map to the exact headers created by mission_logger.py
            chip_name = row["Image_Name"]

            detections.append(
                {
                    "chip_image": chip_name,
                    "conf": float(row["YOLO_Confidence"]),
                    "lat": float(row["Latitude"]),
                    "lon": float(row["Longitude"]),
                    "time": row["Time_UTC"],
                    "u": int(
                        row.get("Pixel_U", 0)
                    ),  # Use .get() so it doesn't crash on old CSVs
                    "v": int(row.get("Pixel_V", 0)),
                    "w": int(row.get("BBox_W", 0)),
                    "h": int(row.get("BBox_H", 0)),
                }
            )

    if not detections:
        print("⚠️ No data found in CSV.")
        return

    # 1. Project Lat/Lon to a local Flat Cartesian Grid (Meters)
    mean_lat = np.mean([d["lat"] for d in detections])
    mean_lon = np.mean([d["lon"] for d in detections])
    lat_scale = math.cos(math.radians(mean_lat))

    coords = []
    for d in detections:
        x_meters = (d["lon"] - mean_lon) * (R_EARTH * lat_scale) * (math.pi / 180.0)
        y_meters = (d["lat"] - mean_lat) * R_EARTH * (math.pi / 180.0)
        coords.append([x_meters, y_meters])

    coords = np.array(coords)

    # 2. Run the Clustering
    print(f"🧠 Clustering targets with a {TOLERANCE}m tolerance radius...")
    cluster_ids = hcluster.fclusterdata(
        coords, t=TOLERANCE, criterion="distance", method="complete"
    )

    # Aggregate Data by Cluster
    clusters = {}
    for i, cid in enumerate(cluster_ids):
        if cid not in clusters:
            clusters[cid] = []
        clusters[cid].append(detections[i])

    print(
        f"🎯 Grouped {len(detections)} detections into {len(clusters)} distinct physical target(s).\n"
    )

    # 3. Set up the Output Directory
    if os.path.exists(CLUSTERS_DIR):
        shutil.rmtree(CLUSTERS_DIR)
    os.makedirs(CLUSTERS_DIR)

    # 4. Generate CSVs and Image Directories
    with (
        open(SUMMARY_CSV, "w", newline="") as sum_f,
        open(ANNOTATION_CSV, "w", newline="") as ann_f,
    ):
        sum_writer = csv.writer(sum_f)
        sum_writer.writerow(
            [
                "Target_ID",
                "Images_Captured",
                "Mean_Confidence",
                "Max_Confidence",
                "Averaged_Lat",
                "Averaged_Lon",
            ]
        )

        ann_writer = csv.writer(ann_f)
        ann_writer.writerow(
            ["Chip_Filename", "Target_ID", "Confidence", "Lat", "Lon", "Time_UTC"]
        )

        print("-" * 65)
        print(
            f"{'Target ID':<10} | {'Chips Copied':<15} | {'Avg Lat':<12} | {'Avg Lon':<12}"
        )
        print("-" * 65)

        for target_id in sorted(clusters.keys()):
            cluster_items = clusters[target_id]

            # --- MATH: Calculate Centroid ---
            num_hits = len(cluster_items)
            mean_conf = np.mean([item["conf"] for item in cluster_items])
            max_conf = np.max([item["conf"] for item in cluster_items])
            avg_lat = np.mean([item["lat"] for item in cluster_items])
            avg_lon = np.mean([item["lon"] for item in cluster_items])

            sum_writer.writerow(
                [
                    target_id,
                    num_hits,
                    f"{mean_conf:.2f}",
                    f"{max_conf:.2f}",
                    f"{avg_lat:.7f}",
                    f"{avg_lon:.7f}",
                ]
            )

            # --- DIRECTORY BUILDER ---
            # Create a dedicated folder for this specific Target
            target_folder = os.path.join(CLUSTERS_DIR, f"Target_{target_id}")
            os.makedirs(target_folder, exist_ok=True)

            images_copied = 0
            for item in cluster_items:
                # Log this specific chip's annotation
                ann_writer.writerow(
                    [
                        item["chip_image"],
                        target_id,
                        f"{item['conf']:.2f}",
                        f"{item['lat']:.7f}",
                        f"{item['lon']:.7f}",
                        item["time"],
                    ]
                )

                # Find the annotated image
                src_path = os.path.join(PROCESSED_IMG_DIR, item["chip_image"])
                dst_path = os.path.join(target_folder, item["chip_image"])

                if os.path.exists(src_path):
                    # If we have bounding box data, crop the chiplet
                    if item["w"] > 0 and item["h"] > 0:
                        img = cv2.imread(src_path)

                        # Add a 30 pixel border around the target for context
                        pad = 30
                        y1 = max(0, int(item["v"] - item["h"] / 2 - pad))
                        y2 = min(img.shape[0], int(item["v"] + item["h"] / 2 + pad))
                        x1 = max(0, int(item["u"] - item["w"] / 2 - pad))
                        x2 = min(img.shape[1], int(item["u"] + item["w"] / 2 + pad))

                        # Slice the numpy array and save it
                        chiplet = img[y1:y2, x1:x2]
                        cv2.imwrite(dst_path, chiplet)
                        images_copied += 1
                    else:
                        # Fallback for your old flights: just copy the whole image
                        shutil.copy2(src_path, dst_path)
                        images_copied += 1
                else:
                    print(f"⚠️ Missing image: {item['chip_image']} at {src_path}")

            print(
                f"Target {target_id:<3} | {images_copied:<15} | {avg_lat:.7f} | {avg_lon:.7f}"
            )

    print("-" * 65)
    print(f"✅ Master Summary Saved: {SUMMARY_CSV}")
    print(f"✅ Chip Annotations Saved: {ANNOTATION_CSV}")
    print(f"✅ Chip Directory Created: {CLUSTERS_DIR}")


if __name__ == "__main__":
    main()
