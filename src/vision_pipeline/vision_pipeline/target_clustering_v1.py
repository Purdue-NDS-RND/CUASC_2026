# /// script
# dependencies = [
#   "numpy",
#   "scipy",
#   "opencv-python",
# ]
# ///
"""
Target Clustering & Chip Sorter — Dual CSV Edition

Runs the complete clustering pipeline twice:
  1. mission_log_full.csv  → target_clusters/full/
  2. mission_log_prime.csv → target_clusters/prime/

Then prints a side-by-side comparison so you can immediately see how
much the prime filter tightens your GPS centroids.

Update FLIGHT_DIR below to point at your flight folder, then run:
    python target_clustering.py
"""

import csv
import math
import os
import shutil

import cv2
import numpy as np
import scipy.cluster.hierarchy as hcluster

# ---------------------------------------------------------------------------
# CONFIGURATION — update FLIGHT_DIR before each run
# ---------------------------------------------------------------------------
FLIGHT_DIR = "/home/nds2/CUASC_Mission_Data/Flight_20260424_205344"

CSV_FULL = os.path.join(FLIGHT_DIR, "mission_log_full.csv")
CSV_PRIME = os.path.join(FLIGHT_DIR, "mission_log_prime.csv")

PROCESSED_IMG_DIR = os.path.join(FLIGHT_DIR, "targets")

CLUSTERS_DIR = os.path.join(FLIGHT_DIR, "target_clusters")
CLUSTERS_FULL_DIR = os.path.join(CLUSTERS_DIR, "full")
CLUSTERS_PRIME_DIR = os.path.join(CLUSTERS_DIR, "prime")

TOLERANCE = 5  # metres — complete-linkage max diameter per cluster
R_EARTH = 6378137.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def load_csv(csv_path: str) -> list:
    """Load a mission log CSV into a list of detection dicts.
    Handles both old (no Is_Prime) and new (with Is_Prime) formats."""
    detections = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            detections.append(
                {
                    "chip_image": row["Image_Name"],
                    "conf": float(row["YOLO_Confidence"]),
                    "lat": float(row["Latitude"]),
                    "lon": float(row["Longitude"]),
                    "time": row["Time_UTC"],
                    "u": int(row.get("Pixel_U", 0)),
                    "v": int(row.get("Pixel_V", 0)),
                    "w": int(row.get("BBox_W", 0)),
                    "h": int(row.get("BBox_H", 0)),
                    "is_prime": row.get("Is_Prime", "True"),
                }
            )
    return detections


def project_to_metres(detections: list):
    """Project lat/lon to a local flat ENU grid centred on the mean position."""
    mean_lat = np.mean([d["lat"] for d in detections])
    mean_lon = np.mean([d["lon"] for d in detections])
    lat_scale = math.cos(math.radians(mean_lat))
    coords = []
    for d in detections:
        x = (d["lon"] - mean_lon) * R_EARTH * lat_scale * (math.pi / 180.0)
        y = (d["lat"] - mean_lat) * R_EARTH * (math.pi / 180.0)
        coords.append([x, y])
    return np.array(coords), mean_lat, mean_lon


def cluster_detections(detections: list, tolerance: float) -> dict:
    """Run complete-linkage hierarchical clustering. Returns {cluster_id: [detections]}."""
    if len(detections) == 0:
        return {}

    coords, _, _ = project_to_metres(detections)

    if len(detections) == 1:
        return {1: detections}

    cluster_ids = hcluster.fclusterdata(
        coords, t=tolerance, criterion="distance", method="complete"
    )

    clusters = {}
    for i, cid in enumerate(cluster_ids):
        clusters.setdefault(cid, []).append(detections[i])
    return clusters


def metres_spread(cluster_items: list) -> float:
    """Max distance from any point to the cluster centroid (metres)."""
    if len(cluster_items) < 2:
        return 0.0
    lats = [d["lat"] for d in cluster_items]
    lons = [d["lon"] for d in cluster_items]
    clat = np.mean(lats)
    clon = np.mean(lons)
    lat_scale = math.cos(math.radians(clat))
    max_r = 0.0
    for d in cluster_items:
        dy = (d["lat"] - clat) * R_EARTH * (math.pi / 180.0)
        dx = (d["lon"] - clon) * R_EARTH * lat_scale * (math.pi / 180.0)
        max_r = max(max_r, math.hypot(dx, dy))
    return max_r


def save_cluster_outputs(
    clusters: dict,
    output_dir: str,
    img_src_dir: str,
    label: str,
) -> list:
    """
    Write summary CSV, annotation CSV, and cropped chip folders.
    Returns a list of summary row dicts for the comparison table.
    """
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)

    summary_csv = os.path.join(output_dir, "cluster_summary.csv")
    annotation_csv = os.path.join(output_dir, "chip_annotations.csv")

    summary_rows = []

    with (
        open(summary_csv, "w", newline="") as sum_f,
        open(annotation_csv, "w", newline="") as ann_f,
    ):
        sum_writer = csv.writer(sum_f)
        sum_writer.writerow(
            [
                "Target_ID",
                "Detections",
                "Mean_Confidence",
                "Max_Confidence",
                "Averaged_Lat",
                "Averaged_Lon",
                "Spread_m",
            ]
        )

        ann_writer = csv.writer(ann_f)
        ann_writer.writerow(
            [
                "Chip_Filename",
                "Target_ID",
                "Confidence",
                "Lat",
                "Lon",
                "Time_UTC",
                "Is_Prime",
            ]
        )

        print(
            f"\n  {'Target':<8} {'Dets':>5} {'Avg Lat':>13} {'Avg Lon':>14} "
            f"{'Spread':>9} {'AvgConf':>8}"
        )
        print(f"  {'-' * 62}")

        for tid in sorted(clusters.keys()):
            items = clusters[tid]
            n = len(items)
            mean_conf = float(np.mean([d["conf"] for d in items]))
            max_conf = float(np.max([d["conf"] for d in items]))
            avg_lat = float(np.mean([d["lat"] for d in items]))
            avg_lon = float(np.mean([d["lon"] for d in items]))
            spread = metres_spread(items)

            sum_writer.writerow(
                [
                    tid,
                    n,
                    f"{mean_conf:.3f}",
                    f"{max_conf:.3f}",
                    f"{avg_lat:.7f}",
                    f"{avg_lon:.7f}",
                    f"{spread:.2f}",
                ]
            )

            target_folder = os.path.join(output_dir, f"Target_{tid}")
            os.makedirs(target_folder, exist_ok=True)
            images_copied = 0

            for item in items:
                ann_writer.writerow(
                    [
                        item["chip_image"],
                        tid,
                        f"{item['conf']:.2f}",
                        f"{item['lat']:.7f}",
                        f"{item['lon']:.7f}",
                        item["time"],
                        item["is_prime"],
                    ]
                )

                src = os.path.join(img_src_dir, item["chip_image"])
                dst = os.path.join(target_folder, item["chip_image"])

                if os.path.exists(src):
                    if item["w"] > 0 and item["h"] > 0:
                        img = cv2.imread(src)
                        pad = 30
                        y1 = max(0, int(item["v"] - item["h"] / 2 - pad))
                        y2 = min(img.shape[0], int(item["v"] + item["h"] / 2 + pad))
                        x1 = max(0, int(item["u"] - item["w"] / 2 - pad))
                        x2 = min(img.shape[1], int(item["u"] + item["w"] / 2 + pad))
                        cv2.imwrite(dst, img[y1:y2, x1:x2])
                    else:
                        shutil.copy2(src, dst)
                    images_copied += 1
                else:
                    print(f"  ⚠️  Missing image: {src}")

            print(
                f"  T{tid:<7} {n:>5} {avg_lat:>13.7f} {avg_lon:>14.7f} "
                f"{spread:>8.2f}m {mean_conf:>8.3f}"
            )

            summary_rows.append(
                {
                    "tid": tid,
                    "n": n,
                    "avg_lat": avg_lat,
                    "avg_lon": avg_lon,
                    "spread": spread,
                    "mean_conf": mean_conf,
                }
            )

    print(f"\n  ✅ Summary    → {summary_csv}")
    print(f"  ✅ Annotations → {annotation_csv}")
    print(f"  ✅ Chip dirs   → {output_dir}/Target_*/")
    return summary_rows


def print_comparison(full_rows: list, prime_rows: list):
    """Side-by-side table showing how the prime filter changed each cluster."""
    print(f"\n{'=' * 75}")
    print(f"  FULL vs PRIME COMPARISON  (tolerance={TOLERANCE}m, complete linkage)")
    print(f"{'=' * 75}")
    print(f"  {'':10} {'─── FULL ───':>30}   {'─── PRIME ───':>30}")
    print(
        f"  {'Target':<10} {'Dets':>5} {'Spread':>8} {'AvgConf':>8}   "
        f"{'Dets':>5} {'Spread':>8} {'AvgConf':>8}  {'Δ Spread':>10}"
    )
    print(f"  {'-' * 73}")

    # Index prime rows by closest lat/lon match to each full row
    def find_prime_match(full_row):
        if not prime_rows:
            return None
        lat_scale = math.cos(math.radians(full_row["avg_lat"]))

        def dist(p):
            dy = (p["avg_lat"] - full_row["avg_lat"]) * R_EARTH * math.pi / 180
            dx = (
                (p["avg_lon"] - full_row["avg_lon"])
                * R_EARTH
                * lat_scale
                * math.pi
                / 180
            )
            return math.hypot(dx, dy)

        best = min(prime_rows, key=dist)
        return best if dist(best) < 50 else None  # only match within 50m

    for fr in sorted(full_rows, key=lambda r: r["tid"]):
        pr = find_prime_match(fr)
        if pr:
            delta = pr["spread"] - fr["spread"]
            delta_str = f"{delta:+.2f}m"
            prime_str = f"{pr['n']:>5} {pr['spread']:>7.2f}m {pr['mean_conf']:>8.3f}"
        else:
            delta_str = "  no match"
            prime_str = f"{'—':>5} {'—':>8} {'—':>8}"

        print(
            f"  T{fr['tid']:<9} {fr['n']:>5} {fr['spread']:>7.2f}m "
            f"{fr['mean_conf']:>8.3f}   {prime_str}  {delta_str:>10}"
        )

    print(f"{'=' * 75}")
    if prime_rows:
        avg_full_spread = np.mean([r["spread"] for r in full_rows])
        avg_prime_spread = np.mean([r["spread"] for r in prime_rows])
        print(
            f"  Mean spread — FULL: {avg_full_spread:.2f}m  "
            f"PRIME: {avg_prime_spread:.2f}m  "
            f"Improvement: {avg_full_spread - avg_prime_spread:+.2f}m"
        )
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    os.makedirs(CLUSTERS_DIR, exist_ok=True)

    # ── Full CSV ──────────────────────────────────────────────────────────
    full_rows = []
    if os.path.exists(CSV_FULL):
        print(f"\n{'=' * 65}")
        print(f"  FULL CSV  →  {CSV_FULL}")
        print(f"{'=' * 65}")
        full_dets = load_csv(CSV_FULL)
        print(f"  Loaded {len(full_dets)} detections")
        if full_dets:
            full_clusters = cluster_detections(full_dets, TOLERANCE)
            print(f"  → {len(full_clusters)} clusters at {TOLERANCE}m tolerance")
            full_rows = save_cluster_outputs(
                full_clusters, CLUSTERS_FULL_DIR, PROCESSED_IMG_DIR, "FULL"
            )
    else:
        print(f"\n⚠️  Full CSV not found: {CSV_FULL}")
        print(
            "   (This is expected if you haven't flown with the updated mission_logger yet)"
        )

    # ── Prime CSV ─────────────────────────────────────────────────────────
    prime_rows = []
    if os.path.exists(CSV_PRIME):
        print(f"\n{'=' * 65}")
        print(f"  PRIME CSV  →  {CSV_PRIME}")
        print(f"{'=' * 65}")
        prime_dets = load_csv(CSV_PRIME)
        print(f"  Loaded {len(prime_dets)} detections (center-zone only)")
        if prime_dets:
            prime_clusters = cluster_detections(prime_dets, TOLERANCE)
            print(f"  → {len(prime_clusters)} clusters at {TOLERANCE}m tolerance")
            prime_rows = save_cluster_outputs(
                prime_clusters, CLUSTERS_PRIME_DIR, PROCESSED_IMG_DIR, "PRIME"
            )
    else:
        print(f"\n⚠️  Prime CSV not found: {CSV_PRIME}")
        print(
            "   (This file is written by the updated mission_logger — fly again to generate it)"
        )

    # ── Comparison ────────────────────────────────────────────────────────
    if full_rows and prime_rows:
        print_comparison(full_rows, prime_rows)
    elif full_rows:
        print("\n  (Prime CSV missing — comparison not available)")
    elif prime_rows:
        print("\n  (Full CSV missing — comparison not available)")
    else:
        print("\n  ❌ No data found in either CSV.")

    print(f"📁 All outputs under: {CLUSTERS_DIR}")


if __name__ == "__main__":
    main()
