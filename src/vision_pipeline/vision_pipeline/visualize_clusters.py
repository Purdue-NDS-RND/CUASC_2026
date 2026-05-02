# /// script
# dependencies = [
#   "numpy",
#   "pandas",
#   "matplotlib",
# ]
# ///
"""
GPS Cluster Visualizer for CUASC Mission Log
Run on your local machine:
    pip install pandas matplotlib scipy numpy
    python visualize_clusters.py
Saves output to target_clusters/cluster_analysis.png
"""

import math
import os

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.cm import get_cmap
from matplotlib.colors import Normalize

# ── Config ────────────────────────────────────────────────────────────────────
FLIGHT_DIR = "/home/nds02/CUASC_Mission_Data/Flight_20260424_205344"

CSV_PATH = os.path.join(FLIGHT_DIR, "mission_log.csv")
OUTPUT_DIR = os.path.join(FLIGHT_DIR, "cluster_analysis")
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "cluster_analysis.png")
R_EARTH = 6378137.0
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Load ──────────────────────────────────────────────────────────────────────
df = pd.read_csv(CSV_PATH)
df.columns = df.columns.str.strip()
df["Latitude"] = pd.to_numeric(df["Latitude"], errors="coerce")
df["Longitude"] = pd.to_numeric(df["Longitude"], errors="coerce")
df["YOLO_Confidence"] = pd.to_numeric(df["YOLO_Confidence"], errors="coerce")
df.dropna(subset=["Latitude", "Longitude"], inplace=True)

# ── Convert GPS → local metres (ENU relative to centroid) ─────────────────────
lat0 = df["Latitude"].mean()
lon0 = df["Longitude"].mean()


def gps_to_m(lat, lon):
    dy = (lat - lat0) * R_EARTH * (math.pi / 180.0)
    dx = (lon - lon0) * R_EARTH * math.cos(math.radians(lat0)) * (math.pi / 180.0)
    return dx, dy  # East, North in metres


df["x_m"], df["y_m"] = zip(
    *df.apply(lambda r: gps_to_m(r["Latitude"], r["Longitude"]), axis=1)
)


# ── Per-image spread stats ─────────────────────────────────────────────────────
def spread_m(group):
    if len(group) < 2:
        return 0.0
    cx, cy = group["x_m"].mean(), group["y_m"].mean()
    dists = np.sqrt((group["x_m"] - cx) ** 2 + (group["y_m"] - cy) ** 2)
    return dists.max()


spread = df.groupby("Image_Name").apply(spread_m).rename("spread_m").reset_index()
df = df.merge(spread, on="Image_Name")

# ── Timeline order for colour mapping ─────────────────────────────────────────
image_order = df.drop_duplicates("Image_Name")[["Image_Name", "Time_UTC"]].copy()
image_order = image_order.sort_values("Time_UTC").reset_index(drop=True)
image_order["order"] = image_order.index
df = df.merge(image_order[["Image_Name", "order"]], on="Image_Name")

# ── Summary stats print ───────────────────────────────────────────────────────
print(f"\n{'=' * 60}")
print(f"  Mission Log Summary")
print(f"{'=' * 60}")
print(f"  Total detections     : {len(df)}")
print(f"  Unique target images : {df['Image_Name'].nunique()}")
print(
    f"  Lat range            : {df['Latitude'].min():.7f} → {df['Latitude'].max():.7f}"
)
print(
    f"  Lon range            : {df['Longitude'].min():.7f} → {df['Longitude'].max():.7f}"
)
print(
    f"  Total area (m)       : {df['x_m'].max() - df['x_m'].min():.1f} E/W  ×  "
    f"{df['y_m'].max() - df['y_m'].min():.1f} N/S"
)
print(f"\n  Per-image detection spread (max radius from centroid):")
print(f"  {'Image':<20} {'Detections':>10} {'Max spread (m)':>15} {'Avg conf':>10}")
print(f"  {'-' * 57}")
for img, grp in df.groupby("Image_Name"):
    sp = grp["spread_m"].iloc[0]
    n = len(grp)
    cfg = grp["YOLO_Confidence"].mean()
    flag = "  ⚠️  HIGH SPREAD" if sp > 15 else ""
    print(f"  {img:<20} {n:>10} {sp:>14.1f}m {cfg:>10.2f}{flag}")

# Convert 20ft and 65ft to metres for reference lines
ft20_m = 20 * 0.3048  # 6.1 m
ft65_m = 65 * 0.3048  # 19.8 m
print(f"\n  Reference: 20 ft = {ft20_m:.1f} m | 65 ft = {ft65_m:.1f} m")
print(f"{'=' * 60}\n")

# ── Figure layout: 2×2 grid ───────────────────────────────────────────────────
fig = plt.figure(figsize=(20, 16))
fig.suptitle(
    "CUASC Mission GPS Cluster Analysis", fontsize=16, fontweight="bold", y=0.98
)

# ── Panel 1: All detections coloured by image, with spread circles ─────────────
ax1 = fig.add_subplot(2, 2, 1)
cmap = get_cmap("tab20")
norm = Normalize(vmin=0, vmax=df["order"].max())

for img, grp in df.groupby("Image_Name"):
    colour = cmap(norm(grp["order"].iloc[0]))
    ax1.scatter(grp["x_m"], grp["y_m"], color=colour, s=30, zorder=3, alpha=0.8)
    # Draw spread circle around centroid
    cx, cy = grp["x_m"].mean(), grp["y_m"].mean()
    sp = grp["spread_m"].iloc[0]
    ax1.scatter(cx, cy, marker="+", s=80, color=colour, zorder=4)
    if sp > 0:
        circle = plt.Circle(
            (cx, cy), sp, color=colour, fill=False, alpha=0.3, linewidth=0.8
        )
        ax1.add_patch(circle)

# Reference circles from origin
for r, label, style in [(ft20_m, "20 ft", "--"), (ft65_m, "65 ft", ":")]:
    circle = plt.Circle(
        (0, 0),
        r,
        color="red",
        fill=False,
        linestyle=style,
        linewidth=1.5,
        label=f"{label} = {r:.1f}m radius",
    )
    ax1.add_patch(circle)

ax1.set_aspect("equal")
ax1.set_xlabel("East (m)")
ax1.set_ylabel("North (m)")
ax1.set_title("All Detections by Image\n(+ = centroid, circle = max spread)")
ax1.grid(True, alpha=0.3)
ax1.legend(loc="upper right", fontsize=8)
ax1.axhline(0, color="gray", linewidth=0.5)
ax1.axvline(0, color="gray", linewidth=0.5)

# ── Panel 2: Detections coloured by confidence ────────────────────────────────
ax2 = fig.add_subplot(2, 2, 2)
sc = ax2.scatter(
    df["x_m"],
    df["y_m"],
    c=df["YOLO_Confidence"],
    cmap="RdYlGn",
    vmin=0.5,
    vmax=1.0,
    s=25,
    alpha=0.85,
    zorder=3,
)
plt.colorbar(sc, ax=ax2, label="YOLO Confidence")
ax2.set_aspect("equal")
ax2.set_xlabel("East (m)")
ax2.set_ylabel("North (m)")
ax2.set_title("Detections Coloured by Confidence\n(green=high, red=low)")
ax2.grid(True, alpha=0.3)
ax2.axhline(0, color="gray", linewidth=0.5)
ax2.axvline(0, color="gray", linewidth=0.5)

# ── Panel 3: Per-image spread over time ───────────────────────────────────────
ax3 = fig.add_subplot(2, 2, 3)
spread_sorted = df.drop_duplicates("Image_Name").sort_values("order")
colours = ["red" if s > ft20_m else "steelblue" for s in spread_sorted["spread_m"]]
bars = ax3.bar(
    range(len(spread_sorted)),
    spread_sorted["spread_m"],
    color=colours,
    alpha=0.8,
    edgecolor="white",
    linewidth=0.5,
)
ax3.axhline(
    ft20_m,
    color="orange",
    linestyle="--",
    linewidth=1.5,
    label=f"20 ft = {ft20_m:.1f} m",
)
ax3.axhline(
    ft65_m, color="red", linestyle=":", linewidth=1.5, label=f"65 ft = {ft65_m:.1f} m"
)
ax3.set_xlabel("Image (chronological order)")
ax3.set_ylabel("Max spread from centroid (m)")
ax3.set_title("Per-Image Detection Spread Over Time\n(red bars exceed 20 ft threshold)")
ax3.legend()
ax3.grid(True, alpha=0.3, axis="y")

# Annotate worst offenders
for i, (_, row) in enumerate(spread_sorted.iterrows()):
    if row["spread_m"] > ft65_m:
        ax3.text(
            i,
            row["spread_m"] + 0.3,
            row["Image_Name"].replace(".jpg", ""),
            ha="center",
            va="bottom",
            fontsize=5,
            rotation=90,
            color="red",
        )

# ── Panel 4: Spread vs confidence scatter ─────────────────────────────────────
ax4 = fig.add_subplot(2, 2, 4)
per_img = (
    df.groupby("Image_Name")
    .agg(
        spread_m=("spread_m", "first"),
        mean_conf=("YOLO_Confidence", "mean"),
        n_det=("YOLO_Confidence", "count"),
    )
    .reset_index()
)

sc4 = ax4.scatter(
    per_img["mean_conf"],
    per_img["spread_m"],
    c=per_img["n_det"],
    cmap="plasma",
    s=60,
    alpha=0.85,
    zorder=3,
)
plt.colorbar(sc4, ax=ax4, label="Detections per image")
ax4.axhline(
    ft20_m,
    color="orange",
    linestyle="--",
    linewidth=1.5,
    label=f"20 ft = {ft20_m:.1f}m",
)
ax4.axhline(
    ft65_m, color="red", linestyle=":", linewidth=1.5, label=f"65 ft = {ft65_m:.1f}m"
)

# Linear trend
z = np.polyfit(per_img["mean_conf"], per_img["spread_m"], 1)
p = np.poly1d(z)
xs = np.linspace(per_img["mean_conf"].min(), per_img["mean_conf"].max(), 100)
ax4.plot(xs, p(xs), "k--", linewidth=1, alpha=0.5, label="Trend")

ax4.set_xlabel("Mean YOLO Confidence per Image")
ax4.set_ylabel("Max spread from centroid (m)")
ax4.set_title("Spread vs Confidence\n(do high-confidence frames cluster tighter?)")
ax4.legend(fontsize=8)
ax4.grid(True, alpha=0.3)

plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.savefig(OUTPUT_FILE, dpi=150, bbox_inches="tight")
print(f"✅ Saved → {OUTPUT_FILE}")
plt.show()
