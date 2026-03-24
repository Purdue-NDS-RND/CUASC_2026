"""
Target Visualizer Node (Matplotlib) - GPS Mode

Live 2D scatter plot of target observations and estimates in GPS coordinates.
Subscribes to localizer outputs and displays them in real-time.

Features:
  - Color-coded observations per target
  - Larger markers for filtered estimates
  - Drone GPS position overlay
  - Auto-scaling based on GPS bounds
  - GPS coordinate display

Usage:
  ros2 run drone_control target_visualizer
  ros2 run drone_control target_visualizer --ros-args -p update_rate_hz:=10.0
"""

import json
from collections import deque
from typing import Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for live updates
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np


# Match colors from localizer
TARGET_COLORS = [
    '#FF0000',  # Red
    '#00FF00',  # Green
    '#0000FF',  # Blue
    '#FFFF00',  # Yellow
    '#FF00FF',  # Magenta
    '#00FFFF',  # Cyan
    '#FF8000',  # Orange
    '#8000FF',  # Purple
    '#008000',  # Dark green
    '#808080',  # Gray
]


class TargetVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("target_visualizer")

        # Parameters
        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("history_size", 200)  # Max points per target
        self.declare_parameter("auto_scale", True)
        self.declare_parameter("show_drone", True)
        self.declare_parameter("show_grid", True)
        self.declare_parameter("marker_size_obs", 10)
        self.declare_parameter("marker_size_est", 150)

        self._history_size = self.get_parameter("history_size").get_parameter_value().integer_value

        # State - now using GPS coordinates (lat, lon)
        self._drone_gps: Optional[Tuple[float, float]] = None  # (lat, lon)
        self._target_observations: Dict[str, deque] = {}  # target_id -> deque of (lat, lon)
        self._target_estimates: Dict[str, Tuple[float, float]] = {}  # target_id -> (lat, lon)
        self._target_colors: Dict[str, str] = {}
        self._next_color_idx = 0
        self._estimates_json: Optional[dict] = None
        self._spawned_target_gps: Optional[Tuple[float, float]] = None  # Direct from target_spawner

        # Subscribers
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._on_drone_gps,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/drone_control/targets/estimates_gps",
            self._on_estimates,
            10,
        )
        # Subscribe to spawned target GPS directly
        self.create_subscription(
            NavSatFix,
            "/drone_control/target/gps",
            self._on_spawned_target_gps,
            10,
        )

        # Setup matplotlib
        plt.ion()  # Interactive mode
        self._fig, self._ax = plt.subplots(figsize=(10, 10))
        self._fig.canvas.manager.set_window_title("Target Localizer - GPS View")

        # Timer for plot updates
        rate = self.get_parameter("update_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._update_plot)

        self.get_logger().info("GPS Target visualizer ready - matplotlib window opened")

    def _on_drone_gps(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:  # Valid fix
            self._drone_gps = (msg.latitude, msg.longitude)

    def _on_spawned_target_gps(self, msg: NavSatFix) -> None:
        """Receive spawned target position directly from target_spawner."""
        self._spawned_target_gps = (msg.latitude, msg.longitude)

    def _on_estimates(self, msg: String) -> None:
        """Parse JSON estimates which contain per-target GPS data."""
        try:
            data = json.loads(msg.data)
            self._estimates_json = data

            for target_id, target_data in data.get("targets", {}).items():
                # Assign color if new target
                if target_id not in self._target_colors:
                    self._target_colors[target_id] = TARGET_COLORS[self._next_color_idx % len(TARGET_COLORS)]
                    self._target_observations[target_id] = deque(maxlen=self._history_size)
                    self._next_color_idx += 1

                # Update estimate from GPS data
                gps = target_data.get("gps", {})
                if "latitude" in gps and "longitude" in gps:
                    lat = gps["latitude"]
                    lon = gps["longitude"]
                    self._target_estimates[target_id] = (lat, lon)
                    # Add to observation history
                    self._target_observations[target_id].append((lat, lon))

        except json.JSONDecodeError:
            pass

    def _update_plot(self) -> None:
        """Redraw the matplotlib plot with GPS coordinates."""
        if not plt.fignum_exists(self._fig.number):
            self.get_logger().info("Plot window closed, shutting down")
            raise SystemExit

        self._ax.clear()

        # Grid
        if self.get_parameter("show_grid").get_parameter_value().bool_value:
            self._ax.grid(True, alpha=0.3)

        marker_size_obs = self.get_parameter("marker_size_obs").get_parameter_value().integer_value
        marker_size_est = self.get_parameter("marker_size_est").get_parameter_value().integer_value

        all_lats, all_lons = [], []

        # Plot observations (smaller dots) - using lon as X, lat as Y
        for target_id, obs_history in self._target_observations.items():
            if not obs_history:
                continue

            color = self._target_colors.get(target_id, '#888888')
            lats = [p[0] for p in obs_history]
            lons = [p[1] for p in obs_history]
            all_lats.extend(lats)
            all_lons.extend(lons)

            # Fade older points
            alphas = np.linspace(0.2, 0.7, len(lats))
            for i, (lat, lon) in enumerate(zip(lats, lons)):
                self._ax.scatter(lon, lat, c=color, s=marker_size_obs, alpha=alphas[i], edgecolors='none')

        # Plot estimates (larger markers with labels)
        for target_id, (lat, lon) in self._target_estimates.items():
            color = self._target_colors.get(target_id, '#888888')
            self._ax.scatter(lon, lat, c=color, s=marker_size_est, marker='o', edgecolors='black', linewidths=2, zorder=10)
            self._ax.annotate(
                f"T{target_id}",
                (lon, lat),
                textcoords="offset points",
                xytext=(10, 10),
                fontsize=12,
                fontweight='bold',
                color=color,
            )

        # Plot drone position
        if self.get_parameter("show_drone").get_parameter_value().bool_value and self._drone_gps is not None:
            d_lat, d_lon = self._drone_gps
            self._ax.scatter(d_lon, d_lat, c='black', s=200, marker='^', zorder=20, label='Drone')
            all_lats.append(d_lat)
            all_lons.append(d_lon)

        # Plot spawned target position (red X)
        if self._spawned_target_gps is not None:
            t_lat, t_lon = self._spawned_target_gps
            self._ax.scatter(t_lon, t_lat, c='red', s=300, marker='X', zorder=15, label='Target', edgecolors='darkred', linewidths=2)
            all_lats.append(t_lat)
            all_lons.append(t_lon)
            all_lons.append(d_lon)

        # Axis scaling
        if self.get_parameter("auto_scale").get_parameter_value().bool_value and all_lats and all_lons:
            # Add margin in degrees (roughly 20m at mid-latitudes)
            margin = 0.0002
            lat_min, lat_max = min(all_lats) - margin, max(all_lats) + margin
            lon_min, lon_max = min(all_lons) - margin, max(all_lons) + margin
            
            # Keep aspect ratio approximately 1:1 (adjust for latitude)
            lat_center = (lat_min + lat_max) / 2
            lon_range = lon_max - lon_min
            lat_range = lat_max - lat_min
            
            # Correct for longitude compression at higher latitudes
            lon_correction = np.cos(np.radians(lat_center))
            corrected_lon_range = lon_range * lon_correction
            
            max_range = max(corrected_lon_range, lat_range)
            
            self._ax.set_xlim(lon_min, lon_max)
            self._ax.set_ylim(lat_min, lat_max)

        self._ax.set_xlabel("Longitude", fontsize=12)
        self._ax.set_ylabel("Latitude", fontsize=12)
        self._ax.set_title("Target Localization - GPS View", fontsize=14)
        
        # Format axis labels as GPS coordinates
        self._ax.xaxis.set_major_formatter(plt.FormatStrFormatter('%.6f'))
        self._ax.yaxis.set_major_formatter(plt.FormatStrFormatter('%.6f'))

        # Info text
        info_lines = []
        if self._drone_gps:
            info_lines.append(f"Drone: ({self._drone_gps[0]:.7f}, {self._drone_gps[1]:.7f})")
        info_lines.append(f"Targets: {len(self._target_estimates)}")

        if self._estimates_json:
            for tid, tdata in self._estimates_json.get("targets", {}).items():
                n_obs = tdata.get("num_observations", 0)
                gps = tdata.get("gps", {})
                if gps:
                    info_lines.append(
                        f"  T{tid}: {n_obs} obs\n"
                        f"    GPS: ({gps.get('latitude', 0):.7f}, {gps.get('longitude', 0):.7f})"
                    )
                else:
                    info_lines.append(f"  T{tid}: {n_obs} obs")

        info_text = "\n".join(info_lines)
        self._ax.text(
            0.02, 0.98, info_text,
            transform=self._ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8)
        )

        self._fig.canvas.draw()
        self._fig.canvas.flush_events()


def main() -> None:
    rclpy.init()
    node = TargetVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
