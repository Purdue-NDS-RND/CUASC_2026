"""
Target Tracker Simulator Node

Simulates noisy GPS tracking of spawned targets (target_box_*).
Adds distance-dependent noise to simulate real sensor behavior:
- Far targets = more noise
- Close targets = less noise

Publishes noisy GPS estimates for the visualizer.

Usage:
  ros2 run drone_control target_tracker_sim
"""

import json
import math
import random
from typing import Optional, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import HomePosition
from std_msgs.msg import String


class TargetTrackerSim(Node):
    def __init__(self) -> None:
        super().__init__("target_tracker_sim")

        # Parameters
        self.declare_parameter("update_rate_hz", 5.0)
        self.declare_parameter("max_noise_m", 8.0)  # Max noise when far
        self.declare_parameter("min_noise_m", 0.5)  # Min noise when close
        self.declare_parameter("noise_falloff_dist_m", 60.0)  # Distance for max noise
        self.declare_parameter("detection_range_m", 100.0)  # Max detection range

        # State
        self._drone_gps: Optional[NavSatFix] = None
        self._drone_local: Optional[PoseStamped] = None
        self._home: Optional[HomePosition] = None
        self._target_gps: Optional[NavSatFix] = None
        self._target_local: Optional[Tuple[float, float]] = None  # (x, y) in local frame
        
        # Tracking state
        self._target_id = "1"
        self._observation_count = 0
        self._last_target_lat: Optional[float] = None
        self._last_target_lon: Optional[float] = None

        # Publishers
        self._estimates_pub = self.create_publisher(
            String, "/drone_control/targets/estimates_gps", 10
        )
        self._noisy_gps_pub = self.create_publisher(
            NavSatFix, "/drone_control/target/gps_noisy", 10
        )
        # Publish noisy GPS as a waypoint for navigation
        self._noisy_waypoint_pub = self.create_publisher(
            NavSatFix, "/drone_control/waypoint/gps_noisy", 10
        )

        # Subscribers
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._on_drone_gps,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_drone_local,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            HomePosition,
            "/mavros/home_position/home",
            self._on_home,
            10,
        )
        self.create_subscription(
            NavSatFix,
            "/drone_control/target/gps",
            self._on_target_gps,
            10,
        )

        # Timer
        rate = self.get_parameter("update_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        self.get_logger().info("Target tracker simulator ready")

    def _on_drone_gps(self, msg: NavSatFix) -> None:
        self._drone_gps = msg

    def _on_drone_local(self, msg: PoseStamped) -> None:
        self._drone_local = msg

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    def _on_target_gps(self, msg: NavSatFix) -> None:
        """Receive true target GPS from target_spawner."""
        # Check if target changed (new spawn)
        if (self._last_target_lat is not None and 
            (abs(msg.latitude - self._last_target_lat) > 0.0001 or
             abs(msg.longitude - self._last_target_lon) > 0.0001)):
            # New target spawned, increment ID
            self._target_id = str(int(self._target_id) + 1)
            self._observation_count = 0
            self.get_logger().info(f"New target detected, now tracking T{self._target_id}")
        
        self._target_gps = msg
        self._last_target_lat = msg.latitude
        self._last_target_lon = msg.longitude
        
        # Convert to local frame if we have home
        if self._home is not None:
            self._target_local = self._gps_to_local(msg.latitude, msg.longitude)

    def _gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert GPS to local ENU coordinates."""
        if self._home is None:
            return (0.0, 0.0)
        
        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude
        
        meters_per_deg = 111111.0
        lat_rad = math.radians(lat0)
        
        x = (lon - lon0) * meters_per_deg * math.cos(lat_rad)
        y = (lat - lat0) * meters_per_deg
        
        return (x, y)

    def _local_to_gps(self, x: float, y: float) -> Tuple[float, float]:
        """Convert local ENU to GPS coordinates."""
        if self._home is None:
            return (0.0, 0.0)
        
        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude
        
        meters_per_deg = 111111.0
        lat_rad = math.radians(lat0)
        
        dlat = y / meters_per_deg
        dlon = x / (meters_per_deg * math.cos(lat_rad))
        
        return (lat0 + dlat, lon0 + dlon)

    def _calculate_distance(self) -> Optional[float]:
        """Calculate 3D distance from drone to target."""
        if self._drone_local is None or self._target_local is None:
            return None
        
        dx = self._drone_local.pose.position.x - self._target_local[0]
        dy = self._drone_local.pose.position.y - self._target_local[1]
        dz = self._drone_local.pose.position.z  # Target at ground level
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _get_noise_std(self, distance: float) -> float:
        """Calculate noise standard deviation based on distance."""
        max_noise = self.get_parameter("max_noise_m").get_parameter_value().double_value
        min_noise = self.get_parameter("min_noise_m").get_parameter_value().double_value
        falloff = self.get_parameter("noise_falloff_dist_m").get_parameter_value().double_value
        
        # Linear interpolation: close = min noise, far = max noise
        t = min(distance / max(falloff, 1.0), 1.0)
        return min_noise + t * (max_noise - min_noise)

    def _on_timer(self) -> None:
        if self._target_gps is None or self._drone_gps is None:
            return
        if self._drone_local is None or self._target_local is None:
            return

        # Check detection range
        distance = self._calculate_distance()
        if distance is None:
            return
        
        max_range = self.get_parameter("detection_range_m").get_parameter_value().double_value
        if distance > max_range:
            return  # Target out of range

        # Only detect if drone is above 5m (can see ground)
        if self._drone_local.pose.position.z < 5.0:
            return

        # Calculate noise based on distance
        noise_std = self._get_noise_std(distance)
        
        # Add noise in local frame, then convert to GPS
        noisy_x = self._target_local[0] + random.gauss(0, noise_std)
        noisy_y = self._target_local[1] + random.gauss(0, noise_std)
        
        noisy_lat, noisy_lon = self._local_to_gps(noisy_x, noisy_y)
        
        self._observation_count += 1

        # Publish noisy GPS
        noisy_msg = NavSatFix()
        noisy_msg.header.stamp = self.get_clock().now().to_msg()
        noisy_msg.header.frame_id = "gps"
        noisy_msg.latitude = noisy_lat
        noisy_msg.longitude = noisy_lon
        noisy_msg.altitude = self._target_gps.altitude
        noisy_msg.status.status = noisy_msg.status.STATUS_FIX
        self._noisy_gps_pub.publish(noisy_msg)
        
        # Also publish as noisy waypoint for navigation
        self._noisy_waypoint_pub.publish(noisy_msg)

        # Publish JSON estimates for visualizer
        estimates = {
            "timestamp": self.get_clock().now().nanoseconds,
            "targets": {
                self._target_id: {
                    "num_observations": self._observation_count,
                    "gps": {
                        "latitude": noisy_lat,
                        "longitude": noisy_lon,
                    },
                    "true_gps": {
                        "latitude": self._target_gps.latitude,
                        "longitude": self._target_gps.longitude,
                    },
                    "distance_m": distance,
                    "noise_std_m": noise_std,
                }
            }
        }
        
        json_msg = String()
        json_msg.data = json.dumps(estimates)
        self._estimates_pub.publish(json_msg)


def main() -> None:
    rclpy.init()
    node = TargetTrackerSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
