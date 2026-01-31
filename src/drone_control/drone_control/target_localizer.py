"""
Multi-Target Localizer Node

Takes camera detections (pixel coordinates + target ID) + drone pose and estimates
each target's GPS coordinates using geometric projection.

Pipeline:
  1. Receive detection (target_id, pixel u, v, confidence) from CV node
  2. Project pixel to 3D ray using camera intrinsics
  3. Rotate ray to world frame using drone orientation
  4. Intersect ray with ground plane (z = ground_altitude)
  5. Add to per-target observation buffer, compute filtered estimate
  6. Convert local XYZ to GPS using home position

Detection Message Format (vision_msgs/Detection2D):
  - header: timestamp and frame_id
  - bbox.center.position.x: pixel u (center column)
  - bbox.center.position.y: pixel v (center row)
  - bbox.size_x: bounding box width (pixels)
  - bbox.size_y: bounding box height (pixels)
  - results[0].hypothesis.class_id: target ID string
  - results[0].hypothesis.score: confidence (0-1)

Publications:
  - /drone_control/targets/estimates (geometry_msgs/PoseArray) - all target estimates
  - /drone_control/targets/estimates_gps (std_msgs/String) - JSON with all GPS estimates
  - /drone_control/targets/observations (geometry_msgs/PoseArray) - raw observations (all targets)
  - /drone_control/targets/markers (visualization_msgs/MarkerArray) - RViz visualization

For visualization, observations are color-coded per target.
"""

import json
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, ColorRGBA
from vision_msgs.msg import Detection2D
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_rotation_matrix(q) -> List[List[float]]:
    """Convert quaternion (x, y, z, w) to 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return [
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]


def rotate_vector(R: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Apply rotation matrix R to vector v."""
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )


# Distinct colors for different targets (up to 10, then cycles)
TARGET_COLORS = [
    (1.0, 0.0, 0.0),  # Red
    (0.0, 1.0, 0.0),  # Green
    (0.0, 0.0, 1.0),  # Blue
    (1.0, 1.0, 0.0),  # Yellow
    (1.0, 0.0, 1.0),  # Magenta
    (0.0, 1.0, 1.0),  # Cyan
    (1.0, 0.5, 0.0),  # Orange
    (0.5, 0.0, 1.0),  # Purple
    (0.0, 0.5, 0.0),  # Dark green
    (0.5, 0.5, 0.5),  # Gray
]


@dataclass
class Observation:
    x: float
    y: float
    z: float
    confidence: float
    time_ns: int
    bbox_width_px: float = 0.0
    bbox_height_px: float = 0.0


@dataclass
class TargetState:
    """State for a single tracked target."""
    target_id: str
    observations: deque = field(default_factory=lambda: deque(maxlen=50))
    estimate: Optional[Tuple[float, float, float]] = None
    estimate_gps: Optional[Tuple[float, float, float]] = None  # lat, lon, alt
    color_idx: int = 0


class TargetLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("target_localizer")

        # Camera intrinsics for Arducam IMX519 + 120° M12 lens
        # Default assumes 1280x720 streaming resolution
        self.declare_parameter("camera_fx", 424.0)  # Focal length x (pixels)
        self.declare_parameter("camera_fy", 424.0)  # Focal length y (pixels)
        self.declare_parameter("camera_cx", 640.0)  # Principal point x (pixels)
        self.declare_parameter("camera_cy", 360.0)  # Principal point y (pixels)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)

        # Camera mounting (relative to drone body frame)
        # Assumes camera points down (-Z in body frame)
        # pitch_offset: 0 = straight down, positive = tilted forward
        self.declare_parameter("camera_pitch_offset_deg", 0.0)
        self.declare_parameter("camera_x_offset_m", 0.0)  # Forward from CoG
        self.declare_parameter("camera_y_offset_m", 0.0)  # Right from CoG
        self.declare_parameter("camera_z_offset_m", 0.0)  # Down from CoG

        # Ground plane altitude (local frame)
        self.declare_parameter("ground_altitude_m", 0.0)

        # Filtering parameters
        self.declare_parameter("observation_buffer_size", 50)
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("max_observation_age_s", 30.0)
        self.declare_parameter("outlier_threshold_m", 10.0)

        # Output
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 10.0)

        # State
        self._drone_pose: Optional[PoseStamped] = None
        self._home: Optional[HomePosition] = None
        self._targets: Dict[str, TargetState] = {}
        self._next_color_idx = 0
        self._obs_buffer_size = (
            self.get_parameter("observation_buffer_size").get_parameter_value().integer_value
        )

        # Publishers
        self._estimates_pub = self.create_publisher(
            PoseArray, "/drone_control/targets/estimates", 10
        )
        self._estimates_gps_pub = self.create_publisher(
            String, "/drone_control/targets/estimates_gps", 10
        )
        self._observations_pub = self.create_publisher(
            PoseArray, "/drone_control/targets/observations", 10
        )
        self._markers_pub = self.create_publisher(
            MarkerArray, "/drone_control/targets/markers", 10
        )
        self._status_pub = self.create_publisher(
            String, "/drone_control/localizer/status", 10
        )

        # Subscribers
        self.create_subscription(
            Detection2D,
            "/drone_control/detection",
            self._on_detection,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            HomePosition,
            "/mavros/home_position/home",
            self._on_home,
            10,
        )

        # Timer
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        self.get_logger().info("Multi-target localizer ready")

    def _on_pose(self, msg: PoseStamped) -> None:
        self._drone_pose = msg

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    def _on_detection(self, msg: Detection2D) -> None:
        """Process a detection from vision_msgs/Detection2D."""
        if self._drone_pose is None:
            self.get_logger().warn("No drone pose yet, ignoring detection", throttle_duration_sec=5.0)
            return

        # Parse Detection2D message
        if not msg.results:
            self.get_logger().warn("Detection has no results", throttle_duration_sec=5.0)
            return
        
        target_id = msg.results[0].hypothesis.class_id if msg.results[0].hypothesis.class_id else "default"
        confidence = msg.results[0].hypothesis.score if msg.results[0].hypothesis.score > 0 else 1.0
        
        # Bounding box center and size
        u = msg.bbox.center.position.x
        v = msg.bbox.center.position.y
        bbox_width = msg.bbox.size_x
        bbox_height = msg.bbox.size_y

        # Project to local coordinates
        local_pos = self._pixel_to_local(u, v)
        if local_pos is None:
            return

        x, y, z = local_pos
        now_ns = self.get_clock().now().nanoseconds

        # Get or create target state
        if target_id not in self._targets:
            self._targets[target_id] = TargetState(
                target_id=target_id,
                observations=deque(maxlen=self._obs_buffer_size),
                color_idx=self._next_color_idx,
            )
            self._next_color_idx = (self._next_color_idx + 1) % len(TARGET_COLORS)
            self.get_logger().info(f"New target detected: '{target_id}'")

        # Add observation
        obs = Observation(
            x=x, y=y, z=z,
            confidence=confidence,
            time_ns=now_ns,
            bbox_width_px=bbox_width,
            bbox_height_px=bbox_height,
        )
        self._targets[target_id].observations.append(obs)

    def _pixel_to_local(self, u: float, v: float) -> Optional[Tuple[float, float, float]]:
        """Project pixel coordinates to local world frame."""
        if self._drone_pose is None:
            return None

        # Get camera intrinsics
        fx = self.get_parameter("camera_fx").get_parameter_value().double_value
        fy = self.get_parameter("camera_fy").get_parameter_value().double_value
        cx = self.get_parameter("camera_cx").get_parameter_value().double_value
        cy = self.get_parameter("camera_cy").get_parameter_value().double_value

        # Pixel to normalized camera ray
        ray_x = (u - cx) / fx
        ray_y = (v - cy) / fy
        ray_z = 1.0

        # Normalize
        ray_len = math.sqrt(ray_x*ray_x + ray_y*ray_y + ray_z*ray_z)
        ray_x /= ray_len
        ray_y /= ray_len
        ray_z /= ray_len

        # Apply camera pitch offset
        pitch_offset = math.radians(
            self.get_parameter("camera_pitch_offset_deg").get_parameter_value().double_value
        )
        cos_p, sin_p = math.cos(pitch_offset), math.sin(pitch_offset)

        # Camera Z points into image; for downward camera, this maps to -Z in body
        # Rotate around body X axis by pitch_offset
        body_ray_x = ray_x
        body_ray_y = cos_p * ray_z - sin_p * 1.0  # ray_z forward, 1.0 down
        body_ray_z = sin_p * ray_z + cos_p * (-1.0)  # -1.0 = down in body frame

        # Normalize again
        ray_len = math.sqrt(body_ray_x**2 + body_ray_y**2 + body_ray_z**2)
        if ray_len < 1e-6:
            return None
        body_ray = (body_ray_x / ray_len, body_ray_y / ray_len, body_ray_z / ray_len)

        # Rotate to world frame
        q = self._drone_pose.pose.orientation
        R = quaternion_to_rotation_matrix(q)
        world_ray = rotate_vector(R, body_ray)

        # Camera position in world
        drone_x = self._drone_pose.pose.position.x
        drone_y = self._drone_pose.pose.position.y
        drone_z = self._drone_pose.pose.position.z

        cam_offset_body = (
            self.get_parameter("camera_x_offset_m").get_parameter_value().double_value,
            self.get_parameter("camera_y_offset_m").get_parameter_value().double_value,
            self.get_parameter("camera_z_offset_m").get_parameter_value().double_value,
        )
        cam_offset_world = rotate_vector(R, cam_offset_body)
        cam_x = drone_x + cam_offset_world[0]
        cam_y = drone_y + cam_offset_world[1]
        cam_z = drone_z + cam_offset_world[2]

        ground_z = self.get_parameter("ground_altitude_m").get_parameter_value().double_value

        # Intersect ray with ground plane
        if abs(world_ray[2]) < 1e-6:
            return None  # Ray horizontal

        t = (ground_z - cam_z) / world_ray[2]
        if t < 0:
            return None  # Behind camera

        target_x = cam_x + t * world_ray[0]
        target_y = cam_y + t * world_ray[1]
        target_z = ground_z

        return (target_x, target_y, target_z)

    def _on_timer(self) -> None:
        """Compute estimates and publish everything."""
        self._prune_old_observations()

        for target_id, state in self._targets.items():
            estimate = self._compute_filtered_estimate(state)
            state.estimate = estimate
            if estimate is not None:
                state.estimate_gps = self._local_to_gps_tuple(*estimate)

        self._publish_estimates()
        self._publish_observations()
        self._publish_markers()

    def _prune_old_observations(self) -> None:
        """Remove old observations from all targets."""
        max_age_ns = int(
            self.get_parameter("max_observation_age_s").get_parameter_value().double_value * 1e9
        )
        now_ns = self.get_clock().now().nanoseconds

        for state in self._targets.values():
            while state.observations:
                if now_ns - state.observations[0].time_ns > max_age_ns:
                    state.observations.popleft()
                else:
                    break

    def _compute_filtered_estimate(self, state: TargetState) -> Optional[Tuple[float, float, float]]:
        """Compute weighted mean with outlier rejection."""
        min_obs = self.get_parameter("min_observations").get_parameter_value().integer_value
        if len(state.observations) < min_obs:
            return None

        # First pass: unweighted mean
        sum_x, sum_y, sum_z = 0.0, 0.0, 0.0
        for obs in state.observations:
            sum_x += obs.x
            sum_y += obs.y
            sum_z += obs.z
        count = len(state.observations)
        mean_x, mean_y, mean_z = sum_x / count, sum_y / count, sum_z / count

        # Second pass: weighted mean with outlier rejection
        outlier_thresh = self.get_parameter("outlier_threshold_m").get_parameter_value().double_value
        sum_x, sum_y, sum_z, sum_w = 0.0, 0.0, 0.0, 0.0

        for obs in state.observations:
            dx = obs.x - mean_x
            dy = obs.y - mean_y
            dz = obs.z - mean_z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if dist > outlier_thresh:
                continue

            w = obs.confidence
            sum_x += w * obs.x
            sum_y += w * obs.y
            sum_z += w * obs.z
            sum_w += w

        if sum_w < 1e-6:
            return None

        return (sum_x / sum_w, sum_y / sum_w, sum_z / sum_w)

    def _local_to_gps_tuple(self, x: float, y: float, z: float) -> Optional[Tuple[float, float, float]]:
        """Convert local ENU to GPS (lat, lon, alt)."""
        if self._home is None:
            return None

        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude
        alt0 = self._home.geo.altitude

        lat_rad = math.radians(lat0)
        meters_per_deg = 111111.0

        dlat = y / meters_per_deg
        dlon = x / (meters_per_deg * max(math.cos(lat_rad), 1e-6))

        return (lat0 + dlat, lon0 + dlon, alt0 + z)

    def _publish_estimates(self) -> None:
        """Publish all target estimates as PoseArray and GPS JSON."""
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        now = self.get_clock().now().to_msg()

        # PoseArray for local estimates
        pose_array = PoseArray()
        pose_array.header.stamp = now
        pose_array.header.frame_id = frame_id

        # GPS estimates as JSON
        gps_data = {"timestamp": now.sec + now.nanosec * 1e-9, "targets": {}}

        for target_id, state in self._targets.items():
            if state.estimate is None:
                continue

            x, y, z = state.estimate
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            target_data = {
                "local": {"x": x, "y": y, "z": z},
                "num_observations": len(state.observations),
            }
            if state.estimate_gps is not None:
                lat, lon, alt = state.estimate_gps
                target_data["gps"] = {"latitude": lat, "longitude": lon, "altitude": alt}

            gps_data["targets"][target_id] = target_data

        self._estimates_pub.publish(pose_array)

        gps_msg = String()
        gps_msg.data = json.dumps(gps_data, indent=2)
        self._estimates_gps_pub.publish(gps_msg)

    def _publish_observations(self) -> None:
        """Publish all raw observations as PoseArray."""
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        now = self.get_clock().now().to_msg()

        pose_array = PoseArray()
        pose_array.header.stamp = now
        pose_array.header.frame_id = frame_id

        for state in self._targets.values():
            for obs in state.observations:
                pose = Pose()
                pose.position.x = obs.x
                pose.position.y = obs.y
                pose.position.z = obs.z
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)

        self._observations_pub.publish(pose_array)

    def _publish_markers(self) -> None:
        """Publish visualization markers for RViz."""
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        now = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        marker_id = 0

        for target_id, state in self._targets.items():
            color = TARGET_COLORS[state.color_idx]

            # Observations as point cloud
            if state.observations:
                obs_marker = Marker()
                obs_marker.header.stamp = now
                obs_marker.header.frame_id = frame_id
                obs_marker.ns = f"observations_{target_id}"
                obs_marker.id = marker_id
                marker_id += 1
                obs_marker.type = Marker.POINTS
                obs_marker.action = Marker.ADD
                obs_marker.scale.x = 0.3
                obs_marker.scale.y = 0.3
                obs_marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.5)

                for obs in state.observations:
                    obs_marker.points.append(Point(x=obs.x, y=obs.y, z=obs.z))

                marker_array.markers.append(obs_marker)

            # Estimate as sphere
            if state.estimate is not None:
                x, y, z = state.estimate
                est_marker = Marker()
                est_marker.header.stamp = now
                est_marker.header.frame_id = frame_id
                est_marker.ns = f"estimate_{target_id}"
                est_marker.id = marker_id
                marker_id += 1
                est_marker.type = Marker.SPHERE
                est_marker.action = Marker.ADD
                est_marker.pose.position.x = x
                est_marker.pose.position.y = y
                est_marker.pose.position.z = z
                est_marker.pose.orientation.w = 1.0
                est_marker.scale.x = 1.0
                est_marker.scale.y = 1.0
                est_marker.scale.z = 1.0
                est_marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
                marker_array.markers.append(est_marker)

                # Label with target ID
                label_marker = Marker()
                label_marker.header.stamp = now
                label_marker.header.frame_id = frame_id
                label_marker.ns = f"label_{target_id}"
                label_marker.id = marker_id
                marker_id += 1
                label_marker.type = Marker.TEXT_VIEW_FACING
                label_marker.action = Marker.ADD
                label_marker.pose.position.x = x
                label_marker.pose.position.y = y
                label_marker.pose.position.z = z + 2.0
                label_marker.pose.orientation.w = 1.0
                label_marker.scale.z = 1.5
                label_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                label_marker.text = f"Target {target_id}"
                marker_array.markers.append(label_marker)

        self._markers_pub.publish(marker_array)

    def get_all_estimates(self) -> Dict[str, dict]:
        """Get all current estimates (for external use)."""
        result = {}
        for target_id, state in self._targets.items():
            if state.estimate is None:
                continue
            x, y, z = state.estimate
            entry = {"local": {"x": x, "y": y, "z": z}}
            if state.estimate_gps is not None:
                lat, lon, alt = state.estimate_gps
                entry["gps"] = {"latitude": lat, "longitude": lon, "altitude": alt}
            result[target_id] = entry
        return result

    def clear_target(self, target_id: str) -> None:
        """Clear observations for a specific target."""
        if target_id in self._targets:
            self._targets[target_id].observations.clear()
            self._targets[target_id].estimate = None
            self._targets[target_id].estimate_gps = None

    def clear_all(self) -> None:
        """Clear all targets."""
        self._targets.clear()


def main() -> None:
    rclpy.init()
    node = TargetLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
