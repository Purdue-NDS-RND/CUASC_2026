"""
URDF-Powered Target Localizer Node

ARCHITECTURE OVERVIEW:
This node is the crucial bridge between 2D Computer Vision and 3D Flight Navigation.
It performs the following pipeline:
  1. Ingestion: Receives 2D pixel coordinates (u, v) from the YOLO neural network.
  2. Raycasting: Uses the Arducam's intrinsic matrix (via image_geometry) to push
     a 3D vector out of the camera lens.
  3. Kinematics (URDF): Asks the ROS 2 TF2 tree exactly how the camera is mounted
     to the drone, and rotates the ray accordingly.
  4. Ground Intersection: Calculates where that 3D ray hits the physical ground.
  5. Temporal Filtering: Stores the last 50 ground hits in a memory buffer to filter
     out YOLO flickering, false positives, and camera vibration.
  6. Geolocation: Projects the smoothed local coordinate onto the curvature of the Earth
     to generate a final Latitude/Longitude GPS waypoint for the flight controller.
"""

import json
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import image_geometry
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from mavros_msgs.msg import HomePosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import CameraInfo, NavSatFix
from std_msgs.msg import ColorRGBA, String
from vision_msgs.msg import Detection2D
from visualization_msgs.msg import Marker, MarkerArray

# RViz UI Colors: Used to visually separate different targets on the map
TARGET_COLORS = [
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
    (1.0, 1.0, 0.0),
    (1.0, 0.0, 1.0),
    (0.0, 1.0, 1.0),
    (1.0, 0.5, 0.0),
    (0.5, 0.0, 1.0),
]


@dataclass
class Observation:
    """Represents a single, raw ground-hit calculated from one YOLO frame."""

    x: float
    y: float
    z: float
    confidence: float
    time_ns: int


@dataclass
class TargetState:
    """Holds the historical memory buffer for a specific YOLO class ID."""

    target_id: str
    observations: deque = field(default_factory=lambda: deque(maxlen=50))
    estimate: Optional[Tuple[float, float, float]] = None
    estimate_gps: Optional[Tuple[float, float, float]] = None
    color_idx: int = 0


class TargetLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("target_localizer")

        # Constant: Earth Radius in meters (WGS84 Equator approximation)
        self.R_EARTH = 6378137.0

        # The mathematical model that uses our arducam_info.yaml file to fix lens distortion
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False

        # TF2 setup: This buffer actively listens to the URDF for camera mount angles
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------------------------------------------------------
        # ROS 2 Parameters (Tunable via YAML without recompiling)
        # ---------------------------------------------------------
        self.declare_parameter("ground_altitude_m", 0.0)  # Assumes target field is flat
        self.declare_parameter(
            "observation_buffer_size", 50
        )  # How many frames to remember
        self.declare_parameter(
            "min_observations", 3
        )  # Ignore YOLO if it only flashes for 1-2 frames
        self.declare_parameter(
            "max_observation_age_s", 10.0
        )  # Forget points older than 10 seconds
        self.declare_parameter(
            "outlier_threshold_m", 10.0
        )  # Reject points >10m away from the center mass
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", 10.0)

        # ---------------------------------------------------------
        # State Variables
        # ---------------------------------------------------------
        self._drone_pose: Optional[PoseStamped] = None
        self._home: Optional[HomePosition] = None
        self._targets: Dict[str, TargetState] = {}
        self._next_color_idx = 0

        # ---------------------------------------------------------
        # Publishers (Data Out)
        # ---------------------------------------------------------
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

        # ---------------------------------------------------------
        # Subscribers (Data In)
        # ---------------------------------------------------------
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_camera_info, 10
        )
        self.create_subscription(
            Detection2D, "/drone_control/detection", self._on_detection, 10
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            HomePosition, "/mavros/home_position/home", self._on_home, 10
        )

        # Main processing loop
        rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        self.get_logger().info(
            "URDF Localizer Initialized. Waiting for Intrinsics & Telemetry..."
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Loads the radial distortion matrices (K and D) from the camera driver."""
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info("✅ Intrinsic Matrices Loaded successfully!")

    def _on_pose(self, msg: PoseStamped) -> None:
        """Live drone position and orientation (Pitch/Roll/Yaw) from MAVROS."""
        self._drone_pose = msg

    def _on_home(self, msg: HomePosition) -> None:
        """The GPS coordinates of where the drone took off. Used as the (0,0,0) origin."""
        self._home = msg

    def _on_detection(self, msg: Detection2D) -> None:
        """
        Triggered every time YOLO finds an object.
        This is the main entry point for the raycasting math.
        """
        # Safety Check: Can't do math without the matrices and drone location
        if not self.camera_info_received or self._drone_pose is None:
            return
        if not msg.results:
            return

        # Extract YOLO Semantic Data
        target_id = (
            msg.results[0].hypothesis.class_id
            if msg.results[0].hypothesis.class_id
            else "target"
        )
        confidence = (
            msg.results[0].hypothesis.score
            if msg.results[0].hypothesis.score > 0
            else 1.0
        )

        # Extract YOLO Geometric Data (Center of bounding box)
        u = msg.bbox.center.position.x
        v = msg.bbox.center.position.y

        # Perform the 3D Projection Math
        local_pos = self._pixel_to_local(u, v)
        if local_pos is None:
            return

        x, y, z = local_pos
        now_ns = self.get_clock().now().nanoseconds

        # If this is a new target ID (e.g., "tractor"), create a new memory buffer for it
        if target_id not in self._targets:
            self._targets[target_id] = TargetState(
                target_id=target_id,
                observations=deque(
                    maxlen=self.get_parameter("observation_buffer_size")
                    .get_parameter_value()
                    .integer_value
                ),
                color_idx=self._next_color_idx,
            )
            self._next_color_idx = (self._next_color_idx + 1) % len(TARGET_COLORS)
            self.get_logger().info(f"Tracking new target: '{target_id}'")

        # Save this single raycast hit to the buffer for filtering later
        obs = Observation(x=x, y=y, z=z, confidence=confidence, time_ns=now_ns)
        self._targets[target_id].observations.append(obs)

    def _pixel_to_local(
        self, u: float, v: float
    ) -> Optional[Tuple[float, float, float]]:
        """
        THE CORE MATHEMATICAL PIPELINE:
        Converts a 2D pixel to a 3D intersection point on the physical ground.
        """
        # ---------------------------------------------------------
        # STEP 1: Intrinsic Projection (Un-flattening the image)
        # ---------------------------------------------------------
        # PinholeCameraModel uses the YAML file to generate a 3D directional ray.
        # Note: In ROS optical frames, Z is pointing straight forward out of the lens.
        ray_opt = self.camera_model.projectPixelTo3dRay((u, v))

        try:
            # ---------------------------------------------------------
            # STEP 2: The URDF Lookup (Where is the camera mounted?)
            # ---------------------------------------------------------
            # We ask TF2: "What is the physical relationship between the drone's
            # center (base_link) and the camera lens (camera_optical_frame) RIGHT NOW?"
            t_mount = self.tf_buffer.lookup_transform(
                "base_link", "camera_optical_frame", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Waiting for URDF transform: {e}")
            return None

        # Convert the TF2 Quaternion into a professional SciPy Rotation matrix
        q_m = t_mount.transform.rotation
        r_mount = R_scipy.from_quat([q_m.x, q_m.y, q_m.z, q_m.w])

        # Extract how many meters the camera sits away from the drone's center
        cam_offset = (
            t_mount.transform.translation.x,
            t_mount.transform.translation.y,
            t_mount.transform.translation.z,
        )

        # Apply the mount rotation: The ray is now correctly angled relative to the drone's body
        ray_body = r_mount.apply(ray_opt)

        # ---------------------------------------------------------
        # STEP 3: The Flight Controller Lookup (How is the drone flying?)
        # ---------------------------------------------------------
        # If the drone is banking hard left, we must twist our ray hard left.
        q_d = self._drone_pose.pose.orientation
        r_drone = R_scipy.from_quat([q_d.x, q_d.y, q_d.z, q_d.w])

        # Apply the drone's flight rotation: The ray is now aligned with the Earth (ENU frame)
        world_ray = r_drone.apply(ray_body)

        # ---------------------------------------------------------
        # STEP 4: Ground Intersection (Raycasting)
        # ---------------------------------------------------------
        drone_pos = self._drone_pose.pose.position

        # Calculate exactly where the physical lens is floating in 3D world space
        cam_world_offset = r_drone.apply(cam_offset)
        cam_x = drone_pos.x + cam_world_offset[0]
        cam_y = drone_pos.y + cam_world_offset[1]
        cam_z = drone_pos.z + cam_world_offset[2]

        ground_z = (
            self.get_parameter("ground_altitude_m").get_parameter_value().double_value
        )

        # Prevent divide-by-zero if the camera is pointing perfectly horizontal
        if abs(world_ray[2]) < 1e-6:
            return None

        # The Scalar 't': How many times do we have to multiply the ray's Z-component
        # until it reaches the ground?
        t = (ground_z - cam_z) / world_ray[2]

        # If t is negative, the ray is shooting up into the sky. (Drone is probably upside down).
        if t < 0:
            return None

        # Scale the X and Y components of the ray by 't' to find the exact target location
        target_x = cam_x + t * world_ray[0]
        target_y = cam_y + t * world_ray[1]

        return (target_x, target_y, ground_z)

    def _on_timer(self) -> None:
        """Runs at 10Hz to clean up memory and publish the final smoothed GPS coords."""
        self._prune_old_observations()

        for target_id, state in self._targets.items():
            estimate = self._compute_filtered_estimate(state)
            state.estimate = estimate
            if estimate is not None:
                # Convert the smoothed XYZ to Latitude/Longitude
                state.estimate_gps = self._local_to_gps_tuple(*estimate)

        self._publish_estimates()
        self._publish_observations()
        self._publish_markers()

    def _prune_old_observations(self) -> None:
        """Forgets targets if the drone flies away and hasn't seen them in 10 seconds."""
        max_age_ns = int(
            self.get_parameter("max_observation_age_s")
            .get_parameter_value()
            .double_value
            * 1e9
        )
        now_ns = self.get_clock().now().nanoseconds

        for state in self._targets.values():
            while state.observations and (
                now_ns - state.observations[0].time_ns > max_age_ns
            ):
                state.observations.popleft()

    def _compute_filtered_estimate(
        self, state: TargetState
    ) -> Optional[Tuple[float, float, float]]:
        """
        THE OUTLIER REJECTION FILTER:
        Takes the buffer of 50 ground-hits, throws away the anomalies, and returns the center of mass.
        """
        min_obs = (
            self.get_parameter("min_observations").get_parameter_value().integer_value
        )
        if len(state.observations) < min_obs:
            return None

        # Pass 1: Find the raw geometric center of all 50 points
        sum_x = sum(o.x for o in state.observations)
        sum_y = sum(o.y for o in state.observations)
        count = len(state.observations)
        mean_x, mean_y = sum_x / count, sum_y / count

        # Pass 2: Weighted Mean & Anomaly Pruning
        outlier_thresh = (
            self.get_parameter("outlier_threshold_m").get_parameter_value().double_value
        )
        sum_x, sum_y, sum_z, sum_w = 0.0, 0.0, 0.0, 0.0

        for obs in state.observations:
            # If a point is wildly far away from the center (e.g. YOLO glitched on a rock), ignore it.
            dist = math.hypot(obs.x - mean_x, obs.y - mean_y)
            if dist > outlier_thresh:
                continue

            # Weight the points based on YOLO's confidence score
            sum_x += obs.confidence * obs.x
            sum_y += obs.confidence * obs.y
            sum_z += obs.confidence * obs.z
            sum_w += obs.confidence

        if sum_w < 1e-6:
            return None

        return (sum_x / sum_w, sum_y / sum_w, sum_z / sum_w)

    def _local_to_gps_tuple(
        self, x: float, y: float, z: float
    ) -> Optional[Tuple[float, float, float]]:
        """
        SPHERICAL EARTH GEOLOCATION:
        Wraps the flat Cartesian (X,Y) coordinates around the curvature of the Earth.
        """
        if self._home is None:
            return None

        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude
        alt0 = self._home.geo.altitude

        # X is East, Y is North in ENU Frame
        # 1 degree of Latitude is roughly constant everywhere
        lat_offset = (y / self.R_EARTH) * (180.0 / math.pi)

        # 1 degree of Longitude shrinks as you move away from the equator toward the poles.
        # We multiply by cos(latitude) to scale the distance correctly for the Mojave desert.
        lon_scale = math.cos(math.radians(lat0))
        lon_offset = (x / (self.R_EARTH * lon_scale)) * (180.0 / math.pi)

        return (lat0 + lat_offset, lon0 + lon_offset, alt0 + z)

    # ---------------------------------------------------------
    # Publishing Methods (Formatting data for RViz and Flight Controller)
    # ---------------------------------------------------------
    def _publish_estimates(self) -> None:
        """Publishes the final, smoothed GPS waypoints as a JSON string for the UI."""
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        now = self.get_clock().now().to_msg()

        pose_array = PoseArray()
        pose_array.header.stamp = now
        pose_array.header.frame_id = frame_id

        gps_data = {"timestamp": now.sec + now.nanosec * 1e-9, "targets": {}}

        for target_id, state in self._targets.items():
            if state.estimate is None:
                continue

            x, y, z = state.estimate
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            target_data = {
                "local": {"x": x, "y": y, "z": z},
                "num_observations": len(state.observations),
            }
            if state.estimate_gps is not None:
                lat, lon, alt = state.estimate_gps
                target_data["gps"] = {
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": alt,
                }

            gps_data["targets"][target_id] = target_data

        self._estimates_pub.publish(pose_array)
        gps_msg = String()
        gps_msg.data = json.dumps(gps_data, indent=2)
        self._estimates_gps_pub.publish(gps_msg)

    def _publish_observations(self) -> None:
        """Publishes the raw, unfiltered point cloud of every YOLO hit."""
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        now = self.get_clock().now().to_msg()

        pose_array = PoseArray()
        pose_array.header.stamp = now
        pose_array.header.frame_id = frame_id

        for state in self._targets.values():
            for obs in state.observations:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = obs.x, obs.y, obs.z
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)

        self._observations_pub.publish(pose_array)

    def _publish_markers(self) -> None:
        """Draws the transparent dots (history) and solid spheres (center) in RViz 3D."""
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        marker_id = 0

        for target_id, state in self._targets.items():
            color = TARGET_COLORS[state.color_idx]

            # Draw the raw 50-point history buffer as tiny transparent dots
            if state.observations:
                obs_marker = Marker()
                obs_marker.header.stamp, obs_marker.header.frame_id = now, frame_id
                obs_marker.ns, obs_marker.id = f"obs_{target_id}", marker_id
                marker_id += 1
                obs_marker.type, obs_marker.action = Marker.POINTS, Marker.ADD
                obs_marker.scale.x = obs_marker.scale.y = 0.3
                obs_marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.5)
                for obs in state.observations:
                    obs_marker.points.append(Point(x=obs.x, y=obs.y, z=obs.z))
                marker_array.markers.append(obs_marker)

            # Draw the final smoothed target estimate as a solid 1-meter sphere
            if state.estimate is not None:
                x, y, z = state.estimate
                est_marker = Marker()
                est_marker.header.stamp, est_marker.header.frame_id = now, frame_id
                est_marker.ns, est_marker.id = f"est_{target_id}", marker_id
                marker_id += 1
                est_marker.type, est_marker.action = Marker.SPHERE, Marker.ADD
                (
                    est_marker.pose.position.x,
                    est_marker.pose.position.y,
                    est_marker.pose.position.z,
                ) = x, y, z
                est_marker.pose.orientation.w = 1.0
                est_marker.scale.x = est_marker.scale.y = est_marker.scale.z = 1.0
                est_marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
                marker_array.markers.append(est_marker)

        self._markers_pub.publish(marker_array)


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
