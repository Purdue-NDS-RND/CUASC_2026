"""
Mission Logger Node for Post-Flight Review

Pipeline:
1. Caches raw images from the camera.
2. Receives YOLO Detection2D messages.
3. Uses URDF/TF2 to raycast the detection to a global GPS coordinate.
4. Checks if this GPS coordinate is within 10 meters of an already-saved target.
5. If it is a NEW target, it pulls the matching image, saves it to disk, and logs to a CSV.
"""

import csv
import math
import os
from datetime import datetime
from typing import List, Optional, Tuple

import cv2
import image_geometry
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from vision_msgs.msg import Detection2D


class MissionLogger(Node):
    def __init__(self) -> None:
        super().__init__("mission_logger")

        # ---------------------------------------------------------
        # Mission Logging Setup
        # ---------------------------------------------------------
        # Create a timestamped folder in the user's home directory for this specific flight
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.expanduser(
            f"~/CUASC_Mission_Data/Flight_{timestamp_str}"
        )
        os.makedirs(self.save_dir, exist_ok=True)

        self.csv_path = os.path.join(self.save_dir, "mission_log.csv")
        self._init_csv()

        self.image_counter = 1
        self.saved_target_locations: List[
            Tuple[float, float]
        ] = []  # Stores (Lat, Lon) of saved targets
        self.min_dist_m = 10.0  # Don't save a new image if a target is within 10 meters

        # Image Caching (To match Detections to their exact Image frame)
        self.cv_bridge = CvBridge()
        self.image_cache = {}  # Dict of {timestamp_nanoseconds: cv2_image}
        self.cache_size = 30  # Keep the last 30 frames in RAM

        # ---------------------------------------------------------
        # Math & TF2 Setup
        # ---------------------------------------------------------
        self.R_EARTH = 6378137.0
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.declare_parameter("ground_altitude_m", 0.0)

        self._drone_pose: Optional[PoseStamped] = None
        self._home: Optional[HomePosition] = None

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_camera_info, 10
        )
        self.create_subscription(Image, "/camera/image_raw", self._on_image, 10)
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

        self.get_logger().info(
            f"🚀 Mission Logger Ready. Saving data to: {self.save_dir}"
        )

    def _init_csv(self):
        """Creates the CSV file and writes the header row."""
        with open(self.csv_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                ["Image_Name", "Latitude", "Longitude", "Time_UTC", "YOLO_Confidence"]
            )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True

    def _on_pose(self, msg: PoseStamped) -> None:
        self._drone_pose = msg

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    def _on_image(self, msg: Image) -> None:
        """Continuously caches the latest images from the camera."""
        timestamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        self.image_cache[timestamp_ns] = cv_image

        # Prune old images to prevent RAM explosion
        if len(self.image_cache) > self.cache_size:
            oldest_key = min(self.image_cache.keys())
            del self.image_cache[oldest_key]

    def _on_detection(self, msg: Detection2D) -> None:
        """Triggers when YOLO finds a target."""
        if (
            not self.camera_info_received
            or self._drone_pose is None
            or self._home is None
        ):
            return
        if not msg.results:
            return

        # 1. Calculate Ground Coordinates using our URDF Math
        u = msg.bbox.center.position.x
        v = msg.bbox.center.position.y
        confidence = msg.results[0].hypothesis.score

        gps_coord = self._raycast_to_gps(u, v)
        if gps_coord is None:
            return

        target_lat, target_lon = gps_coord

        # 2. Spatial Throttling: Is this a duplicate of a target we just saved?
        for saved_lat, saved_lon in self.saved_target_locations:
            dist = self._calculate_distance_m(
                target_lat, target_lon, saved_lat, saved_lon
            )
            if dist < self.min_dist_m:
                return  # We already logged this target area. Ignore it.

        # 3. IT IS A NEW TARGET! Log it.
        self._log_target(msg, target_lat, target_lon, confidence, u, v)

    def _log_target(
        self,
        det_msg: Detection2D,
        lat: float,
        lon: float,
        conf: float,
        u: float,
        v: float,
    ):
        """Grabs the image, saves it, and writes the CSV row."""
        timestamp_ns = det_msg.header.stamp.sec * 1e9 + det_msg.header.stamp.nanosec

        # Look for the exact image frame that generated this detection
        if timestamp_ns not in self.image_cache:
            self.get_logger().warn(
                "Could not find matching image in cache! Skipping log."
            )
            return

        # 1. Prepare the Image
        frame = self.image_cache[timestamp_ns].copy()

        # Draw a circle on the target so the human reviewer can find it instantly
        cv2.circle(frame, (int(u), int(v)), 50, (0, 0, 255), 4)  # Red Circle
        text = f"Target {self.image_counter} | Lat: {lat:.6f}, Lon: {lon:.6f}"
        cv2.putText(
            frame, text, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 4
        )

        image_filename = f"target_{self.image_counter:03d}.jpg"
        image_path = os.path.join(self.save_dir, image_filename)

        # 2. Save Image to Disk
        cv2.imwrite(image_path, frame)

        # 3. Write to CSV
        time_utc = datetime.utcnow().strftime("%H:%M:%S")
        with open(self.csv_path, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [image_filename, f"{lat:.7f}", f"{lon:.7f}", time_utc, f"{conf:.2f}"]
            )

        self.get_logger().info(
            f"✅ LOGGED: {image_filename} -> Lat: {lat:.7f}, Lon: {lon:.7f}"
        )

        # 4. Update Memory
        self.saved_target_locations.append((lat, lon))
        self.image_counter += 1

    def _raycast_to_gps(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """Combined Raycast and Geolocator pipeline using URDF/TF2."""
        # 1. Optical Ray
        ray_opt = self.camera_model.projectPixelTo3dRay((u, v))

        # 2. URDF Camera Mount Lookup
        try:
            t_mount = self.tf_buffer.lookup_transform(
                "base_link", "camera_optical_frame", rclpy.time.Time()
            )
        except Exception:
            return None

        q_m = t_mount.transform.rotation
        r_mount = R_scipy.from_quat([q_m.x, q_m.y, q_m.z, q_m.w])
        cam_offset = (
            t_mount.transform.translation.x,
            t_mount.transform.translation.y,
            t_mount.transform.translation.z,
        )

        # 3. Drone Flight Orientation Lookup
        q_d = self._drone_pose.pose.orientation
        r_drone = R_scipy.from_quat([q_d.x, q_d.y, q_d.z, q_d.w])

        # 4. Rotate Ray to Earth Frame
        world_ray = r_drone.apply(r_mount.apply(ray_opt))

        # 5. Ground Intersection
        drone_pos = self._drone_pose.pose.position
        cam_world_offset = r_drone.apply(cam_offset)
        cam_z = drone_pos.z + cam_world_offset[2]
        ground_z = (
            self.get_parameter("ground_altitude_m").get_parameter_value().double_value
        )

        if abs(world_ray[2]) < 1e-6:
            return None
        t = (ground_z - cam_z) / world_ray[2]
        if t < 0:
            return None

        target_x = (drone_pos.x + cam_world_offset[0]) + t * world_ray[0]
        target_y = (drone_pos.y + cam_world_offset[1]) + t * world_ray[1]

        # 6. Geolocation (Spherical Projection)
        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude

        lat_offset = (target_y / self.R_EARTH) * (180.0 / math.pi)
        lon_scale = math.cos(math.radians(lat0))
        lon_offset = (target_x / (self.R_EARTH * lon_scale)) * (180.0 / math.pi)

        return (lat0 + lat_offset, lon0 + lon_offset)

    def _calculate_distance_m(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        """Calculates distance in meters between two GPS coordinates to prevent duplicate logs."""
        dy = (lat2 - lat1) * self.R_EARTH * (math.pi / 180.0)
        dx = (
            (lon2 - lon1)
            * self.R_EARTH
            * math.cos(math.radians(lat1))
            * (math.pi / 180.0)
        )
        return math.hypot(dx, dy)


def main() -> None:
    rclpy.init()
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
