"""
Mission Logger Node

Runs two tasks in parallel:

1. CONTINUOUS RECORDER  – saves every single incoming frame as a JPEG
   to  <save_dir>/frames/  so you have a complete flight record.

2. TARGET GEOLOCATOR    – time-synchronises images with YOLO detections,
   raycasts each detection to the ground plane, and writes GPS coordinates
   to mission_log.csv.  Annotated images of new targets are saved separately
   to  <save_dir>/targets/.
"""

import csv
import math
import os
from datetime import datetime
from typing import List, Tuple

import cv2
import image_geometry
import message_filters
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray


class MissionLogger(Node):
    def __init__(self) -> None:
        super().__init__("mission_logger")

        # ------------------------------------------------------------------
        # Output directories
        # ------------------------------------------------------------------
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.expanduser(
            f"~/CUASC_Mission_Data/Flight_{timestamp_str}"
        )

        # All raw frames go here — gives you a complete visual record
        self._frames_dir = os.path.join(self.save_dir, "frames")
        # Annotated images of novel GPS targets go here
        self._targets_dir = os.path.join(self.save_dir, "targets")

        os.makedirs(self._frames_dir, exist_ok=True)
        os.makedirs(self._targets_dir, exist_ok=True)

        self.csv_path = os.path.join(self.save_dir, "mission_log.csv")
        self._init_csv()

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._frame_counter = 0  # increments for every received frame
        self._target_counter = 1  # increments only when a new target is logged
        self.saved_target_locations: List[Tuple[float, float]] = []
        self.min_dist_m = 10.0

        self.cv_bridge = CvBridge()
        self.R_EARTH = 6378137.0
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False
        self.declare_parameter("ground_altitude_m", 0.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._drone_pose = None
        self._home = None

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------

        # Camera info — needed once to set up the pinhole model
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_camera_info, 10
        )

        # Drone pose from Pixhawk via MAVROS
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )

        # Home position — used as the GPS origin for coordinate conversion
        self.create_subscription(
            HomePosition, "/mavros/home_position/home", self._on_home, 10
        )

        # CONTINUOUS RECORDER: every frame, unconditionally
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self._on_every_frame,
            10,
        )

        # TARGET GEOLOCATOR: only fires when image + detections arrive together
        img_sub = message_filters.Subscriber(self, Image, "/camera/image_raw")
        det_sub = message_filters.Subscriber(
            self, Detection2DArray, "/drone_control/detection"
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [img_sub, det_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self._on_synced_data)

        self.get_logger().info(
            f"🚀 Mission Logger Ready.\n"
            f"   All frames  → {self._frames_dir}\n"
            f"   GPS targets → {self._targets_dir}\n"
            f"   CSV log     → {self.csv_path}"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _init_csv(self):
        with open(self.csv_path, mode="w", newline="") as f:
            csv.writer(f).writerow(
                ["Image_Name", "Latitude", "Longitude", "Time_UTC", "YOLO_Confidence"]
            )

    # ------------------------------------------------------------------
    # Standard subscriber callbacks
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True

    def _on_pose(self, msg: PoseStamped) -> None:
        self._drone_pose = msg

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    # ------------------------------------------------------------------
    # Continuous frame recorder
    # ------------------------------------------------------------------

    def _on_every_frame(self, msg: Image) -> None:
        """Saves every incoming frame to disk unconditionally.

        Files are named by a zero-padded counter so they sort correctly
        in any file browser.  A ROS timestamp is embedded in the name
        so you can correlate them with the CSV log later if needed.
        """
        self._frame_counter += 1
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        stamp_sec = msg.header.stamp.sec
        stamp_ns = msg.header.stamp.nanosec
        filename = f"frame_{self._frame_counter:06d}_{stamp_sec}_{stamp_ns}.jpg"
        path = os.path.join(self._frames_dir, filename)

        cv2.imwrite(path, frame)

    # ------------------------------------------------------------------
    # Synchronised target geolocator
    # ------------------------------------------------------------------

    def _on_synced_data(self, img_msg: Image, det_array_msg: Detection2DArray) -> None:
        """Fires only when an image and its matching YOLO array arrive
        within 50 ms of each other."""

        if (
            not self.camera_info_received
            or self._drone_pose is None
            or self._home is None
        ):
            return

        if not det_array_msg.detections:
            return

        frame = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        targets_logged_this_frame = False

        for det in det_array_msg.detections:
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            confidence = det.results[0].hypothesis.score

            gps_coord = self._raycast_to_gps(u, v)
            if gps_coord is None:
                continue
            target_lat, target_lon = gps_coord

            # Spatial deduplication — skip if within min_dist_m of a known target
            is_duplicate = any(
                self._calculate_distance_m(target_lat, target_lon, lat, lon)
                < self.min_dist_m
                for lat, lon in self.saved_target_locations
            )
            if is_duplicate:
                continue

            # New target — annotate the frame
            cv2.circle(frame, (int(u), int(v)), 50, (0, 0, 255), 4)
            text = f"Lat: {target_lat:.6f}, Lon: {target_lon:.6f}"
            cv2.putText(
                frame,
                text,
                (int(u) - 100, int(v) - 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0, 255, 0),
                4,
            )

            # Write to CSV immediately so data survives a crash
            image_filename = f"target_{self._target_counter:03d}.jpg"
            time_utc = datetime.utcnow().strftime("%H:%M:%S")
            with open(self.csv_path, mode="a", newline="") as f:
                csv.writer(f).writerow(
                    [
                        image_filename,
                        f"{target_lat:.7f}",
                        f"{target_lon:.7f}",
                        time_utc,
                        f"{confidence:.2f}",
                    ]
                )

            self.get_logger().info(
                f"✅ LOGGED: {image_filename} → "
                f"Lat: {target_lat:.7f}, Lon: {target_lon:.7f}"
            )
            self.saved_target_locations.append((target_lat, target_lon))
            targets_logged_this_frame = True

        if targets_logged_this_frame:
            image_path = os.path.join(
                self._targets_dir, f"target_{self._target_counter:03d}.jpg"
            )
            cv2.imwrite(image_path, frame)
            self._target_counter += 1

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def _raycast_to_gps(self, u: float, v: float):
        """Project a pixel (u, v) through the camera model and drone attitude
        onto the ground plane, returning (latitude, longitude) or None."""

        # Undistort the single pixel using the camera matrix
        rectified_u, rectified_v = self.camera_model.rectifyPoint((u, v))

        # Shoot the optical ray through the undistorted pixel
        ray_opt = self.camera_model.projectPixelTo3dRay((rectified_u, rectified_v))

        # Look up camera→body transform from TF2
        try:
            t_mount = self.tf_buffer.lookup_transform(
                "base_link", "camera_optical_frame", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF2 lookup failed: {e}")
            return None

        q_m = t_mount.transform.rotation
        r_mount = R_scipy.from_quat([q_m.x, q_m.y, q_m.z, q_m.w])
        cam_offset = (
            t_mount.transform.translation.x,
            t_mount.transform.translation.y,
            t_mount.transform.translation.z,
        )

        # Rotate the ray into the world frame using current drone attitude
        q_d = self._drone_pose.pose.orientation
        r_drone = R_scipy.from_quat([q_d.x, q_d.y, q_d.z, q_d.w])
        world_ray = r_drone.apply(r_mount.apply(ray_opt))

        # Intersect with ground plane
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

        # Convert local NED offsets to GPS
        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude
        lat_offset = (target_y / self.R_EARTH) * (180.0 / math.pi)
        lon_scale = math.cos(math.radians(lat0))
        lon_offset = (target_x / (self.R_EARTH * lon_scale)) * (180.0 / math.pi)

        return (lat0 + lat_offset, lon0 + lon_offset)

    def _calculate_distance_m(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        dy = (lat2 - lat1) * self.R_EARTH * (math.pi / 180.0)
        dx = (
            (lon2 - lon1)
            * self.R_EARTH
            * math.cos(math.radians(lat1))
            * (math.pi / 180.0)
        )
        return math.hypot(dx, dy)


def main():
    rclpy.init()
    node = MissionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
