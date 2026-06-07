"""
Background Flight Data Logger
Subscribes to camera feeds and flight telemetry, logging synchronized
snapshots directly to a session directory for offline raycasting.
"""

import csv
import os
from datetime import datetime

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class BackgroundDataLogger(Node):
    def __init__(self) -> None:
        super().__init__("background_data_logger")

        # Session Setup
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        base_session_dir = os.path.expanduser("~/raycast_sessions")
        os.makedirs(base_session_dir, exist_ok=True)

        self.session_dir = os.path.join(
            base_session_dir, f"Raycast_Session_{timestamp_str}"
        )
        self.images_dir = os.path.join(self.session_dir, "raw_frames")
        os.makedirs(self.images_dir, exist_ok=True)

        self.csv_path = os.path.join(self.session_dir, "telemetry_metadata.csv")
        self.camera_info_path = os.path.join(self.session_dir, "camera_info.yaml")
        self._init_csv()

        # Cache states
        self._current_pose = None
        self._home_position = None
        self._camera_info_saved = False
        self._cv_bridge = CvBridge()
        self._frame_count = 0

        # QoS Profiles
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        transient_local_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribers
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._pose_callback,
            best_effort_qos,
        )
        self.create_subscription(
            HomePosition,
            "/mavros/home_position/home",
            self._home_callback,
            transient_local_qos,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self._camera_info_callback,
            best_effort_qos,
        )
        self.create_subscription(
            Image, "/camera/image_raw", self._image_callback, best_effort_qos
        )

        self.get_logger().info(
            f"📊 Background Logger Initialized.\nSaving session data to: {self.session_dir}"
        )

    def _init_csv(self):
        headers = [
            "filename",
            "stamp_sec",
            "stamp_nanosec",
            "drone_x",
            "drone_y",
            "drone_z",
            "qx",
            "qy",
            "qz",
            "qw",
            "home_lat",
            "home_lon",
            "home_alt",
        ]
        with open(self.csv_path, mode="w", newline="") as f:
            csv.writer(f).writerow(headers)

    def _pose_callback(self, msg: PoseStamped):
        self._current_pose = msg

    def _home_callback(self, msg: HomePosition):
        self._home_position = msg

    def _camera_info_callback(self, msg: CameraInfo):
        if self._camera_info_saved:
            return

        try:
            self.get_logger().info(
                f"Received CameraInfo: width={msg.width}, height={msg.height}"
            )

            cam_data = {
                "image_width": int(msg.width),
                "image_height": int(msg.height),
                "camera_matrix": {"data": [float(x) for x in msg.k]},
                "distortion_coefficients": {"data": [float(x) for x in msg.d]},
                "projection_matrix": {"data": [float(x) for x in msg.p]},
            }

            with open(self.camera_info_path, "w") as f:
                yaml.safe_dump(cam_data, f, default_flow_style=False)

            self._camera_info_saved = True

            self.get_logger().info(
                f"Camera calibration saved to {self.camera_info_path}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to save camera calibration: {e}")

    def _image_callback(self, msg: Image):
        if self._current_pose is None:
            self.get_logger().warn(
                "Skipping frame: Local pose telemetry stream not available yet.",
                throttle_duration_sec=3.0,
            )
            return
        if self._home_position is None:
            self.get_logger().warn(
                "Skipping frame: Home GPS anchor locked frame missing.",
                throttle_duration_sec=3.0,
            )
            return

        self._frame_count += 1
        filename = f"frame_{self._frame_count:06d}.jpg"
        filepath = os.path.join(self.images_dir, filename)

        try:
            # Convert and save the raw image array
            cv_img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imwrite(filepath, cv_img)

            # Extract instantaneous physical state matching image shutter arrival
            pos = self._current_pose.pose.position
            ori = self._current_pose.pose.orientation

            row = [
                filename,
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                pos.x,
                pos.y,
                pos.z,
                ori.x,
                ori.y,
                ori.z,
                ori.w,
                self._home_position.geo.latitude,
                self._home_position.geo.longitude,
                self._home_position.geo.altitude,
            ]

            with open(self.csv_path, mode="a", newline="") as f:
                csv.writer(f).writerow(row)

        except Exception as e:
            self.get_logger().error(f"Failed to log snapshot data frame: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = BackgroundDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
