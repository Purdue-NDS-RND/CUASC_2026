"""
Arducam Jetson Image Grabber Node (Fully Rectified & Calibrated)
"""

import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class ImageGrabber(Node):
    def __init__(self) -> None:
        super().__init__("image_grabber")

        self.declare_parameter("image_width", 3840)
        self.declare_parameter("image_height", 2160)
        self.declare_parameter("fps", 17)
        self.declare_parameter("shutter_speed", 1000)
        self.declare_parameter("wb_mode", 6)
        self.declare_parameter("image_publishing_rate", 4.0)
        self.declare_parameter("camera_info_file", "arducam_info.yaml")

        self.declare_parameter("enable_timelapse", True)
        self.declare_parameter("save_dir", "/home/nds2/camera_captures_calibrated")
        self.declare_parameter("save_interval_sec", 1.0)

        self.width = self.get_parameter("image_width").value
        self.height = self.get_parameter("image_height").value
        fps = self.get_parameter("fps").value
        shutter = self.get_parameter("shutter_speed").value
        wb = self.get_parameter("wb_mode").value
        yaml_file = self.get_parameter("camera_info_file").value

        self._enable_timelapse = self.get_parameter("enable_timelapse").value
        self._save_dir = self.get_parameter("save_dir").value
        self._save_interval = self.get_parameter("save_interval_sec").value
        self._last_save_time = 0.0

        if self._enable_timelapse:
            os.makedirs(self._save_dir, exist_ok=True)

        self._cv_bridge = CvBridge()

        # --- NEW: Setup the Rectification Maps ---
        self._camera_info_msg = self._load_camera_info(yaml_file)
        self._setup_rectification_maps()

        pipeline = self._get_pipeline(self.width, self.height, fps, shutter, wb)
        self._camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self._camera.isOpened():
            raise RuntimeError("Camera failed to initialize.")

        self._image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        rate = self.get_parameter("image_publishing_rate").value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)
        self.get_logger().info(f"✅ Calibrated Camera Ready at {rate} Hz")

    def _load_camera_info(self, filename):
        pkg_share = get_package_share_directory("vision_pipeline")
        yaml_path = os.path.join(pkg_share, "config", filename)
        with open(yaml_path, "r") as file_handle:
            calib_data = yaml.safe_load(file_handle)

        msg = CameraInfo()
        msg.header.frame_id = "camera_link"
        msg.width = calib_data["image_width"]
        msg.height = calib_data["image_height"]
        msg.k = calib_data["camera_matrix"]["data"]
        msg.d = calib_data["distortion_coefficients"]["data"]
        return msg

    def _setup_rectification_maps(self):
        """Pre-computes the heavy math for the dewarping process."""
        self.get_logger().info("📐 Computing lens distortion maps...")
        K = np.array(self._camera_info_msg.k).reshape((3, 3))
        D = np.array(self._camera_info_msg.d)

        # Alpha=0 removes black pixels created by dewarping
        new_K, roi = cv2.getOptimalNewCameraMatrix(
            K, D, (self.width, self.height), 0, (self.width, self.height)
        )
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (self.width, self.height), cv2.CV_16SC2
        )

    def _get_pipeline(self, width, height, fps, shutter_speed_inv, wb_mode):
        exp_str = (
            f"exposuretimerange='{int(1000000000 / shutter_speed_inv)} {int(1000000000 / shutter_speed_inv)}'"
            if shutter_speed_inv > 0
            else ""
        )
        return (
            f"nvarguscamerasrc sensor-id=0 {exp_str} wbmode={wb_mode} ! "
            f"video/x-raw(memory:NVMM), width={width}, height={height}, "
            f"format=NV12, framerate={fps}/1 ! "
            f"nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
        )

    def _on_timer(self) -> None:
        success, raw_frame = self._camera.read()
        if not success:
            return

        # --- THE FIX: INSTANT RECTIFICATION ---
        # Flatten the image perfectly before anyone else sees it
        frame = cv2.remap(
            raw_frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
        )

        now = self.get_clock().now().to_msg()

        img_msg = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "camera_link"
        self._image_pub.publish(img_msg)

        self._camera_info_msg.header.stamp = now
        self._info_pub.publish(self._camera_info_msg)

        if self._enable_timelapse:
            current_time = time.time()
            if current_time - self._last_save_time >= self._save_interval:
                filename = os.path.join(
                    self._save_dir, f"img_{now.sec}_{now.nanosec}.jpg"
                )
                cv2.imwrite(filename, frame)
                self._last_save_time = current_time


def main() -> None:
    rclpy.init()
    node = ImageGrabber()
    rclpy.spin(node)
    if hasattr(node, "_camera"):
        node._camera.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
