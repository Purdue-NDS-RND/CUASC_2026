"""
Arducam Jetson Image Grabber Node (Calibrated)
"""

import os

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class ImageGrabber(Node):
    def __init__(self) -> None:
        super().__init__("image_grabber")

        # Hardware Parameters
        self.declare_parameter("image_width", 3840)
        self.declare_parameter("image_height", 2160)
        self.declare_parameter("fps", 17)
        self.declare_parameter("shutter_speed", 1000)
        self.declare_parameter("wb_mode", 6)
        self.declare_parameter("image_publishing_rate", 4.0)

        # New Parameter: Name of your calibration file
        self.declare_parameter("camera_info_file", "arducam_info.yaml")

        width = self.get_parameter("image_width").value
        height = self.get_parameter("image_height").value
        fps = self.get_parameter("fps").value
        shutter = self.get_parameter("shutter_speed").value
        wb = self.get_parametgit config pull.rebase false  # mergeer("wb_mode").value
        yaml_file = self.get_parameter("camera_info_file").value

        self._cv_bridge = CvBridge()

        # Load Camera Info
        self._camera_info_msg = self._load_camera_info(yaml_file)

        # GStreamer Pipeline
        pipeline = self._get_pipeline(width, height, fps, shutter, wb)
        self.get_logger().info(f"Opening camera with pipeline:\n{pipeline}")
        self._camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self._camera.isOpened():
            self.get_logger().error("❌ Failed to open Arducam via GStreamer!")
            raise RuntimeError("Camera failed to initialize.")

        # Publishers
        self._image_pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        # Timer
        rate = self.get_parameter("image_publishing_rate").value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        self.get_logger().info(f"✅ Camera Ready. Publishing Image & Info at {rate} Hz")

    def _load_camera_info(self, filename):
        """Finds and parses the YAML calibration file into a CameraInfo message."""
        try:
            # Dynamically find the config folder in the installed package
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
            msg.r = calib_data["rectification_matrix"]["data"]
            msg.p = calib_data["projection_matrix"]["data"]
            msg.distortion_model = calib_data["distortion_model"]
            self.get_logger().info(f"✅ Successfully loaded calibration: {filename}")
            return msg
        except Exception as e:
            self.get_logger().warn(
                f"⚠️ Could not load calibration file {filename}. Error: {e}"
            )
            return CameraInfo()  # Return empty message if it fails

    def _get_pipeline(self, width, height, fps, shutter_speed_inv, wb_mode):
        if shutter_speed_inv > 0:
            exposure_ns = int(1000000000 / shutter_speed_inv)
            exp_str = f"exposuretimerange='{exposure_ns} {exposure_ns}'"
        else:
            exp_str = ""

        wb_str = f"wbmode={wb_mode}"

        return (
            f"nvarguscamerasrc sensor-id=0 {exp_str} {wb_str} ! "
            f"video/x-raw(memory:NVMM), width={width}, height={height}, "
            f"format=NV12, framerate={fps}/1 ! "
            f"nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
        )

    def _on_timer(self) -> None:
        success, frame = self._camera.read()
        if not success:
            return

        now = self.get_clock().now().to_msg()

        # Publish Image
        img_msg = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "camera_link"
        self._image_pub.publish(img_msg)

        # Publish Camera Info with identical timestamp
        self._camera_info_msg.header.stamp = now
        self._info_pub.publish(self._camera_info_msg)


def main() -> None:
    rclpy.init()
    node = ImageGrabber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "_camera"):
            node._camera.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
