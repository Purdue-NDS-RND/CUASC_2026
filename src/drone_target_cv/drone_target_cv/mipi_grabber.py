"""
Arducam Jetson Image Grabber Node (Minimal & Threaded with Compression)
"""

import os
import threading
import time

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class MIPIGrabber(Node):
    def __init__(self) -> None:
        super().__init__("mipi_grabber")

        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("fps", 60)
        self.declare_parameter("shutter_speed", 1000)
        self.declare_parameter("wb_mode", 6)
        self.declare_parameter("image_publishing_rate", 30.0)
        self.declare_parameter("camera_info_file", "mipi_info.yaml")
        self.declare_parameter("compressed_quality", 20)  # Added to match usb_grabber

        self._width = self.get_parameter("image_width").value
        self._height = self.get_parameter("image_height").value
        fps = self.get_parameter("fps").value
        shutter = self.get_parameter("shutter_speed").value
        wb = self.get_parameter("wb_mode").value
        yaml_file = self.get_parameter("camera_info_file").value
        publish_rate = float(self.get_parameter("image_publishing_rate").value)
        self._compressed_quality = int(self.get_parameter("compressed_quality").value)

        self._cv_bridge = CvBridge()
        self._camera_info_msg = self._load_scaled_camera_info(yaml_file)

        pipeline = self._get_pipeline(self._width, self._height, fps, shutter, wb)
        self._camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self._camera.isOpened():
            raise RuntimeError(
                "❌ Camera failed to initialize — check GStreamer pipeline."
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create both publishers
        self._image_pub = self.create_publisher(Image, "/camera/image", qos)
        self._compressed_pub = self.create_publisher(
            CompressedImage, "/camera/image/compressed", qos
        )
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos)

        # Threading state for decoupled capture
        self._frame_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._latest_frame = None
        self._latest_frame_seq = 0
        self._last_published_seq = 0

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="mipi_grabber_capture",
            daemon=True,
        )
        self._capture_thread.start()

        self._timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._on_timer)

        self.get_logger().info(
            f"✅ MIPIGrabber ready.\n"
            f"   Resolution : {self._width}x{self._height}\n"
            f"   Sensor FPS : {fps}\n"
            f"   Publish Hz : {publish_rate}\n"
            f"   JPEG Qual  : {self._compressed_quality}\n"
            f"   QoS        : BEST_EFFORT depth=1"
        )

    def _load_scaled_camera_info(self, filename: str) -> CameraInfo:
        pkg_share = get_package_share_directory("drone_target_cv")
        yaml_path = os.path.join(pkg_share, "config", filename)

        with open(yaml_path, "r") as fh:
            calib_data = yaml.safe_load(fh)

        msg = CameraInfo()
        msg.header.frame_id = "camera_link"
        msg.width = self._width
        msg.height = self._height

        # Scale K matrix from 4K calibration to current target resolution
        orig_w = calib_data["image_width"]
        orig_h = calib_data["image_height"]
        scale_x = self._width / orig_w
        scale_y = self._height / orig_h

        K = calib_data["camera_matrix"]["data"]

        # Explicitly cast every element to float() to satisfy ROS 2 type bindings
        msg.k = [
            float(K[0] * scale_x),
            float(K[1]),
            float(K[2] * scale_x),
            float(K[3]),
            float(K[4] * scale_y),
            float(K[5] * scale_y),
            float(K[6]),
            float(K[7]),
            float(K[8]),
        ]

        # Safely cast the distortion coefficients as well
        msg.d = [float(x) for x in calib_data["distortion_coefficients"]["data"]]
        msg.distortion_model = calib_data.get("distortion_model", "plumb_bob")

        msg.p = [
            float(msg.k[0]),
            float(msg.k[1]),
            float(msg.k[2]),
            0.0,
            float(msg.k[3]),
            float(msg.k[4]),
            float(msg.k[5]),
            0.0,
            float(msg.k[6]),
            float(msg.k[7]),
            float(msg.k[8]),
            0.0,
        ]
        return msg

    def _get_pipeline(
        self, width: int, height: int, fps: int, shutter_speed_inv: int, wb_mode: int
    ) -> str:
        exp_str = (
            f"exposuretimerange='{int(1e9 / shutter_speed_inv)} {int(1e9 / shutter_speed_inv)}'"
            if shutter_speed_inv > 0
            else ""
        )
        pipeline = (
            f"nvarguscamerasrc sensor-id=0 {exp_str} wbmode={wb_mode} ! "
            f"video/x-raw(memory:NVMM), width={width}, height={height}, "
            f"format=NV12, framerate={fps}/1 ! "
            f"nvvidconv ! video/x-raw, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
        )
        return pipeline

    def _capture_loop(self) -> None:
        while not self._stop_event.is_set():
            success, frame = self._camera.read()
            if not success:
                time.sleep(0.01)
                continue

            with self._frame_lock:
                self._latest_frame = frame
                self._latest_frame_seq += 1

    def _on_timer(self) -> None:
        with self._frame_lock:
            if (
                self._latest_frame is None
                or self._latest_frame_seq == self._last_published_seq
            ):
                return
            frame = self._latest_frame.copy()
            self._last_published_seq = self._latest_frame_seq

        now = self.get_clock().now().to_msg()

        # 1. Publish Raw Image for Target CV
        img_msg = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "camera_link"
        self._image_pub.publish(img_msg)

        # 2. Encode and Publish Compressed Image for Laptop Telemetry
        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._compressed_quality]
        )
        if ok:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = now
            compressed_msg.header.frame_id = "camera_link"
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded.tobytes()
            self._compressed_pub.publish(compressed_msg)

        # 3. Publish Camera Info
        self._camera_info_msg.header.stamp = now
        self._info_pub.publish(self._camera_info_msg)

    def close(self) -> None:
        self._stop_event.set()
        if hasattr(self, "_capture_thread") and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        if hasattr(self, "_camera"):
            self._camera.release()


def main() -> None:
    rclpy.init()
    node = MIPIGrabber()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
