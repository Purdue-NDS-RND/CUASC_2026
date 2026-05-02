"""
Arducam Jetson Image Grabber (Compressed / Low-Bandwidth Variant)
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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CompressedGrabber(Node):
    def __init__(self) -> None:
        super().__init__("compressed_grabber")

        self.declare_parameter("image_width", 3840)
        self.declare_parameter("image_height", 2160)
        self.declare_parameter("fps", 17)
        self.declare_parameter("shutter_speed", 1000)
        self.declare_parameter("wb_mode", 6)
        self.declare_parameter("image_publishing_rate", 4.0)
        self.declare_parameter("publish_raw_stream", True)
        self.declare_parameter("publish_full_res", False)
        self.declare_parameter("publish_monitor_stream", False)
        self.declare_parameter("publish_compressed_stream", True)
        self.declare_parameter("compressed_quality", 70)
        self.declare_parameter("monitor_width", 960)
        self.declare_parameter("monitor_height", 540)
        self.declare_parameter("camera_info_file", "arducam_info.yaml")
        self.declare_parameter("enable_timelapse", True)
        self.declare_parameter("save_dir", "/home/nds02/camera_captures_calibrated")
        self.declare_parameter("save_interval_sec", 1.0)

        self.width = self.get_parameter("image_width").value
        self.height = self.get_parameter("image_height").value
        fps = self.get_parameter("fps").value
        shutter = self.get_parameter("shutter_speed").value
        wb = self.get_parameter("wb_mode").value
        yaml_file = self.get_parameter("camera_info_file").value
        self._publish_raw_stream = self.get_parameter("publish_raw_stream").value
        self._publish_full_res = self.get_parameter("publish_full_res").value
        self._publish_monitor_stream = self.get_parameter("publish_monitor_stream").value
        self._publish_compressed_stream = self.get_parameter(
            "publish_compressed_stream"
        ).value
        self._compressed_quality = self.get_parameter("compressed_quality").value
        self._monitor_width = self.get_parameter("monitor_width").value
        self._monitor_height = self.get_parameter("monitor_height").value

        self._enable_timelapse = self.get_parameter("enable_timelapse").value
        self._save_dir = self.get_parameter("save_dir").value
        self._save_interval = self.get_parameter("save_interval_sec").value
        self._last_save_time = 0.0

        if self._enable_timelapse:
            os.makedirs(self._save_dir, exist_ok=True)

        self._cv_bridge = CvBridge()

        self._camera_info_msg = self._load_camera_info(yaml_file)
        self._setup_rectification_maps()

        pipeline = self._get_pipeline(self.width, self.height, fps, shutter, wb)
        self._camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self._camera.isOpened():
            raise RuntimeError(
                "Camera failed to initialize — check GStreamer pipeline."
            )
        self._camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_pub = None
        if self._publish_raw_stream:
            self._image_pub = self.create_publisher(Image, "/camera/image_raw", qos)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos)
        self._compressed_pub = None
        if self._publish_compressed_stream:
            self._compressed_pub = self.create_publisher(
                CompressedImage,
                "/camera/image_raw/compressed",
                qos,
            )
        self._monitor_pub = None
        if self._publish_monitor_stream:
            self._monitor_pub = self.create_publisher(
                Image,
                "/camera/image_monitor",
                qos,
            )

        rate = self.get_parameter("image_publishing_rate").value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        self._frames_published = 0
        self._frames_read_failed = 0
        self._last_hz_check_time = time.time()
        self._frames_since_hz = 0

        self.get_logger().info(
            f"CompressedGrabber ready.\n"
            f"   Resolution : {self.width}x{self.height}\n"
            f"   Sensor FPS : {fps}\n"
            f"   Publish Hz : {rate}\n"
            f"   Raw stream : {self._publish_raw_stream}\n"
            f"   Full-res   : {self._publish_full_res}\n"
            f"   Compressed : {self._publish_compressed_stream} "
            f"(jpeg q={self._compressed_quality})\n"
            f"   Monitor pub: {self._publish_monitor_stream}\n"
            f"   Monitor    : {self._monitor_width}x{self._monitor_height}\n"
            f"   QoS        : BEST_EFFORT depth=1\n"
            f"   Timelapse  : {self._enable_timelapse} "
            f"(every {self._save_interval}s -> {self._save_dir})"
        )

    def _load_camera_info(self, filename):
        pkg_share = get_package_share_directory("vision_pipeline")
        yaml_path = os.path.join(pkg_share, "config", filename)
        self.get_logger().info(f"Loading camera calibration: {yaml_path}")

        with open(yaml_path, "r") as fh:
            calib_data = yaml.safe_load(fh)

        msg = CameraInfo()
        msg.header.frame_id = "camera_link"
        msg.width = calib_data["image_width"]
        msg.height = calib_data["image_height"]
        msg.k = calib_data["camera_matrix"]["data"]
        msg.d = calib_data["distortion_coefficients"]["data"]
        msg.distortion_model = calib_data.get("distortion_model", "plumb_bob")

        if "projection_matrix" in calib_data:
            msg.p = calib_data["projection_matrix"]["data"]
        else:
            k = calib_data["camera_matrix"]["data"]
            msg.p = [
                k[0],
                k[1],
                k[2],
                0.0,
                k[3],
                k[4],
                k[5],
                0.0,
                k[6],
                k[7],
                k[8],
                0.0,
            ]
            self.get_logger().warn(
                "projection_matrix not found in YAML — built P from K."
            )

        self.get_logger().info(
            f"   K  = {msg.k[:3]} ...\n   D  = {msg.d}\n   P  = {msg.p[:3]} ..."
        )
        return msg

    def _setup_rectification_maps(self):
        self.get_logger().info("Computing lens distortion maps (alpha=0)...")
        k = np.array(self._camera_info_msg.k).reshape((3, 3))
        d = np.array(self._camera_info_msg.d)

        new_k, roi = cv2.getOptimalNewCameraMatrix(
            k, d, (self.width, self.height), 0, (self.width, self.height)
        )

        self._camera_info_msg.k = new_k.flatten().tolist()
        self._camera_info_msg.p = [
            new_k[0, 0],
            new_k[0, 1],
            new_k[0, 2],
            0.0,
            new_k[1, 0],
            new_k[1, 1],
            new_k[1, 2],
            0.0,
            new_k[2, 0],
            new_k[2, 1],
            new_k[2, 2],
            0.0,
        ]

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            k, d, None, new_k, (self.width, self.height), cv2.CV_16SC2
        )
        self.get_logger().info(f"   new_K = {new_k[0].tolist()} ...\n   ROI   = {roi}")

    def _get_pipeline(self, width, height, fps, shutter_speed_inv, wb_mode):
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
            f"videoconvert ! video/x-raw, format=BGR ! "
            f"appsink drop=1 max-buffers=1 sync=false"
        )
        self.get_logger().info(f"GStreamer pipeline:\n   {pipeline}")
        return pipeline

    def _on_timer(self) -> None:
        try:
            success, raw_frame = self._camera.read()
            if not success:
                self._frames_read_failed += 1
                self.get_logger().warn(
                    f"camera.read() returned False "
                    f"(total failures: {self._frames_read_failed}). "
                    "GStreamer pipeline may have stalled."
                )
                return

            if self._frames_read_failed:
                self._frames_read_failed = 0

            frame = cv2.remap(
                raw_frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR
            )

            now = self.get_clock().now().to_msg()

            monitor_frame = cv2.resize(
                frame,
                (self._monitor_width, self._monitor_height),
                interpolation=cv2.INTER_LINEAR,
            )

            primary_frame = frame if self._publish_full_res else monitor_frame

            if self._image_pub is not None:
                img_msg = self._cv_bridge.cv2_to_imgmsg(primary_frame, encoding="bgr8")
                img_msg.header.stamp = now
                img_msg.header.frame_id = "camera_link"
                self._image_pub.publish(img_msg)

            if self._compressed_pub is not None:
                ok, encoded = cv2.imencode(
                    ".jpg",
                    primary_frame,
                    [
                        int(cv2.IMWRITE_JPEG_QUALITY),
                        int(self._compressed_quality),
                    ],
                )
                if ok:
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = now
                    compressed_msg.header.frame_id = "camera_link"
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded.tobytes()
                    self._compressed_pub.publish(compressed_msg)
                else:
                    self.get_logger().warn(
                        "Failed to JPEG-encode frame for compressed stream"
                    )

            self._camera_info_msg.header.stamp = now
            self._info_pub.publish(self._camera_info_msg)

            if self._monitor_pub is not None:
                monitor_msg = self._cv_bridge.cv2_to_imgmsg(
                    monitor_frame,
                    encoding="bgr8",
                )
                monitor_msg.header.stamp = now
                monitor_msg.header.frame_id = "camera_link"
                self._monitor_pub.publish(monitor_msg)

            if self._enable_timelapse:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self._last_save_time >= self._save_interval:
                    filename = os.path.join(
                        self._save_dir, f"img_{now.sec}_{now.nanosec}.jpg"
                    )
                    cv2.imwrite(filename, frame)
                    self._last_save_time = current_time

            self._frames_published += 1
            self._frames_since_hz += 1
            now_wall = time.time()
            elapsed = now_wall - self._last_hz_check_time
            if elapsed >= 5.0:
                actual_hz = self._frames_since_hz / elapsed
                self.get_logger().info(
                    f"CompressedGrabber — published {self._frames_published} frames total | "
                    f"actual rate: {actual_hz:.1f} Hz | "
                    f"read failures: {self._frames_read_failed}"
                )
                self._frames_since_hz = 0
                self._last_hz_check_time = now_wall

        except Exception as exc:
            self.get_logger().error(
                f"_on_timer exception (timer will continue): {exc}"
            )

    def close(self) -> None:
        if hasattr(self, "_camera"):
            self._camera.release()


def main() -> None:
    rclpy.init()
    node = CompressedGrabber()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
