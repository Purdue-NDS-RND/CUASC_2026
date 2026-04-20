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
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter("publish_full_res", False)
        self.declare_parameter("publish_monitor_stream", False)
        self.declare_parameter("monitor_width", 960)
        self.declare_parameter("monitor_height", 540)
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
        self._publish_full_res = self.get_parameter("publish_full_res").value
        self._publish_monitor_stream = self.get_parameter("publish_monitor_stream").value
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
                "❌ Camera failed to initialize — check GStreamer pipeline."
            )
        self._camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # ------------------------------------------------------------------
        # QoS: BEST_EFFORT + depth=1 so Foxglove always gets the newest frame.
        # RELIABLE with a deep queue causes Foxglove to see stale backlogged
        # frames and appear frozen on the first image.
        # ------------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_pub = self.create_publisher(Image, "/camera/image_raw", qos)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos)
        self._monitor_pub = None
        if self._publish_monitor_stream:
            self._monitor_pub = self.create_publisher(
                Image,
                "/camera/image_monitor",
                qos,
            )

        rate = self.get_parameter("image_publishing_rate").value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        # ------------------------------------------------------------------
        # Verbose diagnostics state
        # ------------------------------------------------------------------
        self._frames_published = 0
        self._frames_read_failed = 0
        self._last_hz_check_time = time.time()
        self._frames_since_hz = 0

        self.get_logger().info(
            f"✅ ImageGrabber ready.\n"
            f"   Resolution : {self.width}x{self.height}\n"
            f"   Sensor FPS : {fps}\n"
            f"   Publish Hz : {rate}\n"
            f"   Full-res   : {self._publish_full_res}\n"
            f"   Monitor pub: {self._publish_monitor_stream}\n"
            f"   Monitor    : {self._monitor_width}x{self._monitor_height}\n"
            f"   QoS        : BEST_EFFORT depth=1\n"
            f"   Timelapse  : {self._enable_timelapse} "
            f"(every {self._save_interval}s → {self._save_dir})"
        )

    # ------------------------------------------------------------------
    # Camera info loader
    # ------------------------------------------------------------------

    def _load_camera_info(self, filename):
        pkg_share = get_package_share_directory("vision_pipeline")
        yaml_path = os.path.join(pkg_share, "config", filename)
        self.get_logger().info(f"📷 Loading camera calibration: {yaml_path}")

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
            K = calib_data["camera_matrix"]["data"]
            msg.p = [
                K[0],
                K[1],
                K[2],
                0.0,
                K[3],
                K[4],
                K[5],
                0.0,
                K[6],
                K[7],
                K[8],
                0.0,
            ]
            self.get_logger().warn(
                "projection_matrix not found in YAML — built P from K. "
                "Run camera_calibration to regenerate the YAML for best accuracy."
            )

        self.get_logger().info(
            f"   K  = {msg.k[:3]} ...\n   D  = {msg.d}\n   P  = {msg.p[:3]} ..."
        )
        return msg

    # ------------------------------------------------------------------
    # Rectification map setup
    # ------------------------------------------------------------------

    def _setup_rectification_maps(self):
        self.get_logger().info("📐 Computing lens distortion maps (alpha=0)...")
        K = np.array(self._camera_info_msg.k).reshape((3, 3))
        D = np.array(self._camera_info_msg.d)

        new_K, roi = cv2.getOptimalNewCameraMatrix(
            K, D, (self.width, self.height), 0, (self.width, self.height)
        )

        # --- ADD THESE LINES: Overwrite the message so downstream nodes use new_K ---
        self._camera_info_msg.k = new_K.flatten().tolist()
        self._camera_info_msg.p = [
            new_K[0, 0],
            new_K[0, 1],
            new_K[0, 2],
            0.0,
            new_K[1, 0],
            new_K[1, 1],
            new_K[1, 2],
            0.0,
            new_K[2, 0],
            new_K[2, 1],
            new_K[2, 2],
            0.0,
        ]
        # ----------------------------------------------------------------------------

        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (self.width, self.height), cv2.CV_16SC2
        )
        self.get_logger().info(f"   new_K = {new_K[0].tolist()} ...\n   ROI   = {roi}")

    # ------------------------------------------------------------------
    # GStreamer pipeline string
    # ------------------------------------------------------------------

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
        self.get_logger().info(f"🎬 GStreamer pipeline:\n   {pipeline}")
        return pipeline

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        try:
            success, raw_frame = self._camera.read()
            if not success:
                self._frames_read_failed += 1
                self.get_logger().warn(
                    f"⚠️  camera.read() returned False "
                    f"(total failures: {self._frames_read_failed}). "
                    "GStreamer pipeline may have stalled."
                )
                return

            if self._frames_read_failed:
                self._frames_read_failed = 0

            # Rectify (undistort) the raw frame
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

            img_msg = self._cv_bridge.cv2_to_imgmsg(primary_frame, encoding="bgr8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = "camera_link"
            self._image_pub.publish(img_msg)

            # --- Camera info ---
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

            # --- Timelapse save ---
            if self._enable_timelapse:
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self._last_save_time >= self._save_interval:
                    filename = os.path.join(
                        self._save_dir, f"img_{now.sec}_{now.nanosec}.jpg"
                    )
                    cv2.imwrite(filename, frame)
                    self._last_save_time = current_time

            # --- Verbose rate logging every 5 seconds ---
            self._frames_published += 1
            self._frames_since_hz += 1
            now_wall = time.time()
            elapsed = now_wall - self._last_hz_check_time
            if elapsed >= 5.0:
                actual_hz = self._frames_since_hz / elapsed
                self.get_logger().info(
                    f"📡 ImageGrabber — published {self._frames_published} frames total | "
                    f"actual rate: {actual_hz:.1f} Hz | "
                    f"read failures: {self._frames_read_failed}"
                )
                self._frames_since_hz = 0
                self._last_hz_check_time = now_wall

        except Exception as e:
            self.get_logger().error(
                f"❌ _on_timer exception (timer will continue): {e}"
            )

    def close(self) -> None:
        if hasattr(self, "_camera"):
            self._camera.release()


def main() -> None:
    rclpy.init()
    node = ImageGrabber()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
