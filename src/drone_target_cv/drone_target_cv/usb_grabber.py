import os

import cv2
import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


CAMERA_DEVICE_PATHS = {
    "rolling": (
        "/dev/v4l/by-id/"
        "usb-Arducam_Technology_Co.__Ltd._Arducam_12MP_SN0001-video-index0"
    ),
    "global": (
        "/dev/v4l/by-id/"
        "usb-Arducam_Technology_Co.__Ltd._Arducam_OV9782_USB_Camera_UC852-video-index0"
    ),
}


class USBGrabber(Node):
    def __init__(self) -> None:
        super().__init__("usb_grabber")

        self.declare_parameter("camera_type", "rolling")
        self.declare_parameter("device_path", "")
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("publish_width", 640)
        self.declare_parameter("publish_height", 360)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("image_publishing_rate", 30.0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("publish_raw", False)
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("compressed_quality", 20)

        self._camera_type = (
            str(self.get_parameter("camera_type").value).strip().lower()
        )
        self._device_path = self._resolve_device_path(
            str(self.get_parameter("device_path").value)
        )
        self._width = int(self.get_parameter("image_width").value)
        self._height = int(self.get_parameter("image_height").value)
        self._publish_width = int(self.get_parameter("publish_width").value)
        self._publish_height = int(self.get_parameter("publish_height").value)
        self._fps = float(self.get_parameter("fps").value)
        self._publish_rate = float(self.get_parameter("image_publishing_rate").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publish_raw = bool(self.get_parameter("publish_raw").value)
        self._publish_compressed = bool(self.get_parameter("publish_compressed").value)
        self._compressed_quality = int(self.get_parameter("compressed_quality").value)

        self._camera_info_msg = self._minimal_camera_info()

        self._resolved_device_path = os.path.realpath(self._device_path)
        self._frame_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._latest_frame = None
        self._latest_frame_seq = 0
        self._last_published_seq = 0
        self._capture_failures = 0
        self._captured_frames = 0
        self._published_frames = 0
        self._last_rate_log_time = time.time()
        self._captured_since_log = 0
        self._published_since_log = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_pub = None
        if self._publish_raw:
            self._image_pub = self.create_publisher(Image, "/camera/image", qos)
        self._compressed_pub = None
        if self._publish_compressed:
            self._compressed_pub = self.create_publisher(
                CompressedImage,
                "/camera/image/compressed",
                qos,
            )
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos)

        self._camera = self._open_camera()

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="usb_grabber_capture",
            daemon=True,
        )
        self._capture_thread.start()

        self._timer = self.create_timer(
            1.0 / max(self._publish_rate, 1.0), self._on_timer
        )
        actual_width = int(self._camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self._camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self._camera.get(cv2.CAP_PROP_FPS))
        actual_fourcc = int(self._camera.get(cv2.CAP_PROP_FOURCC))
        fourcc_text = "".join(
            chr((actual_fourcc >> (8 * shift)) & 0xFF) for shift in range(4)
        ).strip("\x00")
        backend_name = self._camera.getBackendName()

        self.get_logger().info(
            "USBGrabber ready.\n"
            f"   Camera     : {self._camera_type}\n"
            f"   Device     : {self._device_path}\n"
            f"   Resolved   : {self._resolved_device_path}\n"
            f"   Backend    : {backend_name}\n"
            f"   Capture Res: {actual_width}x{actual_height}\n"
            f"   Publish Res: {self._publish_width}x{self._publish_height}\n"
            f"   Sensor FPS : {actual_fps}\n"
            f"   Publish Hz : {self._publish_rate}\n"
            f"   Frame ID   : {self._frame_id}\n"
            f"   Pixel fmt  : {fourcc_text or 'unknown'}\n"
            f"   Raw pub    : {self._publish_raw}\n"
            f"   Compressed : {self._publish_compressed} "
            f"(jpeg q={self._compressed_quality})\n"
            "   CameraInfo : minimal only (no intrinsics)"
        )

    def _resolve_device_path(self, device_path_override: str) -> str:
        device_path = device_path_override.strip()
        if device_path:
            return device_path

        if self._camera_type in CAMERA_DEVICE_PATHS:
            return CAMERA_DEVICE_PATHS[self._camera_type]

        options = ", ".join(sorted(CAMERA_DEVICE_PATHS))
        raise RuntimeError(
            f"Unsupported camera_type '{self._camera_type}'. "
            f"Expected one of: {options}."
        )

    def _minimal_camera_info(self) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = self._frame_id
        msg.width = self._width
        msg.height = self._height
        return msg

    def _open_camera(self) -> cv2.VideoCapture:
        camera = cv2.VideoCapture(self._resolved_device_path, cv2.CAP_V4L2)
        if not camera.isOpened():
            raise RuntimeError(
                "Failed to open USB camera.\n"
                f"Requested path: {self._device_path}\n"
                f"Resolved path : {self._resolved_device_path}"
            )

        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        camera.set(cv2.CAP_PROP_FPS, self._fps)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return camera

    def _capture_loop(self) -> None:
        while not self._stop_event.is_set():
            success, frame = self._camera.read()
            if not success:
                self._capture_failures += 1
                time.sleep(0.01)
                continue

            with self._frame_lock:
                self._latest_frame = frame
                self._latest_frame_seq += 1

            self._captured_frames += 1
            self._captured_since_log += 1

    def _on_timer(self) -> None:
        with self._frame_lock:
            if self._latest_frame is None or self._latest_frame_seq == self._last_published_seq:
                return
            frame = self._latest_frame.copy()
            self._last_published_seq = self._latest_frame_seq

        if frame is None:
            return

        if (
            frame.shape[1] != self._publish_width
            or frame.shape[0] != self._publish_height
        ):
            frame = cv2.resize(
                frame,
                (self._publish_width, self._publish_height),
                interpolation=cv2.INTER_AREA,
            )

        now = self.get_clock().now().to_msg()
        if self._image_pub is not None:
            image_msg = Image()
            image_msg.header.stamp = now
            image_msg.header.frame_id = self._frame_id
            image_msg.height = frame.shape[0]
            image_msg.width = frame.shape[1]
            image_msg.encoding = "bgr8"
            image_msg.is_bigendian = 0
            image_msg.step = frame.shape[1] * frame.shape[2]
            image_msg.data = frame.tobytes()
            self._image_pub.publish(image_msg)

        if self._compressed_pub is not None:
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [
                    int(cv2.IMWRITE_JPEG_QUALITY),
                    int(self._compressed_quality),
                ],
            )
            if ok:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = now
                compressed_msg.header.frame_id = self._frame_id
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded.tobytes()
                self._compressed_pub.publish(compressed_msg)

        self._camera_info_msg.header.stamp = now
        self._camera_info_msg.header.frame_id = self._frame_id
        self._camera_info_msg.width = frame.shape[1]
        self._camera_info_msg.height = frame.shape[0]
        self._info_pub.publish(self._camera_info_msg)

        self._published_frames += 1
        self._published_since_log += 1
        self._maybe_log_rates()

    def _maybe_log_rates(self) -> None:
        now_wall = time.time()
        elapsed = now_wall - self._last_rate_log_time
        if elapsed < 5.0:
            return

        capture_rate = self._captured_since_log / elapsed
        publish_rate = self._published_since_log / elapsed
        self.get_logger().info(
            "USBGrabber rates | "
            f"capture: {capture_rate:.1f} Hz | "
            f"publish: {publish_rate:.1f} Hz | "
            f"capture failures: {self._capture_failures}"
        )
        self._captured_since_log = 0
        self._published_since_log = 0
        self._last_rate_log_time = now_wall

    def close(self) -> None:
        self._stop_event.set()
        if hasattr(self, "_capture_thread") and self._capture_thread is not None and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=1.0)
        if hasattr(self, "_camera"):
            self._camera.release()


def main() -> None:
    rclpy.init()
    node = USBGrabber()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
