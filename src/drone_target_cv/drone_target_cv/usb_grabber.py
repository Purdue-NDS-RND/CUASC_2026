import os

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class USBGrabber(Node):
    def __init__(self) -> None:
        super().__init__("usb_grabber")

        self.declare_parameter("device_index", 0)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("image_publishing_rate", 15.0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("camera_info_file", "")

        self._device_index = int(self.get_parameter("device_index").value)
        self._width = int(self.get_parameter("image_width").value)
        self._height = int(self.get_parameter("image_height").value)
        self._fps = float(self.get_parameter("fps").value)
        self._publish_rate = float(self.get_parameter("image_publishing_rate").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._camera_info_file = str(self.get_parameter("camera_info_file").value)

        self._cv_bridge = CvBridge()
        self._camera_info_msg = self._load_camera_info(self._camera_info_file)

        self._camera = cv2.VideoCapture(self._device_index)
        self._camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._camera.set(cv2.CAP_PROP_FPS, self._fps)
        self._camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self._camera.isOpened():
            raise RuntimeError(
                f"Failed to open USB camera at device_index={self._device_index}."
            )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_pub = self.create_publisher(Image, "/camera/image", qos)
        self._info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", qos)
        self._timer = self.create_timer(
            1.0 / max(self._publish_rate, 1.0), self._on_timer
        )

        self._frames_published = 0
        self._read_failures = 0

        self.get_logger().info(
            "USBGrabber ready.\n"
            f"   Device     : {self._device_index}\n"
            f"   Resolution : {self._width}x{self._height}\n"
            f"   Sensor FPS : {self._fps}\n"
            f"   Publish Hz : {self._publish_rate}\n"
            f"   Frame ID   : {self._frame_id}\n"
            f"   CameraInfo : {self._camera_info_file or 'minimal fallback'}"
        )

    def _camera_info_path(self, filename: str) -> str:
        if os.path.isabs(filename):
            return filename
        pkg_share = get_package_share_directory("drone_target_cv")
        return os.path.join(pkg_share, "config", filename)

    def _minimal_camera_info(self) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = self._frame_id
        msg.width = self._width
        msg.height = self._height
        return msg

    def _load_camera_info(self, filename: str) -> CameraInfo:
        if not filename:
            self.get_logger().warn(
                "camera_info_file not set; publishing minimal CameraInfo."
            )
            return self._minimal_camera_info()

        yaml_path = self._camera_info_path(filename)
        if not os.path.exists(yaml_path):
            self.get_logger().warn(
                f"Camera info file not found: {yaml_path}. "
                "Publishing minimal CameraInfo."
            )
            return self._minimal_camera_info()

        with open(yaml_path, "r") as fh:
            calib_data = yaml.safe_load(fh)

        msg = self._minimal_camera_info()
        msg.width = int(calib_data.get("image_width", self._width))
        msg.height = int(calib_data.get("image_height", self._height))
        msg.k = calib_data.get("camera_matrix", {}).get("data", msg.k)
        msg.d = calib_data.get("distortion_coefficients", {}).get("data", msg.d)
        msg.distortion_model = calib_data.get("distortion_model", "plumb_bob")
        msg.r = calib_data.get(
            "rectification_matrix",
            {"data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        )["data"]

        if "projection_matrix" in calib_data:
            msg.p = calib_data["projection_matrix"]["data"]
        elif msg.k:
            msg.p = [
                msg.k[0],
                msg.k[1],
                msg.k[2],
                0.0,
                msg.k[3],
                msg.k[4],
                msg.k[5],
                0.0,
                msg.k[6],
                msg.k[7],
                msg.k[8],
                0.0,
            ]

        self.get_logger().info(f"Loaded camera info from {yaml_path}")
        return msg

    def _on_timer(self) -> None:
        success, frame = self._camera.read()
        if not success:
            self._read_failures += 1
            self.get_logger().warn(
                f"USB camera read failed (total failures: {self._read_failures})."
            )
            return

        now = self.get_clock().now().to_msg()
        image_msg = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = now
        image_msg.header.frame_id = self._frame_id
        self._image_pub.publish(image_msg)

        self._camera_info_msg.header.stamp = now
        self._camera_info_msg.header.frame_id = self._frame_id
        self._camera_info_msg.width = frame.shape[1]
        self._camera_info_msg.height = frame.shape[0]
        self._info_pub.publish(self._camera_info_msg)

        self._frames_published += 1

    def close(self) -> None:
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
