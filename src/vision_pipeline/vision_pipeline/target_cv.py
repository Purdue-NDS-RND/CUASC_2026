"""
Target CV Node — Detect a coloured target from a downward-facing camera.

Subscribes to:
  camera/image              (sensor_msgs/Image)            — raw camera frame
  camera/image/compressed   (sensor_msgs/CompressedImage)  — compressed camera frame

Publishes:
  /drone_package_drop/target_detection  (PointStamped)  — centered normalized
                                                          target offsets in [-1, 1]
  /drone_package_drop/image_size        (PointStamped)  — raw image width/height
                                                          for observability/debugging
  /drone_package_drop/debug_image       (sensor_msgs/Image) — annotated debug stream
                                                              when debug_view=True

The detection uses a simple HSV colour filter.  Tune the HSV bounds
via parameters to match whatever colour your ground target is.

Parameters:
  image_topic       Camera topic name            (default "camera/image")
  compressed_input  Subscribe to CompressedImage instead of Image
  debug_view        Publish a debug image stream (default False)
"""

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool


class TargetCV(Node):
    def __init__(self) -> None:
        super().__init__("target_cv")

        # Parameters
        self.declare_parameter("image_topic", "camera/image")
        self.declare_parameter("compressed_input", False)
        self.declare_parameter("debug_view", False)
        self.declare_parameter("start_enabled", True)

        self._image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self._compressed_input = (
            self.get_parameter("compressed_input").get_parameter_value().bool_value
        )
        self._enabled = False
        self._image_sub = None
        self.last_detection = None

        self._enable_service = self.create_service(
            SetBool,
            "/drone_package_drop/set_target_cv_enabled",
            self._handle_enable_request,
        )

        self._detection_pub = self.create_publisher(
            PointStamped, "/drone_package_drop/target_detection", 10
        )
        self._image_size_pub = self.create_publisher(
            PointStamped, "/drone_package_drop/image_size", 10
        )
        self._debug_enabled = (
            self.get_parameter("debug_view").get_parameter_value().bool_value
        )
        self._debug_image_pub = None
        if self._debug_enabled:
            self._debug_image_pub = self.create_publisher(
                Image, "/drone_package_drop/debug_image", 10
            )

        self._image_size: tuple[int, int] | None = None

        self._set_processing_enabled(
            self.get_parameter("start_enabled").get_parameter_value().bool_value
        )
        self.get_logger().info(
            "TargetCV node started — listening on "
            f"'{self._image_topic}' (compressed={self._compressed_input})"
        )

        self.timer = self.create_timer(1.0, self._timer_callback)

    def _timer_callback(self):
        if not self._enabled:
            return
        if self.last_detection:
            x_norm, y_norm, area = self.last_detection
            self.get_logger().info(
                f"Current detection: x_norm={x_norm:.3f}, "
                f"y_norm={y_norm:.3f}, area={area}"
            )
        else:
            self.get_logger().info("No target detected.")

    def _handle_enable_request(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        self._set_processing_enabled(request.data)
        response.success = True
        response.message = (
            "Target detection enabled"
            if request.data
            else "Target detection disabled"
        )
        return response

    def _set_processing_enabled(self, enabled: bool) -> None:
        if enabled == self._enabled:
            return

        if enabled:
            msg_type = CompressedImage if self._compressed_input else Image
            callback = (
                self._on_compressed_image if self._compressed_input else self._on_image
            )
            self._image_sub = self.create_subscription(
                msg_type,
                self._image_topic,
                callback,
                qos_profile_sensor_data,
            )
            self._enabled = True
            self.get_logger().info("Target detection enabled")
            return

        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None
        self._enabled = False
        self.last_detection = None
        self._image_size = None
        self._publish_not_found_detection()
        self.get_logger().info("Target detection disabled")

    def _publish_not_found_detection(self) -> None:
        det = PointStamped()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = "camera"
        # Keep (-1, -1) as the sentinel for "not found".
        det.point.x = -1.0
        det.point.y = -1.0
        det.point.z = 0.0
        self._detection_pub.publish(det)

    def _imgmsg_to_cv2(self, msg: Image) -> np.ndarray:
        """Convert ROS Image message to OpenCV image without cv_bridge."""
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.encoding == "rgb8":
            frame = data.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgr8":
            return data.reshape((msg.height, msg.width, 3))
        if msg.encoding == "mono8":
            return data.reshape((msg.height, msg.width))
        if msg.encoding == "rgba8":
            frame = data.reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def _detect_target_center_moments(self, image):
        annotated = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blur = cv2.GaussianBlur(hsv, (5, 5), 0)

        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(blur, lower_red1, upper_red1)
        mask2 = cv2.inRange(blur, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)

        moments = cv2.moments(clean)
        if moments["m00"] == 0:
            return annotated, None, None

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        area = moments["m00"]

        cv2.drawMarker(
            annotated,
            (center_x, center_y),
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=22,
            thickness=2,
        )
        cv2.circle(annotated, (center_x, center_y), 5, (0, 0, 255), -1)

        return annotated, (center_x, center_y), area

    @staticmethod
    def _normalize_offset(pixel_value: int, frame_extent: int) -> float:
        return ((float(pixel_value) / float(frame_extent)) * 2.0) - 1.0

    def _process_frame(
        self,
        frame: np.ndarray,
        header,
        width: int,
        height: int,
    ) -> None:
        if self._image_size != (width, height):
            self._image_size = (width, height)
            dims = PointStamped()
            dims.header.stamp = self.get_clock().now().to_msg()
            dims.header.frame_id = "camera"
            dims.point.x = float(width)
            dims.point.y = float(height)
            dims.point.z = 0.0
            self._image_size_pub.publish(dims)
            self.get_logger().info(f"Image size: {width}x{height}")

        annotated, center, area = self._detect_target_center_moments(frame)

        if self._debug_image_pub is not None:
            self._debug_image_pub.publish(
                Image(
                    header=header,
                    height=annotated.shape[0],
                    width=annotated.shape[1],
                    encoding="bgr8",
                    is_bigendian=0,
                    data=annotated.tobytes(),
                )
            )

        if center is not None:
            center_x, center_y = center
            x_norm = self._normalize_offset(center_x, width)
            y_norm = self._normalize_offset(center_y, height)
            det = PointStamped()
            det.header.stamp = self.get_clock().now().to_msg()
            det.header.frame_id = "camera"
            det.point.x = x_norm
            det.point.y = y_norm
            det.point.z = float(area) if area is not None else 0.0
            self._detection_pub.publish(det)
            self.last_detection = (x_norm, y_norm, area)
        else:
            self.last_detection = None
            self._publish_not_found_detection()

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._imgmsg_to_cv2(msg)
        except Exception as exc:
            self.get_logger().warn(f"Image conversion error: {exc}")
            return

        self._process_frame(frame, msg.header, msg.width, msg.height)

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Compressed image decode failed")
            return
        self._process_frame(frame, msg.header, frame.shape[1], frame.shape[0])


def main() -> None:
    rclpy.init()
    node = TargetCV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
