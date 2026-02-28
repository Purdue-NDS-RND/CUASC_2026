"""
Target CV Node — Detect a coloured target from a downward-facing camera.

Subscribes to:
  camera/image              (sensor_msgs/Image)   — raw camera frame

Publishes:
  /payload_drop/target_detection  (PointStamped)  — pixel (x, y) of target centre

The detection uses a simple HSV colour filter.  Tune the HSV bounds
via parameters to match whatever colour your ground target is.

Parameters:
  image_topic       Camera topic name           (default "camera/image")
  target_h_low      HSV hue lower bound  0-179  (default 0)
  target_h_high     HSV hue upper bound  0-179  (default 10)
  target_s_low      HSV sat lower bound  0-255  (default 100)
  target_s_high     HSV sat upper bound  0-255  (default 255)
  target_v_low      HSV val lower bound  0-255  (default 100)
  target_v_high     HSV val upper bound  0-255  (default 255)
  min_contour_area  Minimum blob area in px²    (default 200)
  debug_view        Show OpenCV debug window     (default False)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class TargetCV(Node):
    def __init__(self) -> None:
        super().__init__("target_cv")

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter("image_topic", "camera/image")
        self.declare_parameter("target_h_low", 0)
        self.declare_parameter("target_h_high", 10)
        self.declare_parameter("target_s_low", 100)
        self.declare_parameter("target_s_high", 255)
        self.declare_parameter("target_v_low", 100)
        self.declare_parameter("target_v_high", 255)
        self.declare_parameter("min_contour_area", 200)
        self.declare_parameter("debug_view", False)

        image_topic = (
            self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
        )

        # ── Subscriber ───────────────────────────────────────────
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )

        # ── Publisher ────────────────────────────────────────────
        self._detection_pub = self.create_publisher(
            PointStamped, "/payload_drop/target_detection", 10
        )

        self._bridge = CvBridge()

        self.get_logger().info(
            f"TargetCV node started — listening on '{image_topic}'"
        )

    # ==================================================================
    #  Image callback
    # ==================================================================

    def _on_image(self, msg: Image) -> None:
        """Process each camera frame and look for the target."""

        # ── Convert ROS Image → OpenCV BGR ────────────────────────
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        # ── Read HSV bounds from params ──────────────────────────
        h_lo = self.get_parameter("target_h_low").get_parameter_value().integer_value
        h_hi = self.get_parameter("target_h_high").get_parameter_value().integer_value
        s_lo = self.get_parameter("target_s_low").get_parameter_value().integer_value
        s_hi = self.get_parameter("target_s_high").get_parameter_value().integer_value
        v_lo = self.get_parameter("target_v_low").get_parameter_value().integer_value
        v_hi = self.get_parameter("target_v_high").get_parameter_value().integer_value
        min_area = self.get_parameter("min_contour_area").get_parameter_value().integer_value

        # ── HSV threshold ────────────────────────────────────────
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([h_lo, s_lo, v_lo]),
            np.array([h_hi, s_hi, v_hi]),
        )

        # Clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ── Find largest contour ─────────────────────────────────
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < min_area:
            return

        # ── Centroid via moments ─────────────────────────────────
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]

        # ── Publish detection ────────────────────────────────────
        det = PointStamped()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = "camera"
        det.point.x = cx
        det.point.y = cy
        det.point.z = area  # stash blob area in z for debugging
        self._detection_pub.publish(det)

        # ── Optional debug window ────────────────────────────────
        if self.get_parameter("debug_view").get_parameter_value().bool_value:
            cv2.circle(frame, (int(cx), int(cy)), 8, (0, 255, 0), 2)
            cv2.imshow("TargetCV", frame)
            cv2.waitKey(1)


# ======================================================================
#  Entry point
# ======================================================================


def main() -> None:
    rclpy.init()
    node = TargetCV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
