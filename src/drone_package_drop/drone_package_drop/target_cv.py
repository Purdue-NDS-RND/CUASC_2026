"""
Target CV Node — Detect a coloured target from a downward-facing camera.

Subscribes to:
  camera/image              (sensor_msgs/Image)   — raw camera frame

Publishes:
  /drone_package_drop/target_detection  (PointStamped)  — pixel (x, y) of target centre

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
import threading
import queue

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped


class TargetCV(Node):
    def __init__(self) -> None:
        super().__init__("target_cv")

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter("image_topic", "camera/image")
        self.declare_parameter("debug_view", False)

        image_topic = (
            self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
        )
        self.last_detection = None  # Cache last detection for timer callback status printout

        # ── Subscriber ───────────────────────────────────────────
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )

        # ── Publishers ───────────────────────────────────────
        self._detection_pub = self.create_publisher(
            PointStamped, "/drone_package_drop/target_detection", 10
        )
        # Publishes (width, height, 0) so consumers never need a parameter
        self._image_size_pub = self.create_publisher(
            PointStamped, "/drone_package_drop/image_size", 10
        )
        self._image_size: tuple[int, int] | None = None  # (w, h) cached from stream
        
        # ── Debug display thread  ────────────────────────────────
        self._display_queue = queue.Queue(maxsize=1)  # Keep only the latest frame
        self._display_thread = None
        self._display_running = False
        
        if self.get_parameter("debug_view").get_parameter_value().bool_value:
            self._display_running = True
            self._display_thread = threading.Thread(
                target=self._display_worker, daemon=True
            )
            self._display_thread.start()

        self.get_logger().info(
            f"TargetCV node started — listening on '{image_topic}'"
        )

        self.timer = self.create_timer(1.0, self._timer_callback)   

    def _timer_callback(self):
        # Print current detection status like cx cy and area

        if self.last_detection:
            cx, cy, area = self.last_detection
            self.get_logger().info(f"Current detection: cx={cx}, cy={cy}, area={area}")
        else:
            self.get_logger().info("No target detected.")

    def _imgmsg_to_cv2(self, msg: Image) -> np.ndarray:
        """Convert ROS Image message to OpenCV image without cv_bridge."""
        data = np.frombuffer(msg.data, dtype=np.uint8)
        
        if msg.encoding == "rgb8":
            frame = data.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "bgr8":
            return data.reshape((msg.height, msg.width, 3))
        elif msg.encoding == "mono8":
            return data.reshape((msg.height, msg.width))
        elif msg.encoding == "rgba8":
            frame = data.reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        else:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

    def _detect_target_center_moments(self, image):
        annotated = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # HSV thresholds for red
        lower_red1 = np.array([0,70,50])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([170,70,50])
        upper_red2 = np.array([180,255,255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)


        # clean mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)

        # compute moments on the cleaned mask
        M = cv2.moments(clean)
        if M['m00'] == 0:  # no target detected
            return annotated, None, None

        cX = int(M['m10'] / M['m00'])
        cY = int(M['m01'] / M['m00'])
        area = M['m00']  # blob area (0th moment)

        center_pixel_bgr = tuple(int(v) for v in image[cY, cX])
        cv2.drawMarker(annotated, (cX, cY), (0,255,0),
                    markerType=cv2.MARKER_CROSS, markerSize=22, thickness=2)
        cv2.circle(annotated, (cX, cY), 5, (0,0,255), -1)

        return annotated, (cX, cY), area

    def _on_image(self, msg: Image) -> None:
        """Process each camera frame and look for the target."""

        # ── Publish image dims (once, or whenever they change) ──────────────
        if self._image_size != (msg.width, msg.height):
            self._image_size = (msg.width, msg.height)
            dims = PointStamped()
            dims.header.stamp = self.get_clock().now().to_msg()
            dims.header.frame_id = "camera"
            dims.point.x = float(msg.width)
            dims.point.y = float(msg.height)
            dims.point.z = 0.0
            self._image_size_pub.publish(dims)
            self.get_logger().info(f"Image size: {msg.width}x{msg.height}")

        # ── Convert ROS Image → OpenCV BGR ────────────────────────
        try:
            frame = self._imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")
            return


        annotated, center, area = self._detect_target_center_moments(frame)

        # ── Publish detection ────────────────────────────────────
        if center is not None:
            cx, cy = center
            det = PointStamped()
            det.header.stamp = self.get_clock().now().to_msg()
            det.header.frame_id = "camera"
            det.point.x = float(cx)
            det.point.y = float(cy)
            det.point.z = float(area) if area is not None else 0.0
            self._detection_pub.publish(det)
            self.last_detection = (cx, cy, area)

        # ── Optional debug window ────────────────────────────────
        if self._display_running:
            try:
                self._display_queue.put_nowait(annotated)
            except queue.Full:
                pass  # Queue full, skip this frame (latest one is already there)

    def _display_worker(self) -> None:
        """Runs in separate thread, displays frames from the queue."""
        while self._display_running:
            try:
                frame = self._display_queue.get(timeout=1.0)
                cv2.imshow("TargetCV", frame)
                cv2.waitKey(1)
            except queue.Empty:
                continue


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
        node._display_running = False  # Stop display thread
        if node._display_thread:
            node._display_thread.join(timeout=2.0)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
