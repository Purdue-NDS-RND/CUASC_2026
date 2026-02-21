"""
Arducam Jetson Image Grabber Node

Captures frames from an Arducam using an nvarguscamerasrc GStreamer 
pipeline and publishes them as ROS 2 Image messages.
"""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageGrabber(Node):
    def __init__(self) -> None:
        super().__init__("image_grabber")

        # Parameters (These replace your old argparse setup)
        self.declare_parameter("image_width", 4656)
        self.declare_parameter("image_height", 3496)
        self.declare_parameter("fps", 9)
        self.declare_parameter("shutter_speed", 0) 
        self.declare_parameter("image_publishing_rate", 4.0) # Replaces your '-r' interval
        
        # 1. Fetch the parameters
        width = self.get_parameter("image_width").get_parameter_value().integer_value
        height = self.get_parameter("image_height").get_parameter_value().integer_value
        fps = self.get_parameter("fps").get_parameter_value().integer_value
        shutter = self.get_parameter("shutter_speed").get_parameter_value().integer_value

        # 2. Tools needed for image capture and conversion
        self._cv_bridge = CvBridge()
        
        # 3. Generate the GStreamer pipeline string
        pipeline = self._get_pipeline(width, height, fps, shutter)
        self.get_logger().info(f"Opening camera with pipeline:\n{pipeline}")

        # 4. Open the camera using the pipeline!
        self._camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not self._camera.isOpened():
            self.get_logger().error("❌ Failed to open Arducam via GStreamer!")
            raise RuntimeError("Camera failed to initialize.")

        # Publisher
        self._image_pub = self.create_publisher(
            Image, "/camera/image_raw", 10
        )

        # Timer
        rate = self.get_parameter("image_publishing_rate").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        self.get_logger().info(f"✅ Camera Ready. Publishing at {rate} Hz")

    def _get_pipeline(self, width, height, fps, shutter_speed_inv):
        """Generates the nvarguscamerasrc pipeline string."""
        if shutter_speed_inv > 0:
            exposure_ns = int(1000000000 / shutter_speed_inv)
            exp_str = f"exposuretimerange='{exposure_ns} {exposure_ns}'"
            gain_str = "gainrange='1.0 16.0'"
            self.get_logger().info(f"🔒 Locking Shutter to 1/{shutter_speed_inv}s ({exposure_ns} ns)")
        else:
            exp_str = ""
            gain_str = ""
            self.get_logger().info("🤖 Using Auto-Exposure (May cause blur)")

        return (
            f"nvarguscamerasrc sensor-id=0 {exp_str} {gain_str} ! "
            f"video/x-raw(memory:NVMM), width={width}, height={height}, "
            f"format=NV12, framerate={fps}/1 ! "
            f"nvvidconv ! "
            f"video/x-raw, format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw, format=BGR ! "
            f"appsink drop=1"
        )

    def _on_timer(self) -> None:
        """Triggered at the rate of image_publishing_rate"""
        success, frame = self._camera.read()

        if not success:
            self.get_logger().warn("Failed to capture image from camera!")
            return

        # Convert and Publish
        img_msg = self._cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "camera_link"
        self._image_pub.publish(img_msg)

def main() -> None:
    rclpy.init()
    node = ImageGrabber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 1. Release the camera hardware cleanly
        if hasattr(node, '_camera'):
            node._camera.release()

        # 2. Destroy the node
        node.destroy_node()

        # 3. Shutdown rclpy safely
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
