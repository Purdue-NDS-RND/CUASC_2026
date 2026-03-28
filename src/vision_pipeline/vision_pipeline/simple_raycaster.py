"""
Simple Monocular Raycaster (Level-Flight Assumption)
"""

import image_geometry
import rclpy
from geometry_msgs.msg import (
    Point,
    PointStamped,  # Better for world coordinates as it holds a timestamp
)
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2D


class SimpleRaycaster(Node):
    def __init__(self):
        super().__init__("simple_raycaster")

        # Flight Parameters
        self.declare_parameter("altitude_meters", 10.0)

        # The Mathematical Model
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False

        # Subscribers
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.info_callback, 10
        )

        self.pixel_sub = self.create_subscription(
            Detection2D,  # <--- Changed to Detection2D
            "/drone_control/detection",
            self.pixel_callback,
            10,
        )

        # Change the publisher to PointStamped so we can attach a frame_id
        self.world_pub = self.create_publisher(PointStamped, "/vision/target_world", 10)

        self.get_logger().info(
            "Simple Raycaster Initialized. Waiting for Arducam matrices..."
        )

    def info_callback(self, msg):
        # We only need to load the matrices once
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info(
                "✅ Intrinsic Matrices Loaded from /camera/camera_info"
            )

    def pixel_callback(self, msg: Detection2D):
        if not self.camera_info_received:
            self.get_logger().warn(
                "Discarding target: Camera matrices not yet received."
            )
            return

        # 1. Extract the 2D pixel and class data from Detection2D
        u = msg.bbox.center.position.x
        v = msg.bbox.center.position.y

        # Safely extract the class_id if it exists
        target_class = "unknown"
        if len(msg.results) > 0:
            target_class = msg.results[0].hypothesis.class_id

        # 2. Invert the Intrinsic Matrix
        ray = self.camera_model.projectPixelTo3dRay((u, v))

        # 3. Intersect with the Ground Plane
        altitude = self.get_parameter("altitude_meters").value
        scalar = altitude / ray[2]
        ground_x = ray[0] * scalar
        ground_y = ray[1] * scalar

        # 4. Publish the final 3D world coordinate using PointStamped
        target = PointStamped()
        target.header.stamp = (
            msg.header.stamp
        )  # Sync the timestamp with the camera frame
        target.header.frame_id = "camera_link"

        target.point.x = ground_x
        target.point.y = ground_y
        target.point.z = altitude

        self.world_pub.publish(target)
        self.get_logger().info(
            f"Target [{target_class}] at Pixel ({u:.1f}, {v:.1f}) -> Ground Pos: X={ground_x:.2f}m, Y={ground_y:.2f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRaycaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
