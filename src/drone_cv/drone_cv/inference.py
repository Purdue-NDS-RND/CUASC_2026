"""
Inference Node

UPDATE EVERYTHING BELOW IN THIS DOC COMMENT
Publishes fake target detections (vision_msgs/Detection2D) to test
the localizer and visualizer without needing a real camera or CV pipeline.

Spawns random targets and publishes detections with realistic noise.

Detection2D format:
  - bbox.center.position.x/y: pixel center (u, v)
  - bbox.size_x/y: bounding box width/height in pixels
  - results[0].hypothesis.class_id: target ID
  - results[0].hypothesis.score: confidence

Usage:
  ros2 run drone_control detection_simulator
  ros2 run drone_control detection_simulator --ros-args -p num_targets:=5
"""

import math
import random
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose


class Inference(Node):
    def __init__(self) -> None:
        super().__init__("inference")

        # Parameters
        self.declare_parameter("num_targets", 3)
        self.declare_parameter("detection_rate_hz", 5.0)
        self.declare_parameter("spawn_radius_m", 50.0)
        self.declare_parameter("detection_noise_pixels", 20.0)
        self.declare_parameter("miss_probability", 0.1)  # Chance to miss a detection
        self.declare_parameter("false_positive_probability", 0.05)
        self.declare_parameter("target_size_m", 1.0)  # Simulated target size in meters

        # Camera params (should match localizer)
        self.declare_parameter("camera_fx", 424.0)
        self.declare_parameter("camera_fy", 424.0)
        self.declare_parameter("camera_cx", 640.0)
        self.declare_parameter("camera_cy", 360.0)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)

        # State
        self._drone_pose: Optional[PoseStamped] = None
        self._targets: List[Tuple[str, float, float]] = []  # (id, x, y)

        # Generate random targets
        num_targets = (
            self.get_parameter("num_targets").get_parameter_value().integer_value
        )
        spawn_radius = (
            self.get_parameter("spawn_radius_m").get_parameter_value().double_value
        )

        for i in range(num_targets):
            angle = random.uniform(0, 2 * math.pi)
            dist = random.uniform(10, spawn_radius)
            x = dist * math.cos(angle)
            y = dist * math.sin(angle)
            self._targets.append((str(i + 1), x, y))
            self.get_logger().info(f"Target {i + 1} at ({x:.1f}, {y:.1f})")

        # Publisher
        self._detection_pub = self.create_publisher(
            Detection2D, "/drone_control/detection", 10
        )

        # Subscriber
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )

        # Timer
        rate = (
            self.get_parameter("detection_rate_hz").get_parameter_value().double_value
        )
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        self.get_logger().info(f"Detection simulator ready with {num_targets} targets")

    def _on_pose(self, msg: PoseStamped) -> None:
        self._drone_pose = msg

    def _create_detection(
        self,
        target_id: str,
        u: float,
        v: float,
        width: float,
        height: float,
        confidence: float,
    ) -> Detection2D:
        """Create a Detection2D message."""
        msg = Detection2D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"

        # Bounding box
        msg.bbox.center.position.x = u
        msg.bbox.center.position.y = v
        msg.bbox.size_x = width
        msg.bbox.size_y = height

        # Result with class ID and confidence
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = target_id
        result.hypothesis.score = confidence
        msg.results.append(result)

        return msg

    def _on_timer(self) -> None:
        if self._drone_pose is None:
            return

        drone_x = self._drone_pose.pose.position.x
        drone_y = self._drone_pose.pose.position.y
        drone_z = self._drone_pose.pose.position.z

        if drone_z < 5.0:
            # Not high enough to see targets
            return

        # Get camera params
        fx = self.get_parameter("camera_fx").get_parameter_value().double_value
        fy = self.get_parameter("camera_fy").get_parameter_value().double_value
        cx = self.get_parameter("camera_cx").get_parameter_value().double_value
        cy = self.get_parameter("camera_cy").get_parameter_value().double_value
        img_w = self.get_parameter("image_width").get_parameter_value().integer_value
        img_h = self.get_parameter("image_height").get_parameter_value().integer_value
        noise_px = (
            self.get_parameter("detection_noise_pixels")
            .get_parameter_value()
            .double_value
        )
        miss_prob = (
            self.get_parameter("miss_probability").get_parameter_value().double_value
        )
        target_size_m = (
            self.get_parameter("target_size_m").get_parameter_value().double_value
        )

        for target_id, tx, ty in self._targets:
            # Random miss
            if random.random() < miss_prob:
                continue

            # Vector from drone to target (in local frame)
            dx = tx - drone_x
            dy = ty - drone_y
            dz = -drone_z  # Target is at ground level (z=0)

            # Simple pinhole projection (assuming camera points straight down)
            if dz >= 0:
                continue  # Target above drone

            # Project to image plane
            # For downward camera: image X = world Y, image Y = world X (roughly)
            u = cx + fx * (dy / (-dz))
            v = cy + fy * (dx / (-dz))

            # Calculate bounding box size based on distance
            # bbox_size_px = (target_size_m * focal_length) / distance
            bbox_width = (target_size_m * fx) / (-dz)
            bbox_height = (target_size_m * fy) / (-dz)

            # Add some variation to bbox
            bbox_width *= random.uniform(0.9, 1.1)
            bbox_height *= random.uniform(0.9, 1.1)

            # Add noise to center
            u += random.gauss(0, noise_px)
            v += random.gauss(0, noise_px)

            # Check if in frame (with some margin for bbox)
            margin = max(bbox_width, bbox_height) / 2
            if margin <= u < img_w - margin and margin <= v < img_h - margin:
                # Calculate confidence based on distance
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                confidence = max(0.3, min(1.0, 1.0 - dist / 100.0))

                # Publish detection
                det = self._create_detection(
                    target_id, u, v, bbox_width, bbox_height, confidence
                )
                self._detection_pub.publish(det)

        # Occasional false positive
        fp_prob = (
            self.get_parameter("false_positive_probability")
            .get_parameter_value()
            .double_value
        )
        if random.random() < fp_prob:
            u = random.uniform(50, img_w - 50)
            v = random.uniform(50, img_h - 50)
            width = random.uniform(20, 80)
            height = random.uniform(20, 80)
            det = self._create_detection(
                "noise", u, v, width, height, random.uniform(0.2, 0.5)
            )
            self._detection_pub.publish(det)


def main() -> None:
    rclpy.init()
    node = Inference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
