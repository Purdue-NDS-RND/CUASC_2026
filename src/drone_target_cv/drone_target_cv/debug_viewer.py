import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class DebugViewer(Node):
    """
    Combines the raw camera, annotated view, and mask into a single window.
    """

    def __init__(self) -> None:
        super().__init__("debug_viewer")

        self.declare_parameter("compressed_input", False)
        self._compressed_input = self.get_parameter("compressed_input").value

        self._latest_camera: np.ndarray | None = None
        self._latest_annotated: np.ndarray | None = None
        self._latest_mask: np.ndarray | None = None

        if self._compressed_input:
            self._cam_sub = self.create_subscription(
                CompressedImage,
                "camera/image/compressed",
                self._on_camera_compressed,
                qos_profile_sensor_data,
            )
        else:
            self._cam_sub = self.create_subscription(
                Image,
                "camera/image",
                self._on_camera_raw,
                qos_profile_sensor_data,
            )

        self._ann_sub = self.create_subscription(
            Image,
            "/target_cv/annotated",
            self._on_annotated,
            qos_profile_sensor_data,
        )

        self._mask_sub = self.create_subscription(
            Image,
            "/target_cv/mask",
            self._on_mask,
            qos_profile_sensor_data,
        )

        # 30 Hz display timer
        self._timer = self.create_timer(1.0 / 30.0, self._on_timer)
        self.get_logger().info(
            f"Target CV Debug Viewer started (compressed={self._compressed_input}). "
            "Waiting for streams..."
        )

    def _on_camera_raw(self, msg: Image) -> None:
        self._latest_camera = self._imgmsg_to_cv2(msg)

    def _on_camera_compressed(self, msg: CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        self._latest_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def _on_annotated(self, msg: Image) -> None:
        self._latest_annotated = self._imgmsg_to_cv2(msg)

    def _on_mask(self, msg: Image) -> None:
        self._latest_mask = self._imgmsg_to_cv2(msg)

    def _imgmsg_to_cv2(self, msg: Image) -> np.ndarray:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            frame = data.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if msg.encoding == "bgr8":
            return data.reshape((msg.height, msg.width, 3))
        if msg.encoding == "mono8":
            frame = data.reshape((msg.height, msg.width))
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if msg.encoding == "rgba8":
            frame = data.reshape((msg.height, msg.width, 4))
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        
        # fallback
        return np.zeros((360, 640, 3), dtype=np.uint8)

    def _on_timer(self) -> None:
        frames = []
        target_h, target_w = 360, 640

        for frame in (self._latest_camera, self._latest_annotated, self._latest_mask):
            if frame is not None:
                resized = cv2.resize(frame, (target_w, target_h))
                frames.append(resized)
            else:
                frames.append(np.zeros((target_h, target_w, 3), dtype=np.uint8))

        # Add labels
        labels = ["1) Raw Feed", "2) Annotated CV", "3) Mask CV"]
        for f, label in zip(frames, labels):
            cv2.putText(
                f, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA
            )
            cv2.putText(
                f, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA
            )

        # combine horizontally to create a 3-pane dashboard
        canvas = np.hstack(frames)
        
        cv2.imshow("Dashboard: Target CV Debug", canvas)
        cv2.waitKey(1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DebugViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
