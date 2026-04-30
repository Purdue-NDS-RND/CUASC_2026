"""
Target CV Node - Detect a coloured target from a downward-facing camera.

Subscribes to:
  camera/image - raw sensor_msgs/Image frames
  camera/image/compressed - sensor_msgs/CompressedImage frames

Publishes:
  /drone_package_drop/target_detection - normalized target offsets
  /drone_package_drop/image_size - raw image width/height
  /target_cv/annotated - annotated debug stream when debug_view=True
  /target_cv/mask - cleaned red mask as mono8 when debug_view=True

The detection uses a red-pixel target cluster detector that supports both a
solid red practice circle and the official red/white bullseye target. Tune the
HSV and red-dominance parameters to match the current camera and light.

Parameters:
  image_topic       Camera topic name            (default "camera/image")
  compressed_input  Subscribe to CompressedImage instead of Image
  debug_view        Publish annotated and mask debug streams (default False)
"""

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage, Image
from std_srvs.srv import SetBool

from drone_target_cv.target_detector import (
    RedTargetDetector,
    RedTargetDetectorConfig,
    draw_debug_overlay,
)


class TargetCV(Node):
    def __init__(self) -> None:
        super().__init__("target_cv")

        self.declare_parameter("image_topic", "camera/image")
        self.declare_parameter("compressed_input", False)
        self.declare_parameter("debug_view", False)
        self.declare_parameter("start_enabled", True)
        self.declare_parameter("sim_hsv", True)

        self._image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self._compressed_input = (
            self.get_parameter("compressed_input").get_parameter_value().bool_value
        )
        self._debug_enabled = (
            self.get_parameter("debug_view").get_parameter_value().bool_value
        )
        start_enabled = self.get_parameter("start_enabled").get_parameter_value().bool_value
        self._sim_hsv = self.get_parameter("sim_hsv").get_parameter_value().bool_value

        hsv_v_min_default = 50 if self._sim_hsv else 45

        self._detector_config = RedTargetDetectorConfig(
            min_target_area_px=self._declare_float_parameter(
                "min_target_area_px",
                25.0,
            ),
            min_cluster_area_px=self._declare_float_parameter(
                "min_cluster_area_px",
                0.0,
            ),
            min_detection_confidence=self._declare_float_parameter(
                "min_detection_confidence",
                0.25,
            ),
            min_circularity=self._declare_float_parameter("min_circularity", 0.4),
            min_solid_score=self._declare_float_parameter("min_solid_score", 0.0),
            min_bullseye_score=self._declare_float_parameter(
                "min_bullseye_score",
                0.0,
            ),
            hsv_blur_kernel_px=self._declare_int_parameter("hsv_blur_kernel_px", 5),
            morph_kernel_px=self._declare_int_parameter("morph_kernel_px", 3),
            mask_blur_kernel_px=self._declare_int_parameter("mask_blur_kernel_px", 0),
            use_light_normalization=self._declare_bool_parameter(
                "use_light_normalization",
                True,
            ),
            clahe_clip_limit=self._declare_float_parameter("clahe_clip_limit", 2.0),
            clahe_tile_grid_size_px=self._declare_int_parameter(
                "clahe_tile_grid_size_px",
                8,
            ),
            hsv_red1_h_min=self._declare_int_parameter("hsv_red1_h_min", 0),
            hsv_red1_h_max=self._declare_int_parameter("hsv_red1_h_max", 8),
            hsv_red2_h_min=self._declare_int_parameter("hsv_red2_h_min", 168),
            hsv_red2_h_max=self._declare_int_parameter("hsv_red2_h_max", 180),
            hsv_s_min=self._declare_int_parameter("hsv_s_min", 80),
            hsv_s_max=self._declare_int_parameter("hsv_s_max", 255),
            hsv_v_min=self._declare_int_parameter("hsv_v_min", hsv_v_min_default),
            hsv_v_max=self._declare_int_parameter("hsv_v_max", 255),
            red_dominance_ratio=self._declare_float_parameter(
                "red_dominance_ratio",
                1.35,
            ),
            red_difference_min=self._declare_int_parameter("red_difference_min", 40),
            red_min_channel=self._declare_int_parameter("red_min_channel", 60),
            red_dominance_s_min=self._declare_int_parameter("red_dominance_s_min", 95),
            red_dominance_v_min=self._declare_int_parameter("red_dominance_v_min", 55),
            cluster_kernel_px=self._declare_int_parameter("cluster_kernel_px", 31),
            cluster_dilate_iterations=self._declare_int_parameter(
                "cluster_dilate_iterations",
                1,
            ),
            radial_ray_count=self._declare_int_parameter("radial_ray_count", 32),
            radial_sample_count=self._declare_int_parameter("radial_sample_count", 80),
            bullseye_min_transitions=self._declare_float_parameter(
                "bullseye_min_transitions",
                3.0,
            ),
        )
        self._detector = RedTargetDetector(self._detector_config)
        self._enabled = False
        self._image_sub = None
        self.last_detection = None
        self.last_result = None

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
        debug_image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._annotated_pub = None
        self._mask_pub = None
        if self._debug_enabled:
            self._annotated_pub = self.create_publisher(
                Image, "/target_cv/annotated", debug_image_qos
            )
            self._mask_pub = self.create_publisher(
                Image, "/target_cv/mask", debug_image_qos
            )

        self._image_size: tuple[int, int] | None = None

        self._set_processing_enabled(start_enabled)

        self.get_logger().info(
            "TargetCV node started - listening on "
            f"'{self._image_topic}' (compressed={self._compressed_input}, "
            f"sim_hsv={self._sim_hsv})"
        )
        self.get_logger().info(
            "TargetCV detector tuning: "
            f"min_target_area_px={self._detector_config.min_target_area_px}, "
            f"hsv_blur_kernel_px={self._detector_config.hsv_blur_kernel_px}, "
            f"morph_kernel_px={self._detector_config.morph_kernel_px}, "
            f"mask_blur_kernel_px={self._detector_config.mask_blur_kernel_px}, "
            f"min_circularity={self._detector_config.min_circularity}"
        )

        self.timer = self.create_timer(1.0, self._timer_callback)

    def _declare_bool_parameter(self, name: str, default: bool) -> bool:
        self.declare_parameter(name, default)
        return self.get_parameter(name).get_parameter_value().bool_value

    def _declare_float_parameter(self, name: str, default: float) -> float:
        self.declare_parameter(name, default)
        return float(self.get_parameter(name).value)

    def _declare_int_parameter(self, name: str, default: int) -> int:
        self.declare_parameter(name, default)
        return int(self.get_parameter(name).value)

    def _timer_callback(self):
        if not self._enabled:
            return
        if self.last_detection:
            x_norm, y_norm, area, confidence, solid_score, bullseye_score = (
                self.last_detection
            )
            self.get_logger().info(
                f"Current detection: x_norm={x_norm:.3f}, "
                f"y_norm={y_norm:.3f}, area={area:.0f}, "
                f"confidence={confidence:.2f}, solid_score={solid_score:.2f}, "
                f"bullseye_score={bullseye_score:.2f}, "
                f"circularity={self.last_result.circularity:.2f}"
            )
        else:
            if self.last_result is None:
                self.get_logger().info("No target detected.")
                return
            self.get_logger().info(
                "No target detected: "
                f"reason='{self.last_result.reject_reason}', "
                f"raw_red_area_px={self.last_result.raw_red_area_px}, "
                f"clean_red_area_px={self.last_result.clean_red_area_px}, "
                f"cluster_count={self.last_result.cluster_count}, "
                f"best_confidence={self.last_result.confidence:.2f}, "
                f"circularity={self.last_result.circularity:.2f}, "
                f"solid_score={self.last_result.solid_score:.2f}, "
                f"bullseye_score={self.last_result.bullseye_score:.2f}"
            )

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
            if self._compressed_input:
                callback = self._on_compressed_image
            else:
                callback = self._on_image
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
        self.last_result = None
        self._image_size = None
        self._publish_not_found_detection()
        self.get_logger().info("Target detection disabled")

    def _publish_not_found_detection(self) -> None:
        det = PointStamped()
        det.header.stamp = self.get_clock().now().to_msg()
        det.header.frame_id = "camera"
        det.point.x = -1.0
        det.point.y = -1.0
        det.point.z = 0.0
        self._detection_pub.publish(det)

    def _imgmsg_to_cv2(self, msg: Image) -> np.ndarray:
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

        result = self._detector.detect(frame)
        self.last_result = result
        annotated = draw_debug_overlay(frame, result)
        if result.accepted:
            clean_mask = result.selected_red_mask
        else:
            clean_mask = result.clean_red_mask

        if self._annotated_pub is not None:
            annotated = np.ascontiguousarray(annotated)
            self._annotated_pub.publish(
                Image(
                    header=header,
                    height=annotated.shape[0],
                    width=annotated.shape[1],
                    encoding="bgr8",
                    is_bigendian=0,
                    step=annotated.strides[0],
                    data=annotated.tobytes(),
                )
            )
        if self._mask_pub is not None:
            clean_mask = np.ascontiguousarray(clean_mask)
            self._mask_pub.publish(
                Image(
                    header=header,
                    height=clean_mask.shape[0],
                    width=clean_mask.shape[1],
                    encoding="mono8",
                    is_bigendian=0,
                    step=clean_mask.strides[0],
                    data=clean_mask.tobytes(),
                )
            )

        if result.center is not None:
            center_x, center_y = result.center
            x_norm = self._normalize_offset(center_x, width)
            y_norm = self._normalize_offset(center_y, height)
            det = PointStamped()
            det.header.stamp = self.get_clock().now().to_msg()
            det.header.frame_id = "camera"
            det.point.x = x_norm
            det.point.y = y_norm
            det.point.z = float(result.red_area_px)
            self._detection_pub.publish(det)
            self.last_detection = (
                x_norm,
                y_norm,
                result.red_area_px,
                result.confidence,
                result.solid_score,
                result.bullseye_score,
            )
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
