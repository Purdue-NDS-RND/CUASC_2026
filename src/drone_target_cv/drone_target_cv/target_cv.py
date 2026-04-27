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
from rclpy.qos import qos_profile_sensor_data
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
        self.declare_parameter("min_target_area_px", 25.0)
        self.declare_parameter("sim_hsv", True)
        self.declare_parameter("hsv_blur_kernel_px", 5)
        self.declare_parameter("morph_kernel_px", 5)
        self.declare_parameter("mask_blur_kernel_px", 0)
        self.declare_parameter("min_cluster_area_px", 0.0)
        self.declare_parameter("min_detection_confidence", 0.25)
        self.declare_parameter("use_light_normalization", True)
        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_tile_grid_size_px", 8)
        self.declare_parameter("hsv_red1_h_min", 0)
        self.declare_parameter("hsv_red1_h_max", 12)
        self.declare_parameter("hsv_red2_h_min", 168)
        self.declare_parameter("hsv_red2_h_max", 180)
        self.declare_parameter("hsv_s_min", -1)
        self.declare_parameter("hsv_s_max", 255)
        self.declare_parameter("hsv_v_min", -1)
        self.declare_parameter("hsv_v_max", 255)
        self.declare_parameter("red_dominance_ratio", 1.15)
        self.declare_parameter("red_difference_min", 15)
        self.declare_parameter("red_min_channel", 45)
        self.declare_parameter("cluster_kernel_px", 31)
        self.declare_parameter("cluster_dilate_iterations", 1)
        self.declare_parameter("radial_ray_count", 32)
        self.declare_parameter("radial_sample_count", 80)
        self.declare_parameter("bullseye_min_transitions", 3.0)

        image_topic_param = self.get_parameter(
            "image_topic"
        ).get_parameter_value()
        compressed_input_param = self.get_parameter(
            "compressed_input"
        ).get_parameter_value()
        target_area_param = self.get_parameter(
            "min_target_area_px"
        ).get_parameter_value()
        sim_hsv_param = self.get_parameter("sim_hsv").get_parameter_value()

        self._image_topic = image_topic_param.string_value
        self._compressed_input = compressed_input_param.bool_value
        self._min_target_area_px = target_area_param.double_value
        self._sim_hsv = sim_hsv_param.bool_value
        self._hsv_blur_kernel_px = self._odd_kernel_size(
            self.get_parameter("hsv_blur_kernel_px").value
        )
        self._morph_kernel_px = self._odd_kernel_size(
            self.get_parameter("morph_kernel_px").value
        )
        self._mask_blur_kernel_px = self._odd_kernel_size(
            self.get_parameter("mask_blur_kernel_px").value,
            allow_disabled=True,
        )
        hsv_s_min = self._default_hsv_param(
            "hsv_s_min",
            sim_default=70,
            live_default=70,
        )
        hsv_v_min = self._default_hsv_param(
            "hsv_v_min",
            sim_default=50,
            live_default=45,
        )
        self._detector_config = RedTargetDetectorConfig(
            min_target_area_px=self._min_target_area_px,
            min_cluster_area_px=self._get_float_parameter(
                "min_cluster_area_px"
            ),
            min_detection_confidence=self._get_float_parameter(
                "min_detection_confidence"
            ),
            hsv_blur_kernel_px=self._hsv_blur_kernel_px,
            morph_kernel_px=self._morph_kernel_px,
            mask_blur_kernel_px=self._mask_blur_kernel_px,
            use_light_normalization=self._get_bool_parameter(
                "use_light_normalization"
            ),
            clahe_clip_limit=self._get_float_parameter("clahe_clip_limit"),
            clahe_tile_grid_size_px=self._get_int_parameter(
                "clahe_tile_grid_size_px"
            ),
            hsv_red1_h_min=self._get_int_parameter("hsv_red1_h_min"),
            hsv_red1_h_max=self._get_int_parameter("hsv_red1_h_max"),
            hsv_red2_h_min=self._get_int_parameter("hsv_red2_h_min"),
            hsv_red2_h_max=self._get_int_parameter("hsv_red2_h_max"),
            hsv_s_min=hsv_s_min,
            hsv_s_max=self._get_int_parameter("hsv_s_max"),
            hsv_v_min=hsv_v_min,
            hsv_v_max=self._get_int_parameter("hsv_v_max"),
            red_dominance_ratio=self._get_float_parameter(
                "red_dominance_ratio"
            ),
            red_difference_min=self._get_int_parameter("red_difference_min"),
            red_min_channel=self._get_int_parameter("red_min_channel"),
            cluster_kernel_px=self._get_int_parameter("cluster_kernel_px"),
            cluster_dilate_iterations=self._get_int_parameter(
                "cluster_dilate_iterations"
            ),
            radial_ray_count=self._get_int_parameter("radial_ray_count"),
            radial_sample_count=self._get_int_parameter("radial_sample_count"),
            bullseye_min_transitions=self._get_float_parameter(
                "bullseye_min_transitions"
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
        self._debug_enabled = (
            self.get_parameter("debug_view").get_parameter_value().bool_value
        )
        self._annotated_pub = None
        self._mask_pub = None
        if self._debug_enabled:
            self._annotated_pub = self.create_publisher(
                Image, "/target_cv/annotated", 10
            )
            self._mask_pub = self.create_publisher(
                Image, "/target_cv/mask", 10
            )

        self._image_size: tuple[int, int] | None = None

        start_enabled = self.get_parameter(
            "start_enabled"
        ).get_parameter_value().bool_value
        self._set_processing_enabled(start_enabled)
        self.get_logger().info(
            "TargetCV node started - listening on "
            f"'{self._image_topic}' (compressed={self._compressed_input}, "
            f"sim_hsv={self._sim_hsv}, "
            f"hsv_blur_kernel_px={self._hsv_blur_kernel_px}, "
            f"morph_kernel_px={self._morph_kernel_px}, "
            f"mask_blur_kernel_px={self._mask_blur_kernel_px}, "
            f"hsv_s_min={self._detector_config.hsv_s_min}, "
            f"hsv_v_min={self._detector_config.hsv_v_min}, "
            f"cluster_kernel_px={self._detector_config.cluster_kernel_px}, "
            f"min_detection_confidence="
            f"{self._detector_config.min_detection_confidence})"
        )

        self.timer = self.create_timer(1.0, self._timer_callback)

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
                f"bullseye_score={bullseye_score:.2f}"
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

    def _get_bool_parameter(self, name: str) -> bool:
        return self.get_parameter(name).get_parameter_value().bool_value

    def _get_float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _get_int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _default_hsv_param(
        self,
        name: str,
        *,
        sim_default: int,
        live_default: int,
    ) -> int:
        value = self._get_int_parameter(name)
        if value >= 0:
            return value
        return sim_default if self._sim_hsv else live_default

    @staticmethod
    def _odd_kernel_size(value, *, allow_disabled: bool = False) -> int:
        size = int(value)
        if allow_disabled and size <= 1:
            return 0
        size = max(size, 1)
        if size % 2 == 0:
            size += 1
        return size

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
