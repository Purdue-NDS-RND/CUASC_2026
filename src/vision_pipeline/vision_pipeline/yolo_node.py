"""
YOLO Inference Node with Image Slicing

Subscribes to raw camera images, slices them, runs YOLO inference
(via TensorRT engine), applies NMS to remove duplicates, and
publishes the bounding boxes to the drone control pipeline.
"""

import os
import time

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from torchvision.ops import nms
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_node")

        self.declare_parameter("model_path", "yolo26n_v1.0.engine")
        self.declare_parameter("conf_threshold", 0.50)
        self.declare_parameter("iou_threshold", 0.50)
        self.declare_parameter("slice_size", 1280)
        self.declare_parameter("overlap_ratio", 0.2)
        self.declare_parameter("publish_debug_image", True)

        model_filename = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        package_share_dir = get_package_share_directory("vision_pipeline")
        full_model_path = os.path.join(package_share_dir, "models", model_filename)

        self.get_logger().info(f"🧠 Loading YOLO model: {full_model_path}")
        self._model = YOLO(full_model_path, task="detect")
        self.get_logger().info("✅ Model loaded successfully!")

        self._cv_bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Image, "/camera/image_raw", self._image_callback, qos_profile
        )

        self._detection_pub = self.create_publisher(
            Detection2DArray, "/drone_control/detection", 10
        )
        self._debug_img_pub = self.create_publisher(
            Image, "/vision_pipeline/debug_image", 10
        )

        # ------------------------------------------------------------------
        # Verbose diagnostics state
        # ------------------------------------------------------------------
        self._frames_received = 0
        self._frames_with_hits = 0
        self._total_detections = 0
        self._last_hz_time = time.time()
        self._frames_since_hz = 0

        self.get_logger().info(
            f"🔍 YoloNode ready.\n"
            f"   conf_threshold : {self.get_parameter('conf_threshold').get_parameter_value().double_value}\n"
            f"   iou_threshold  : {self.get_parameter('iou_threshold').get_parameter_value().double_value}\n"
            f"   slice_size     : {self.get_parameter('slice_size').get_parameter_value().integer_value}\n"
            f"   overlap_ratio  : {self.get_parameter('overlap_ratio').get_parameter_value().double_value}"
        )

    # ------------------------------------------------------------------
    # Image slicing
    # ------------------------------------------------------------------

    def _slice_image(self, image, slice_size, overlap_ratio):
        h, w = image.shape[:2]
        slices = []
        step = int(slice_size * (1 - overlap_ratio))

        for y in range(0, h, step):
            for x in range(0, w, step):
                y_end = min(y + slice_size, h)
                x_end = min(x + slice_size, w)
                slice_img = image[y:y_end, x:x_end]
                slices.append((slice_img, x, y))
                if x_end >= w:
                    break
            if y_end >= h:
                break
        return slices

    # ------------------------------------------------------------------
    # NMS
    # ------------------------------------------------------------------

    def _apply_nms(self, all_boxes, iou_threshold):
        if len(all_boxes) == 0:
            return []

        boxes_tensor = torch.tensor(
            [[b["x1"], b["y1"], b["x2"], b["y2"]] for b in all_boxes],
            dtype=torch.float32,
        )
        scores_tensor = torch.tensor(
            [b["conf"] for b in all_boxes], dtype=torch.float32
        )
        keep_indices = nms(boxes_tensor, scores_tensor, iou_threshold)
        return [all_boxes[i] for i in keep_indices.tolist()]

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        t_start = time.time()

        self._frames_received += 1
        self._frames_since_hz += 1

        frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        conf_thresh = (
            self.get_parameter("conf_threshold").get_parameter_value().double_value
        )
        iou_thresh = (
            self.get_parameter("iou_threshold").get_parameter_value().double_value
        )
        slice_size = (
            self.get_parameter("slice_size").get_parameter_value().integer_value
        )
        overlap = self.get_parameter("overlap_ratio").get_parameter_value().double_value

        slices = self._slice_image(frame, slice_size, overlap)
        all_boxes = []

        self.get_logger().info(
            f"[Frame {self._frames_received}] "
            f"🖼️  {w}x{h} → {len(slices)} slices "
            f"(slice={slice_size}, overlap={overlap})"
        )

        # Run inference on each slice and log per-slice results
        for slice_idx, (slice_img, offset_x, offset_y) in enumerate(slices):
            t_infer = time.time()
            results = self._model(slice_img, conf=conf_thresh, verbose=False)
            infer_ms = (time.time() - t_infer) * 1000

            n_raw = len(results[0].boxes)
            self.get_logger().info(
                f"   Slice [{slice_idx + 1:02d}/{len(slices):02d}] "
                f"offset=({offset_x},{offset_y}) "
                f"size={slice_img.shape[1]}x{slice_img.shape[0]} "
                f"→ {n_raw} raw detections ({infer_ms:.1f} ms)"
            )

            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                all_boxes.append(
                    {
                        "x1": float(x1) + offset_x,
                        "y1": float(y1) + offset_y,
                        "x2": float(x2) + offset_x,
                        "y2": float(y2) + offset_y,
                        "conf": conf,
                        "cls": cls,
                    }
                )
                self.get_logger().info(
                    f"      ↳ raw box: class={cls} conf={conf:.3f} "
                    f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}) "
                    f"full-img=({float(x1) + offset_x:.0f},"
                    f"{float(y1) + offset_y:.0f},"
                    f"{float(x2) + offset_x:.0f},"
                    f"{float(y2) + offset_y:.0f})"
                )

        # NMS across all slices
        final_boxes = self._apply_nms(all_boxes, iou_thresh)
        t_total_ms = (time.time() - t_start) * 1000

        if len(all_boxes) > 0:
            self.get_logger().info(
                f"   NMS: {len(all_boxes)} raw → {len(final_boxes)} after "
                f"(iou_thresh={iou_thresh})"
            )

        if len(final_boxes) == 0:
            self.get_logger().info(
                f"[Frame {self._frames_received}] ⏭️  No detections after NMS "
                f"(total time: {t_total_ms:.1f} ms)"
            )
        else:
            self._frames_with_hits += 1
            self._total_detections += len(final_boxes)
            for i, box in enumerate(final_boxes):
                self.get_logger().info(
                    f"[Frame {self._frames_received}] "
                    f"✅ Detection {i + 1}/{len(final_boxes)}: "
                    f"class={box['cls']} conf={box['conf']:.3f} "
                    f"center=({box['x1'] + (box['x2'] - box['x1']) / 2:.0f},"
                    f"{box['y1'] + (box['y2'] - box['y1']) / 2:.0f}) "
                    f"size={box['x2'] - box['x1']:.0f}x{box['y2'] - box['y1']:.0f}"
                )

        # Pack and publish Detection2DArray
        det_array_msg = Detection2DArray()
        det_array_msg.header.stamp = msg.header.stamp
        det_array_msg.header.frame_id = msg.header.frame_id

        for box in final_boxes:
            det_msg = Detection2D()
            width = box["x2"] - box["x1"]
            height = box["y2"] - box["y1"]
            det_msg.bbox.center.position.x = box["x1"] + (width / 2.0)
            det_msg.bbox.center.position.y = box["y1"] + (height / 2.0)
            det_msg.bbox.size_x = width
            det_msg.bbox.size_y = height

            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = str(box["cls"])
            result.hypothesis.score = box["conf"]
            det_msg.results.append(result)
            det_array_msg.detections.append(det_msg)

        self._detection_pub.publish(det_array_msg)

        # Log actual inference rate every 10 frames
        if self._frames_received % 10 == 0:
            elapsed = time.time() - self._last_hz_time
            actual_hz = self._frames_since_hz / elapsed if elapsed > 0 else 0
            self.get_logger().info(
                f"📊 YoloNode stats — "
                f"frames received: {self._frames_received} | "
                f"frames with detections: {self._frames_with_hits} | "
                f"total detections: {self._total_detections} | "
                f"actual Hz: {actual_hz:.1f}"
            )
            self._frames_since_hz = 0
            self._last_hz_time = time.time()

        # Optional debug image
        if self.get_parameter("publish_debug_image").get_parameter_value().bool_value:
            debug_frame = frame.copy()
            for box in final_boxes:
                cv2.rectangle(
                    debug_frame,
                    (int(box["x1"]), int(box["y1"])),
                    (int(box["x2"]), int(box["y2"])),
                    (0, 255, 0),
                    4,
                )
                label = f"Class {box['cls']}: {box['conf']:.2f}"
                cv2.putText(
                    debug_frame,
                    label,
                    (int(box["x1"]), int(box["y1"]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                )
            debug_msg = self._cv_bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            self._debug_img_pub.publish(debug_msg)


def main() -> None:
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
