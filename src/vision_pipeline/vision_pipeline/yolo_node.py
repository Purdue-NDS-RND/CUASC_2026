"""
YOLO Inference Node with Image Slicing

Subscribes to raw camera images, slices them, runs YOLO inference
(via TensorRT engine), applies NMS to remove duplicates, and
publishes the bounding boxes to the drone control pipeline.
"""

import os

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from torchvision.ops import nms
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_node")

        # Parameters for inference and slicing
        self.declare_parameter("model_path", "yolo26n_v1.0.engine")
        self.declare_parameter("conf_threshold", 0.50)
        self.declare_parameter("iou_threshold", 0.50)
        self.declare_parameter("slice_size", 1280)
        self.declare_parameter("overlap_ratio", 0.2)
        self.declare_parameter("publish_debug_image", True)

        # Resolve the full path to the model inside the package's share directory
        model_filename = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        package_share_dir = get_package_share_directory("vision_pipeline")
        full_model_path = os.path.join(package_share_dir, "models", model_filename)

        # Load the YOLO model ONCE using the resolved path
        self.get_logger().info(f"Loading YOLO model from: {full_model_path} ...")
        self._model = YOLO(full_model_path, task="detect")
        self.get_logger().info("✅ Model loaded successfully!")

        self._cv_bridge = CvBridge()

        # ---------------------------------------------------------
        # SUBSCRIBER: Listens for images from ImageGrabber
        # ---------------------------------------------------------
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self._image_callback,
            10,
        )

        # ---------------------------------------------------------
        # PUBLISHERS
        # BUG FIX: was declared as Detection2D (singular) but we
        # publish a Detection2DArray. Mismatched types silently
        # drop all messages at the subscriber end.
        # ---------------------------------------------------------
        self._detection_pub = self.create_publisher(
            Detection2DArray, "/drone_control/detection", 10
        )

        # Optional: draw bounding boxes for viewing in RViz / rqt_image_view
        self._debug_img_pub = self.create_publisher(
            Image, "/vision_pipeline/debug_image", 10
        )

    # ------------------------------------------------------------------
    # Image slicing
    # ------------------------------------------------------------------

    def _slice_image(self, image, slice_size, overlap_ratio):
        """Slice image into overlapping tiles so small objects near tile
        borders are not missed."""
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
    # Non-Maximum Suppression
    # ------------------------------------------------------------------

    def _apply_nms(self, all_boxes, iou_threshold):
        """Remove duplicate detections that span tile boundaries."""
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
        """Runs every time a new image arrives from the camera."""

        frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Read parameters dynamically so they can be tuned at runtime
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

        # Slice → infer → collect raw detections
        slices = self._slice_image(frame, slice_size, overlap)
        all_boxes = []

        for slice_img, offset_x, offset_y in slices:
            results = self._model(slice_img, conf=conf_thresh, verbose=False)

            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                all_boxes.append(
                    {
                        "x1": float(x1) + offset_x,
                        "y1": float(y1) + offset_y,
                        "x2": float(x2) + offset_x,
                        "y2": float(y2) + offset_y,
                        "conf": float(box.conf[0].cpu().numpy()),
                        "cls": int(box.cls[0].cpu().numpy()),
                    }
                )

        # Remove cross-tile duplicates
        final_boxes = self._apply_nms(all_boxes, iou_thresh)

        # Pack into a Detection2DArray and publish once per frame
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

        # Optional live debug view
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
