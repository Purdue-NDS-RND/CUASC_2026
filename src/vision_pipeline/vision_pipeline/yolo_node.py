"""
YOLO Inference Node with Image Slicing

Subscribes to raw camera images, slices them, runs YOLO inference
(via TensorRT engine), applies NMS to remove duplicates, and
publishes the bounding boxes to the drone control pipeline.
"""

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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

        # 1. Get the filename from the parameter
        model_filename = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )

        # 2. Dynamically find the path to your package's "share" directory
        package_share_dir = get_package_share_directory("vision_pipeline")

        # 3. Combine them to point to the new 'models' folder
        full_model_path = os.path.join(package_share_dir, "models", model_filename)

        self.get_logger().info(f"Loading YOLO model: {full_model_path}...")
        self._model = YOLO(full_model_path, task="detect")

        self._cv_bridge = CvBridge()

        # Load the YOLO model (TensorRT engine)
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.get_logger().info(f"Loading YOLO model: {model_path}...")
        self._model = YOLO(model_path, task="detect")
        self.get_logger().info("✅ Model loaded successfully!")

        # ---------------------------------------------------------
        # THE SUBSCRIBER: Listens for images from your ImageGrabber
        # ---------------------------------------------------------
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self._image_callback,
            10,  # Queue size
        )

        # ---------------------------------------------------------
        # THE PUBLISHERS: Sends data to your teammate's Localizer
        # ---------------------------------------------------------
        # Note: We use the exact topic name the localizer is expecting!
        self._detection_pub = self.create_publisher(
            Detection2D, "/drone_control/detection", 10
        )

        # Optional: A topic to view the drawn bounding boxes in RViz/rqt
        self._debug_img_pub = self.create_publisher(
            Image, "/vision_pipeline/debug_image", 10
        )

    def _slice_image(self, image, slice_size, overlap_ratio):
        """Manually slice image into overlapping tiles"""
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

    def _apply_nms(self, all_boxes, iou_threshold):
        """Remove duplicate detections using PyTorch NMS"""
        if len(all_boxes) == 0:
            return []

        boxes_tensor = torch.tensor(
            [[box["x1"], box["y1"], box["x2"], box["y2"]] for box in all_boxes],
            dtype=torch.float32,
        )
        scores_tensor = torch.tensor(
            [box["conf"] for box in all_boxes], dtype=torch.float32
        )

        keep_indices = nms(boxes_tensor, scores_tensor, iou_threshold)
        return [all_boxes[i] for i in keep_indices.tolist()]

    def _image_callback(self, msg: Image) -> None:
        """This function runs EVERY TIME a new image arrives from the camera."""

        # 1. Convert ROS Image message back to OpenCV NumPy array
        frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Fetch dynamic parameters
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

        # 2. Slice the image
        slices = self._slice_image(frame, slice_size, overlap)
        all_boxes = []

        # 3. Run Inference on each slice
        for slice_img, offset_x, offset_y in slices:
            results = self._model(slice_img, conf=conf_thresh, verbose=False)

            for box in results[0].boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                all_boxes.append(
                    {
                        "x1": x1 + offset_x,
                        "y1": y1 + offset_y,
                        "x2": x2 + offset_x,
                        "y2": y2 + offset_y,
                        "conf": float(box.conf[0].cpu().numpy()),
                        "cls": int(box.cls[0].cpu().numpy()),
                    }
                )

        # 4. Apply NMS
        final_boxes = self._apply_nms(all_boxes, iou_thresh)

        # 5. Convert to ROS Detection2DArray message
        det_array_msg = Detection2DArray()
        det_array_msg.header.stamp = msg.header.stamp
        det_array_msg.header.frame_id = msg.header.frame_id

        for box in final_boxes:
            det_msg = Detection2D()

            width = box["x2"] - box["x1"]
            height = box["y2"] - box["y1"]
            det_msg.bbox.center.position.x = float(box["x1"] + (width / 2.0))
            det_msg.bbox.center.position.y = float(box["y1"] + (height / 2.0))
            det_msg.bbox.size_x = float(width)
            det_msg.bbox.size_y = float(height)

            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = str(box["cls"])
            result.hypothesis.score = float(box["conf"])
            det_msg.results.append(result)

            det_array_msg.detections.append(det_msg)

        # Publish the array ONCE per frame
        self._detection_pub.publish(det_array_msg)

        # 6. (Optional) Draw boxes and publish debug image so you can view it live
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
