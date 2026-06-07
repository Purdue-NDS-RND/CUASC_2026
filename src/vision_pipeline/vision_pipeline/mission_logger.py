#!/usr/bin/env python3
"""
Unified YOLO Inference & Mission Logger Node (A/B Testing Edition)

Performs sliced YOLO inference via TensorRT, runs two raycasting algorithms
in parallel for every detection using direct frame transformations, deduplicates
targets, logs output data to persistent open-once CSV file streams, and
synchronously outputs raw frames and telemetry to a dedicated raycast session directory.

Takeoff altitude origin is treated as 0.0m (Relative AGL Frame).
"""

import csv
import math
import os
import time
from collections import deque
from datetime import datetime
from typing import List, Tuple

import cv2
import image_geometry
import numpy as np
import rclpy
import torch
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSDurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial.transform import Rotation as R_scipy
from scipy.spatial.transform import Slerp
from sensor_msgs.msg import CameraInfo, Image
from torchvision.ops import nms
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_mission_node")

        # ------------------------------------------------------------------
        # Parameters Configuration
        # ------------------------------------------------------------------
        self.declare_parameter("model_path", "yolo26n_v2.1.engine")
        self.declare_parameter("conf_threshold", 0.50)
        self.declare_parameter("iou_threshold", 0.50)
        self.declare_parameter("slice_size", 1280)
        self.declare_parameter("overlap_ratio", 0.2)
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("ground_altitude_m", 0.0)
        self.declare_parameter("mount_x", 0.0)
        self.declare_parameter("mount_y", 0.0)
        self.declare_parameter("mount_z", 0.0)
        self.declare_parameter("mount_roll", 0.0)
        self.declare_parameter("mount_pitch", 0.0)
        self.declare_parameter("mount_yaw", 0.0)

        # Extract parameters
        model_filename = (
            self.get_parameter("model_path").get_parameter_value().string_value
        )
        package_share_dir = get_package_share_directory("vision_pipeline")
        full_model_path = os.path.join(package_share_dir, "models", model_filename)

        self._mount_x = self.get_parameter("mount_x").get_parameter_value().double_value
        self._mount_y = self.get_parameter("mount_y").get_parameter_value().double_value
        self._mount_z = self.get_parameter("mount_z").get_parameter_value().double_value
        self._mount_roll = (
            self.get_parameter("mount_roll").get_parameter_value().double_value
        )
        self._mount_pitch = (
            self.get_parameter("mount_pitch").get_parameter_value().double_value
        )
        self._mount_yaw = (
            self.get_parameter("mount_yaw").get_parameter_value().double_value
        )

        # ------------------------------------------------------------------
        # Output Directories Setup (Mission Data + Raycast Sessions)
        # ------------------------------------------------------------------
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")

        # 1. Mission Logger Outputs
        self.save_dir = os.path.expanduser(
            f"~/CUASC_Mission_Data/Flight_{timestamp_str}"
        )
        self._frames_dir = os.path.join(self.save_dir, "frames")
        self._targets_dir = os.path.join(self.save_dir, "targets")
        os.makedirs(self._frames_dir, exist_ok=True)
        os.makedirs(self._targets_dir, exist_ok=True)

        self.csv_full_v1 = os.path.join(self.save_dir, "mission_log_full_v1.csv")
        self.csv_prime_v1 = os.path.join(self.save_dir, "mission_log_prime_v1.csv")
        self.csv_full_v2 = os.path.join(self.save_dir, "mission_log_full_v2.csv")
        self.csv_prime_v2 = os.path.join(self.save_dir, "mission_log_prime_v2.csv")

        # 2. Raycast GUI Sessions Outputs
        base_session_dir = os.path.expanduser("~/raycast_sessions")
        os.makedirs(base_session_dir, exist_ok=True)
        self.session_dir = os.path.join(
            base_session_dir, f"Raycast_Session_{timestamp_str}"
        )
        self.images_dir = os.path.join(self.session_dir, "raw_frames")
        os.makedirs(self.images_dir, exist_ok=True)

        self.csv_path = os.path.join(self.session_dir, "telemetry_metadata.csv")
        self.camera_info_path = os.path.join(self.session_dir, "camera_info.yaml")

        # ------------------------------------------------------------------
        # Persistent File Streams Initialization (Open Once Optimization)
        # ------------------------------------------------------------------
        self._init_persistent_csvs()

        # ------------------------------------------------------------------
        # YOLO Model Ingestion
        # ------------------------------------------------------------------
        self.get_logger().info(f"🧠 Loading YOLO model: {full_model_path}")
        self._model = YOLO(full_model_path, task="detect")
        self.get_logger().info("✅ Model loaded successfully!")

        # ------------------------------------------------------------------
        # Pipeline State Setup
        # ------------------------------------------------------------------
        self._frame_counter = 0
        self._target_counter = 1
        self._last_continuous_save_time = 0.0

        self.saved_target_locations_v1: List[Tuple[float, float]] = []
        self.saved_target_locations_v2: List[Tuple[float, float]] = []
        self.min_dist_m = 1.0

        self.cv_bridge = CvBridge()
        self.R_EARTH = 6378137.0
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False

        self._frames_received = 0
        self._frames_with_hits = 0
        self._total_detections = 0
        self._last_hz_time = time.time()
        self._frames_since_hz = 0
        self._last_pose_log_time = 0.0

        self._drone_pose = None
        self._pose_history = deque(maxlen=200)  # ~10s buffer at 20Hz
        self._home = None

        # ------------------------------------------------------------------
        # ROS 2 Communication Links
        # ------------------------------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_camera_info, qos_profile
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )

        home_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            HomePosition, "/mavros/home_position/home", self._on_home, home_qos
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

        self._readiness_timer = self.create_timer(3.0, self._log_readiness)

        self.get_logger().info("🚀 Unified YOLO Mission Node Ready and Online.")

    def _init_persistent_csvs(self):
        mission_headers = [
            "Image_Name",
            "Latitude",
            "Longitude",
            "Time_UTC",
            "YOLO_Confidence",
            "Pixel_U",
            "Pixel_V",
            "BBox_W",
            "BBox_H",
            "Is_Prime",
        ]

        self._f_full_v1 = open(self.csv_full_v1, mode="w", newline="")
        self._f_prime_v1 = open(self.csv_prime_v1, mode="w", newline="")
        self._f_full_v2 = open(self.csv_full_v2, mode="w", newline="")
        self._f_prime_v2 = open(self.csv_prime_v2, mode="w", newline="")

        self._writer_full_v1 = csv.writer(self._f_full_v1)
        self._writer_prime_v1 = csv.writer(self._f_prime_v1)
        self._writer_full_v2 = csv.writer(self._f_full_v2)
        self._writer_prime_v2 = csv.writer(self._f_prime_v2)

        for wr in [
            self._writer_full_v1,
            self._writer_prime_v1,
            self._writer_full_v2,
            self._writer_prime_v2,
        ]:
            wr.writerow(mission_headers)

        # Initialize the open-once persistent metadata log for offline GUI raycasting
        self._f_metadata = open(self.csv_path, mode="w", newline="")
        self._writer_metadata = csv.writer(self._f_metadata)

        gui_headers = [
            "filename",
            "stamp_sec",
            "stamp_nanosec",
            "drone_x",
            "drone_y",
            "drone_z",
            "qx",
            "qy",
            "qz",
            "qw",
            "home_lat",
            "home_lon",
            "home_alt",
        ]
        self._writer_metadata.writerow(gui_headers)
        self._f_metadata.flush()

    def _log_readiness(self):
        if (
            self.camera_info_received
            and self._drone_pose is not None
            and self._home is not None
        ):
            self.get_logger().info(
                "✅ All core systems operational — Raycasting engine is ACTIVE."
            )
            self._readiness_timer.cancel()
            return

        self.get_logger().warn(
            f"⏳ Awaiting system prerequisites:\n"
            f"   camera_info  : {'✅' if self.camera_info_received else '❌ missing topic /camera/camera_info'}\n"
            f"   drone_pose   : {'✅' if self._drone_pose is not None else '❌ missing topic /mavros/local_position/pose'}\n"
            f"   home_position: {'✅' if self._home is not None else '❌ missing topic /mavros/home_position/home (No GPS lock)'}"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info(
                f"✅ Intrinsic mapping loaded. fx={self.camera_model.fx():.2f} fy={self.camera_model.fy():.2f}"
            )

            # Atomic save of physical camera calibration specs for the GUI
            cam_data = {
                "image_width": msg.width,
                "image_height": msg.height,
                "camera_matrix": {"data": list(msg.k)},
                "distortion_coefficients": {"data": list(msg.d)},
                "projection_matrix": {"data": list(msg.p)},
            }

            temp_path = self.camera_info_path + ".tmp"
            try:
                with open(temp_path, "w") as f:
                    yaml.safe_dump(cam_data, f, default_flow_style=False)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(temp_path, self.camera_info_path)
                self.get_logger().info(
                    "📷 Camera calibration parameters saved atomically to session directory."
                )
            except Exception as e:
                self.get_logger().error(
                    f"Failed to atomically write camera calibration parameters: {e}"
                )
                # Reset flag so the next CameraInfo message triggers another attempt
                self.camera_info_received = False
                # Clean up the partial .tmp file so it doesn't block the next os.replace()
                try:
                    os.remove(temp_path)
                except OSError:
                    pass

    def _on_pose(self, msg: PoseStamped) -> None:
        if math.isnan(msg.pose.position.x) or math.isnan(msg.pose.position.z):
            return
        self._drone_pose = msg
        self._pose_history.append(msg)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_pose_log_time >= 1.0:
            p = msg.pose.position
            q = msg.pose.orientation
            r = R_scipy.from_quat([q.x, q.y, q.z, q.w])
            rpy = r.as_euler("xyz", degrees=True)
            self.get_logger().info(
                f"🛸 Pose -> pos: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m rpy: ({rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°)"
            )
            self._last_pose_log_time = now

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    def _slice_image(self, image, slice_size, overlap_ratio):
        h, w = image.shape[:2]
        slices = []
        step = int(slice_size * (1 - overlap_ratio))
        for y in range(0, h, step):
            for x in range(0, w, step):
                y_end = min(y + slice_size, h)
                x_end = min(x + slice_size, w)
                slices.append((image[y:y_end, x:x_end], x, y))
                if x_end >= w:
                    break
            if y_end >= h:
                break
        return slices

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

    def _get_pose_at_time_v1_nearest(self, target_stamp):
        if not self._pose_history:
            return None
        target_sec = target_stamp.sec + target_stamp.nanosec * 1e-9
        best_pose = min(
            self._pose_history,
            key=lambda p: abs(
                (p.header.stamp.sec + p.header.stamp.nanosec * 1e-9) - target_sec
            ),
        )
        best_sec = best_pose.header.stamp.sec + best_pose.header.stamp.nanosec * 1e-9
        delta_ms = abs(best_sec - target_sec) * 1000

        if delta_ms > 200:
            self.get_logger().warn(
                f"Best V1 pose delta is too high ({delta_ms:.0f}ms) -> discarding."
            )
            return None
        return best_pose

    def _get_pose_at_time_v2_interpolated(self, target_stamp):
        if len(self._pose_history) < 2:
            return None
        target_sec = target_stamp.sec + target_stamp.nanosec * 1e-9
        poses = sorted(
            self._pose_history,
            key=lambda p: p.header.stamp.sec + p.header.stamp.nanosec * 1e-9,
        )

        before, after = None, None
        for p in poses:
            p_sec = p.header.stamp.sec + p.header.stamp.nanosec * 1e-9
            if p_sec <= target_sec:
                before = p
            elif p_sec > target_sec and after is None:
                after = p
                break

        if before is None or after is None:
            return None
        t0 = before.header.stamp.sec + before.header.stamp.nanosec * 1e-9
        t1 = after.header.stamp.sec + after.header.stamp.nanosec * 1e-9

        if (t1 - t0) == 0 or (target_sec - t0) > 0.2 or (t1 - target_sec) > 0.2:
            return None

        alpha = (target_sec - t0) / (t1 - t0)
        interp_pose = PoseStamped()
        interp_pose.header.stamp = target_stamp

        # Position Linear Interpolation
        p0, p1 = before.pose.position, after.pose.position
        interp_pose.pose.position.x = p0.x + alpha * (p1.x - p0.x)
        interp_pose.pose.position.y = p0.y + alpha * (p1.y - p0.y)
        interp_pose.pose.position.z = p0.z + alpha * (p1.z - p0.z)

        # Orientation Spherical Linear Interpolation (SLERP)
        rot0 = [
            before.pose.orientation.x,
            before.pose.orientation.y,
            before.pose.orientation.z,
            before.pose.orientation.w,
        ]
        rot1 = [
            after.pose.orientation.x,
            after.pose.orientation.y,
            after.pose.orientation.z,
            after.pose.orientation.w,
        ]

        key_rots = R_scipy.from_quat([rot0, rot1])
        slerp = Slerp([t0, t1], key_rots)
        interp_rot = slerp([target_sec])[0].as_quat()

        interp_pose.pose.orientation.x = interp_rot[0]
        interp_pose.pose.orientation.y = interp_rot[1]
        interp_pose.pose.orientation.z = interp_rot[2]
        interp_pose.pose.orientation.w = interp_rot[3]

        return interp_pose

    def _image_callback(self, msg: Image) -> None:
        t_start = time.time()
        self._frames_received += 1
        self._frames_since_hz += 1

        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = frame.shape[:2]

        # ------------------------------------------------------------------
        # Raycast GUI Session Logger Integration (Synchronized background tracking)
        # ------------------------------------------------------------------
        if self._drone_pose is not None and self._home is not None:
            raw_frame_name = f"frame_{self._frames_received:06d}.jpg"
            raw_filepath = os.path.join(self.images_dir, raw_frame_name)

            try:
                # Save the raw unannotated image frame
                cv2.imwrite(raw_filepath, frame)

                pose_for_log = self._get_pose_at_time_v2_interpolated(msg.header.stamp)

                if pose_for_log is None:
                    pose_for_log = self._get_pose_at_time_v1_nearest(msg.header.stamp)

                if pose_for_log is not None:
                    pos = pose_for_log.pose.position
                    ori = pose_for_log.pose.orientation

                    row_meta = [
                        raw_frame_name,
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec,
                        pos.x,
                        pos.y,
                        pos.z,
                        ori.x,
                        ori.y,
                        ori.z,
                        ori.w,
                        self._home.geo.latitude,
                        self._home.geo.longitude,
                        self._home.geo.altitude,
                    ]

                    self._writer_metadata.writerow(row_meta)
                    self._f_metadata.flush()
            except Exception as e:
                self.get_logger().error(
                    f"Failed to log background raw session data: {e}"
                )

        # Environmental background logging pipeline (1Hz)
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self._last_continuous_save_time) >= 1.0:
            self._frame_counter += 1
            raw_filename = f"frame_{self._frame_counter:06d}_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg"
            cv2.imwrite(os.path.join(self._frames_dir, raw_filename), frame)
            self._last_continuous_save_time = current_time

        # Validate pipeline requirements before executing vision-spatial logs
        if (
            not self.camera_info_received
            or self._drone_pose is None
            or self._home is None
        ):
            return

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

        # Sliced Model Inference
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

        final_boxes = self._apply_nms(all_boxes, iou_thresh)
        t_total_ms = (time.time() - t_start) * 1000

        if len(final_boxes) == 0:
            return

        self._frames_with_hits += 1
        self._total_detections += len(final_boxes)

        # Define Center 60% Bounding Constraints
        margin_x, margin_y = w * 0.20, h * 0.20

        # Synchronize exact A/B poses matching frame shutter timestamp
        pose_v1 = self._get_pose_at_time_v1_nearest(msg.header.stamp)
        pose_v2 = self._get_pose_at_time_v2_interpolated(msg.header.stamp)

        targets_logged_this_frame = False
        annotated_frame = frame.copy()

        for det_idx, box in enumerate(final_boxes):
            u = box["x1"] + (box["x2"] - box["x1"]) / 2.0
            v = box["y1"] + (box["y2"] - box["y1"]) / 2.0
            bbox_w = box["x2"] - box["x1"]
            bbox_h = box["y2"] - box["y1"]

            is_prime = margin_x < u < (w - margin_x) and margin_y < v < (h - margin_y)
            zone_label = "PRIME" if is_prime else "EDGE"

            gps_v1 = self._raycast_to_gps(u, v, pose_v1) if pose_v1 else None
            gps_v2 = self._raycast_to_gps(u, v, pose_v2) if pose_v2 else None

            if gps_v1 is None and gps_v2 is None:
                continue

            # Unique filename tracking prevents multi-detection overwrites
            image_filename = f"target_{self._target_counter:03d}_det_{det_idx:03d}.jpg"
            time_utc = datetime.utcnow().strftime("%H:%M:%S")

            # Evaluate and Log V1 (Legacy Nearest) Data Streams
            if gps_v1:
                lat_v1, lon_v1 = gps_v1
                is_duplicate_v1 = any(
                    self._calculate_distance_m(lat_v1, lon_v1, l, ln) < self.min_dist_m
                    for l, ln in self.saved_target_locations_v1
                )
                if not is_duplicate_v1:
                    row_v1 = [
                        image_filename,
                        f"{lat_v1:.7f}",
                        f"{lon_v1:.7f}",
                        time_utc,
                        f"{box['conf']:.2f}",
                        int(u),
                        int(v),
                        int(bbox_w),
                        int(bbox_h),
                        str(is_prime),
                    ]
                    self._writer_full_v1.writerow(row_v1)
                    if is_prime:
                        self._writer_prime_v1.writerow(row_v1)
                        self.saved_target_locations_v1.append((lat_v1, lon_v1))
                    self._f_full_v1.flush()
                    self._f_prime_v1.flush()

            # Evaluate and Log V2 (Interpolated LERP/SLERP) Data Streams
            if gps_v2:
                lat_v2, lon_v2 = gps_v2
                is_duplicate_v2 = any(
                    self._calculate_distance_m(lat_v2, lon_v2, l, ln) < self.min_dist_m
                    for l, ln in self.saved_target_locations_v2
                )
                if not is_duplicate_v2:
                    row_v2 = [
                        image_filename,
                        f"{lat_v2:.7f}",
                        f"{lon_v2:.7f}",
                        time_utc,
                        f"{box['conf']:.2f}",
                        int(u),
                        int(v),
                        int(bbox_w),
                        int(bbox_h),
                        str(is_prime),
                    ]
                    self._writer_full_v2.writerow(row_v2)
                    if is_prime:
                        self._writer_prime_v2.writerow(row_v2)
                        self.saved_target_locations_v2.append((lat_v2, lon_v2))
                    self._f_full_v2.flush()
                    self._f_prime_v2.flush()

            targets_logged_this_frame = True

            # Annotate Display Elements
            box_color = (0, 255, 0) if is_prime else (0, 165, 255)
            cv2.rectangle(
                annotated_frame,
                (int(box["x1"]), int(box["y1"])),
                (int(box["x2"]), int(box["y2"])),
                box_color,
                6,
            )

        if targets_logged_this_frame:
            target_img_path = os.path.join(
                self._targets_dir, f"target_{self._target_counter:03d}.jpg"
            )
            cv2.imwrite(target_img_path, annotated_frame)
            self._target_counter += 1

        # Publish Standard Detection2DArray to Drone Core
        det_array_msg = Detection2DArray()
        det_array_msg.header.stamp = msg.header.stamp
        det_array_msg.header.frame_id = msg.header.frame_id

        for box in final_boxes:
            det_msg = Detection2D()
            width, height = box["x2"] - box["x1"], box["y2"] - box["y1"]
            det_msg.bbox.center.position.x = box["x1"] + (width / 2.0)
            det_msg.bbox.center.position.y = box["y1"] + (height / 2.0)
            det_msg.bbox.size_x, det_msg.bbox.size_y = width, height

            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = str(box["cls"])
            result.hypothesis.score = box["conf"]
            det_msg.results.append(result)
            det_array_msg.detections.append(det_msg)

        self._detection_pub.publish(det_array_msg)

        # Publish Diagnostic Frame Output
        if self.get_parameter("publish_debug_image").get_parameter_value().bool_value:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            debug_msg.header = msg.header
            self._debug_img_pub.publish(debug_msg)

    def _raycast_to_gps(self, u: float, v: float, pose: PoseStamped):
        fx, fy = self.camera_model.fx(), self.camera_model.fy()
        cx, cy = self.camera_model.cx(), self.camera_model.cy()

        # 1. Pixel to normalized camera optical ray vector
        ray_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        ray_opt /= np.linalg.norm(ray_opt)

        # 2. Map OpenCV Optical convention to Drone Body NED
        ray_body_ned = np.array([-ray_opt[1], ray_opt[0], ray_opt[2]])
        mount_r = R_scipy.from_euler(
            "xyz", [self._mount_roll, self._mount_pitch, self._mount_yaw], degrees=True
        )
        ray_body_ned = mount_r.apply(ray_body_ned)

        # 3. Direct Singularity-Free Coordinate Transformation (Gimbal-Lock Fixed)
        q = pose.pose.orientation
        r_enu = R_scipy.from_quat([q.x, q.y, q.z, q.w])
        R_enu_to_ned = R_scipy.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        drone_r_ned = R_enu_to_ned * r_enu
        ray_world_ned = drone_r_ned.apply(ray_body_ned)
        ray_world_enu = np.array(
            [ray_world_ned[1], ray_world_ned[0], -ray_world_ned[2]]
        )

        # 4. Physical mounting spatial offset transformation
        mount_offset_ned = np.array([self._mount_x, self._mount_y, self._mount_z])
        cam_offset_ned = drone_r_ned.apply(mount_offset_ned)
        cam_offset_enu = np.array(
            [cam_offset_ned[1], cam_offset_ned[0], -cam_offset_ned[2]]
        )

        # 5. Pure Takeoff-Relative Coordinate Intersection (Takeoff Pad = 0.0m AGL Frame)
        drone_pos = pose.pose.position
        cam_z = drone_pos.z + cam_offset_enu[2]
        ground_z = (
            self.get_parameter("ground_altitude_m").get_parameter_value().double_value
        )

        if abs(ray_world_enu[2]) < 1e-6:
            return None
        t = (ground_z - cam_z) / ray_world_enu[2]
        if t < 0:
            return None

        target_x = drone_pos.x + cam_offset_enu[0] + t * ray_world_enu[0]
        target_y = drone_pos.y + cam_offset_enu[1] + t * ray_world_enu[1]

        # 6. Global Geodetic GPS transformation
        lat0, lon0 = self._home.geo.latitude, self._home.geo.longitude
        lat_offset = (target_y / self.R_EARTH) * (180.0 / math.pi)
        lon_scale = math.cos(math.radians(lat0))
        lon_offset = (target_x / (self.R_EARTH * lon_scale)) * (180.0 / math.pi)

        return (lat0 + lat_offset, lon0 + lon_offset)

    def _calculate_distance_m(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        dy = (lat2 - lat1) * self.R_EARTH * (math.pi / 180.0)
        dx = (
            (lon2 - lon1)
            * self.R_EARTH
            * math.cos(math.radians(lat1))
            * (math.pi / 180.0)
        )
        return math.hypot(dx, dy)

    def destroy_node(self):
        # Gracefully flush and commit unwritten buffers to disk before releasing handles
        for f in [
            self._f_full_v1,
            self._f_prime_v1,
            self._f_full_v2,
            self._f_prime_v2,
            self._f_metadata,
        ]:
            if hasattr(f, "close"):
                f.flush()
                f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloMissionNode()
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
