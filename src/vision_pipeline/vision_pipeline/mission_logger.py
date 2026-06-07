#!/usr/bin/env python3
"""
Decoupled Master Mission Logger Node

Subscribes to high-frequency raw imagery and live flight telemetry to log
unannotated post-processing sessions continuously. Matches incoming YOLO detections
against a thread-safe historical image ring buffer to geolocate and crop target chips.

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
import yaml
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
from vision_msgs.msg import Detection2DArray


class MissionLogger(Node):
    def __init__(self) -> None:
        super().__init__("mission_logger")

        # ------------------------------------------------------------------
        # Parameters Configuration
        # ------------------------------------------------------------------
        self.declare_parameter("ground_altitude_m", 0.0)
        self.declare_parameter("mount_x", 0.0)
        self.declare_parameter("mount_y", 0.0)
        self.declare_parameter("mount_z", 0.0)
        self.declare_parameter("mount_roll", 0.0)
        self.declare_parameter("mount_pitch", 0.0)
        self.declare_parameter("mount_yaw", 0.0)
        self.declare_parameter("deduplication_threshold_m", 1.0)

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
        self.min_dist_m = (
            self.get_parameter("deduplication_threshold_m")
            .get_parameter_value()
            .double_value
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

        # Persistent File Handles
        self._init_persistent_csvs()

        # ------------------------------------------------------------------
        # Pipeline State Setup
        # ------------------------------------------------------------------
        self._frame_counter = 0
        self._target_counter = 1
        self._last_continuous_save_time = 0.0

        self.saved_target_locations_v1: List[Tuple[float, float]] = []
        self.saved_target_locations_v2: List[Tuple[float, float]] = []

        self.cv_bridge = CvBridge()
        self.R_EARTH = 6378137.0
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False

        self._frames_received = 0
        self._drone_pose = None
        self._pose_history = deque(maxlen=300)  # ~15s telemetry ring buffer at 20Hz
        self._home = None

        # Thread-safe image ring buffer matching camera frames to YOLO detection timestamps
        self._image_ring_buffer = deque(maxlen=100)

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

        # Subscribe directly to the decoupled YOLO Node detections topic
        self.create_subscription(
            Detection2DArray,
            "/drone_control/detection",
            self._on_detections,
            qos_profile,
        )

        self.get_logger().info("🚀 master mission logger node is online.")

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

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True

            # CRITICAL FIX: Cast elements to float/int primitives to prevent YAML serialization errors
            cam_data = {
                "image_width": int(msg.width),
                "image_height": int(msg.height),
                "camera_matrix": {"data": [float(x) for x in msg.k]},
                "distortion_coefficients": {"data": [float(x) for x in msg.d]},
                "projection_matrix": {"data": [float(x) for x in msg.p]},
            }

            try:
                with open(self.camera_info_path, "w") as f:
                    yaml.safe_dump(cam_data, f, default_flow_style=False)
                self.get_logger().info(
                    "📷 Camera calibration specifications logged to session folder."
                )
            except Exception as e:
                self.get_logger().error(f"Failed to save camera specifications: {e}")

    def _on_pose(self, msg: PoseStamped) -> None:
        if not math.isnan(msg.pose.position.x):
            self._drone_pose = msg
            self._pose_history.append(msg)

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    def _image_callback(self, msg: Image) -> None:
        self._frames_received += 1

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV_Bridge conversion error: {e}")
            return

        # Cache standard frame data inside our decoupled circular ring buffer
        self._image_ring_buffer.append((msg.header.stamp, frame))

        # Synchronously record unannotated post-processing session frames (taking <1.5ms)
        if self._drone_pose is not None and self._home is not None:
            raw_frame_name = f"frame_{self._frames_received:06d}.jpg"
            raw_filepath = os.path.join(self.images_dir, raw_frame_name)

            try:
                cv2.imwrite(raw_filepath, frame)

                # Match telemetry stamp to camera trigger
                pose_for_log = self._get_pose_at_time_v2_interpolated(
                    msg.header.stamp, list(self._pose_history)
                )
                if pose_for_log is None:
                    pose_for_log = self._get_pose_at_time_v1_nearest(
                        msg.header.stamp, list(self._pose_history)
                    )

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

        # Environmental continuous flight logger (1Hz)
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self._last_continuous_save_time) >= 1.0:
            self._frame_counter += 1
            raw_filename = f"frame_{self._frame_counter:06d}_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg"
            cv2.imwrite(os.path.join(self._frames_dir, raw_filename), frame)
            self._last_continuous_save_time = current_time

    def _on_detections(self, msg: Detection2DArray) -> None:
        """Processes incoming YOLO objects matched against buffered frames."""
        if (
            not self.camera_info_received
            or self._drone_pose is None
            or self._home is None
        ):
            return

        # Match detection frame timestamp to the raw frame in the ring buffer
        match_image = self._find_matching_image_from_buffer(msg.header.stamp)
        if match_image is None:
            self.get_logger().warn(
                "⚠️ Discarded detection: Matching frame not found in ring buffer.",
                throttle_duration_sec=3.0,
            )
            return

        h, w = match_image.shape[:2]
        margin_x, margin_y = w * 0.20, h * 0.20

        # Take a snapshot of the telemetry cache to ensure thread safety
        history_snapshot = list(self._pose_history)
        pose_v1 = self._get_pose_at_time_v1_nearest(msg.header.stamp, history_snapshot)
        pose_v2 = self._get_pose_at_time_v2_interpolated(
            msg.header.stamp, history_snapshot
        )

        targets_logged_this_frame = False
        annotated_frame = match_image.copy()

        for det in msg.detections:
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            bbox_w = det.bbox.size_x
            bbox_h = det.bbox.size_y

            is_prime = margin_x < u < (w - margin_x) and margin_y < v < (h - margin_y)
            conf = det.results[0].hypothesis.score

            gps_v1 = self._raycast_to_gps(u, v, pose_v1) if pose_v1 else None
            gps_v2 = self._raycast_to_gps(u, v, pose_v2) if pose_v2 else None

            if gps_v1 is None and gps_v2 is None:
                continue

            image_filename = f"target_{self._target_counter:03d}.jpg"
            time_utc = datetime.utcnow().strftime("%H:%M:%S")

            if gps_v1:
                lat_v1, lon_v1 = gps_v1
                is_duplicate_v1 = False
                if self.min_dist_m > 1e-5:
                    is_duplicate_v1 = any(
                        self._calculate_distance_m(lat_v1, lon_v1, l, ln)
                        < self.min_dist_m
                        for l, ln in self.saved_target_locations_v1
                    )
                if not is_duplicate_v1:
                    row_v1 = [
                        image_filename,
                        f"{lat_v1:.7f}",
                        f"{lon_v1:.7f}",
                        time_utc,
                        f"{conf:.2f}",
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

            if gps_v2:
                lat_v2, lon_v2 = gps_v2
                is_duplicate_v2 = False
                if self.min_dist_m > 1e-5:
                    is_duplicate_v2 = any(
                        self._calculate_distance_m(lat_v2, lon_v2, l, ln)
                        < self.min_dist_m
                        for l, ln in self.saved_target_locations_v2
                    )
                if not is_duplicate_v2:
                    row_v2 = [
                        image_filename,
                        f"{lat_v2:.7f}",
                        f"{lon_v2:.7f}",
                        time_utc,
                        f"{conf:.2f}",
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

            box_color = (0, 255, 0) if is_prime else (0, 165, 255)
            x1 = int(u - bbox_w / 2.0)
            y1 = int(v - bbox_h / 2.0)
            x2 = int(u + bbox_w / 2.0)
            y2 = int(v + bbox_h / 2.0)
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), box_color, 6)

        if targets_logged_this_frame:
            target_img_path = os.path.join(
                self._targets_dir, f"target_{self._target_counter:03d}.jpg"
            )
            cv2.imwrite(target_img_path, annotated_frame)
            self._target_counter += 1

    def _find_matching_image_from_buffer(self, target_stamp) -> np.ndarray:
        if not self._image_ring_buffer:
            return None

        target_sec = target_stamp.sec + target_stamp.nanosec * 1e-9

        # Pull closest image matching stamp key
        best_match = min(
            self._image_ring_buffer,
            key=lambda img_data: abs(
                (img_data[0].sec + img_data[0].nanosec * 1e-9) - target_sec
            ),
        )

        match_sec = best_match[0].sec + best_match[0].nanosec * 1e-9
        if abs(match_sec - target_sec) < 0.25:  # Match within 250ms
            return best_match[1]
        return None

    def _get_pose_at_time_v1_nearest(self, target_stamp, pose_history_snapshot):
        if not pose_history_snapshot:
            return None
        target_sec = target_stamp.sec + target_stamp.nanosec * 1e-9
        best_pose = min(
            pose_history_snapshot,
            key=lambda p: abs(
                (p.header.stamp.sec + p.header.stamp.nanosec * 1e-9) - target_sec
            ),
        )
        best_sec = best_pose.header.stamp.sec + best_pose.header.stamp.nanosec * 1e-9
        delta_ms = abs(best_sec - target_sec) * 1000

        if delta_ms > 200:
            return pose_history_snapshot[-1]
        return best_pose

    def _get_pose_at_time_v2_interpolated(self, target_stamp, pose_history_snapshot):
        if len(pose_history_snapshot) < 2:
            return None
        target_sec = target_stamp.sec + target_stamp.nanosec * 1e-9
        poses = sorted(
            pose_history_snapshot,
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
            return pose_history_snapshot[-1]

        t0 = before.header.stamp.sec + before.header.stamp.nanosec * 1e-9
        t1 = after.header.stamp.sec + after.header.stamp.nanosec * 1e-9

        if (t1 - t0) == 0 or (target_sec - t0) > 0.2 or (t1 - target_sec) > 0.2:
            return pose_history_snapshot[-1]

        alpha = (target_sec - t0) / (t1 - t0)
        interp_pose = PoseStamped()
        interp_pose.header.stamp = target_stamp

        p0, p1 = before.pose.position, after.pose.position
        interp_pose.pose.position.x = p0.x + alpha * (p1.x - p0.x)
        interp_pose.pose.position.y = p0.y + alpha * (p1.y - p0.y)
        interp_pose.pose.position.z = p0.z + alpha * (p1.z - p0.z)

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

    def _raycast_to_gps(self, u: float, v: float, pose: PoseStamped):
        fx, fy = self.camera_model.fx(), self.camera_model.fy()
        cx, cy = self.camera_model.cx(), self.camera_model.cy()

        ray_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        ray_opt /= np.linalg.norm(ray_opt)

        ray_body_ned = np.array([-ray_opt[1], ray_opt[0], ray_opt[2]])
        mount_r = R_scipy.from_euler(
            "xyz", [self._mount_roll, self._mount_pitch, self._mount_yaw], degrees=True
        )
        ray_body_ned = mount_r.apply(ray_body_ned)

        q = pose.pose.orientation
        r_enu = R_scipy.from_quat([q.x, q.y, q.z, q.w])
        R_enu_to_ned = R_scipy.from_matrix([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

        drone_r_ned = R_enu_to_ned * r_enu
        ray_world_ned = drone_r_ned.apply(ray_body_ned)
        ray_world_enu = np.array(
            [ray_world_ned[1], ray_world_ned[0], -ray_world_ned[2]]
        )

        mount_offset_ned = np.array([self._mount_x, self._mount_y, self._mount_z])
        cam_offset_ned = drone_r_ned.apply(mount_offset_ned)
        cam_offset_enu = np.array(
            [cam_offset_ned[1], cam_offset_ned[0], -cam_offset_ned[2]]
        )

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
    node = MissionLogger()
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
