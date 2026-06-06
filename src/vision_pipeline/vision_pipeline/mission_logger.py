"""
Mission Logger Node (A/B Testing Edition)

Runs two raycasting algorithms in parallel for every detection:
  V1 (Legacy) : Uses the nearest historical pose (up to 200ms error).
  V2 (Interp) : Mathematically interpolates the exact pose at shutter time.

Outputs 4 CSVs:
  - mission_log_full_v1.csv  / mission_log_prime_v1.csv
  - mission_log_full_v2.csv  / mission_log_prime_v2.csv
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
import message_filters
import numpy as np
import rclpy
import tf2_ros
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
        # Output directories
        # ------------------------------------------------------------------
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.expanduser(
            f"~/CUASC_Mission_Data/Flight_{timestamp_str}"
        )
        self._frames_dir = os.path.join(self.save_dir, "frames")
        self._targets_dir = os.path.join(self.save_dir, "targets")

        os.makedirs(self._frames_dir, exist_ok=True)
        os.makedirs(self._targets_dir, exist_ok=True)

        # 4 CSVs for A/B Testing
        self.csv_full_v1 = os.path.join(self.save_dir, "mission_log_full_v1.csv")
        self.csv_prime_v1 = os.path.join(self.save_dir, "mission_log_prime_v1.csv")
        self.csv_full_v2 = os.path.join(self.save_dir, "mission_log_full_v2.csv")
        self.csv_prime_v2 = os.path.join(self.save_dir, "mission_log_prime_v2.csv")
        self._init_csvs()

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._frame_counter = 0
        self._target_counter = 1
        self._last_continuous_save_time = 0.0

        # Separate deduplication trackers for V1 and V2
        self.saved_target_locations_v1: List[Tuple[float, float]] = []
        self.saved_target_locations_v2: List[Tuple[float, float]] = []
        self.min_dist_m = 1.0

        self.cv_bridge = CvBridge()
        self.R_EARTH = 6378137.0
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False

        self._synced_callbacks = 0
        self._last_pose_log_time = 0.0

        self._drone_pose = None
        self._pose_history = deque(maxlen=200)  # ~10 s at 20 Hz
        self._home = None

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("ground_altitude_m", 0.0)
        self.declare_parameter("mount_x", 0.0)
        self.declare_parameter("mount_y", 0.0)
        self.declare_parameter("mount_z", 0.0)
        self.declare_parameter("mount_roll", 0.0)
        self.declare_parameter("mount_pitch", 0.0)
        self.declare_parameter("mount_yaw", 0.0)

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
        # Subscribers
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

        img_sub = message_filters.Subscriber(
            self, Image, "/camera/image_raw", qos_profile=qos_profile
        )
        img_sub.registerCallback(self._on_every_frame)

        det_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        det_sub = message_filters.Subscriber(
            self, Detection2DArray, "/drone_control/detection", qos_profile=det_qos
        )

        self.ts = message_filters.TimeSynchronizer([img_sub, det_sub], queue_size=60)
        self.ts.registerCallback(self._on_synced_data)

        self._readiness_timer = self.create_timer(3.0, self._log_readiness)

        self.get_logger().info(
            f"🚀 MissionLogger ready (A/B LERP Testing Enabled).\n"
            f"   frames        → {self._frames_dir}\n"
            f"   targets       → {self._targets_dir}\n"
            f"   V1 full CSV   → {self.csv_full_v1}\n"
            f"   V1 prime CSV  → {self.csv_prime_v1}\n"
            f"   V2 full CSV   → {self.csv_full_v2}\n"
            f"   V2 prime CSV  → {self.csv_prime_v2}\n"
            f"   mount offset (m)  : x={self._mount_x} y={self._mount_y} z={self._mount_z}\n"
            f"   mount rotation(°) : roll={self._mount_roll} pitch={self._mount_pitch} yaw={self._mount_yaw}\n"
            f"   sync: TimeSynchronizer queue=60 | min_dist_m: {self.min_dist_m}"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _init_csvs(self):
        headers = [
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
        for path in [
            self.csv_full_v1,
            self.csv_prime_v1,
            self.csv_full_v2,
            self.csv_prime_v2,
        ]:
            with open(path, mode="w", newline="") as f:
                csv.writer(f).writerow(headers)

    def _log_readiness(self):
        if (
            self.camera_info_received
            and self._drone_pose is not None
            and self._home is not None
        ):
            self.get_logger().info("✅ All systems ready — GPS logging is ACTIVE.")
            self._readiness_timer.cancel()

        self.get_logger().warn(
            f"⏳ Waiting for prerequisites:\n"
            f"   camera_info  : {'✅' if self.camera_info_received else '❌ not received yet'}\n"
            f"   drone_pose   : {'✅' if self._drone_pose is not None else '❌ no /mavros/local_position/pose'}\n"
            f"   home_position: {'✅' if self._home is not None else '❌ no /mavros/home_position/home (needs GPS lock)'}"
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().info(
                f"✅ Camera info received.\n"
                f"   fx={self.camera_model.fx():.2f}  fy={self.camera_model.fy():.2f}\n"
                f"   cx={self.camera_model.cx():.2f}  cy={self.camera_model.cy():.2f}"
            )

    def _on_pose(self, msg: PoseStamped) -> None:
        if math.isnan(msg.pose.position.x) or math.isnan(msg.pose.position.z):
            self.get_logger().info("  Position X or Z is NaN")
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
                f"🛸 Pose — pos: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m  "
                f"rpy: ({rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°)"
            )
            self._last_pose_log_time = now

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    def _on_every_frame(self, msg: Image) -> None:
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self._last_continuous_save_time) < 1.0:
            return
        self._frame_counter += 1
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        filename = f"frame_{self._frame_counter:06d}_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg"
        cv2.imwrite(os.path.join(self._frames_dir, filename), frame)
        self._last_continuous_save_time = current_time

        self.get_logger().info(
            f"💾 Continuous save: {filename} (frame #{self._frame_counter})"
        )

    # ------------------------------------------------------------------
    # Pose Synchronization Math (A/B)
    # ------------------------------------------------------------------
    def _get_pose_at_time_v1_nearest(self, target_stamp):
        """Legacy logic: Grabs the single closest pose"""
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

        if abs(delta_ms) * 1000 > 200:
            self.get_logger().warn(
                f"Best pose is {delta_ms:.0f}ms from image stamp — rejecting. "
                "Ensure Jetson and Pixhawk clocks are synced."
            )
            return None

        self.get_logger().info(
            f"      [PoseSync] matched pose {delta_ms:.1f}ms from image stamp"
        )
        return best_pose

    def _get_pose_at_time_v2_interpolated(self, target_stamp):
        """New logic: Interpolates exact pose at shutter time"""
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

        # LERP Position
        interp_pose = PoseStamped()
        interp_pose.header.stamp = target_stamp
        p0, p1 = before.pose.position, after.pose.position
        interp_pose.pose.position.x = p0.x + alpha * (p1.x - p0.x)
        interp_pose.pose.position.y = p0.y + alpha * (p1.y - p0.y)
        interp_pose.pose.position.z = p0.z + alpha * (p1.z - p0.z)

        # SLERP Orientation
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

    # ------------------------------------------------------------------
    # Synchronised target geolocator
    # ------------------------------------------------------------------
    def _on_synced_data(self, img_msg: Image, det_array_msg: Detection2DArray) -> None:
        self._synced_callbacks += 1
        n_det = len(det_array_msg.detections)

        self.get_logger().info(
            f"🔗 Sync callback #{self._synced_callbacks} — "
            f"{n_det} detection(s) in this frame"
        )

        if (
            not self.camera_info_received
            or self._drone_pose is None
            or self._home is None
        ):
            self.get_logger().warn(
                f"   ⚠️  Dropping synced frame — prerequisites not met:\n"
                f"   camera_info={self.camera_info_received} "
                f"pose={self._drone_pose is not None} "
                f"home={self._home is not None}"
            )
            return

        if n_det == 0:
            return

        frame = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        img_h, img_w = frame.shape[:2]

        # Prime zone boundaries — center 60% (20% margin on every side)
        margin_x = img_w * 0.20
        margin_y = img_h * 0.20

        # Grab BOTH poses for A/B testing
        pose_v1 = self._get_pose_at_time_v1_nearest(img_msg.header.stamp)
        pose_v2 = self._get_pose_at_time_v2_interpolated(img_msg.header.stamp)

        targets_logged_this_frame = False

        for det_idx, det in enumerate(det_array_msg.detections):
            if not det.results:
                self.get_logger().warn(
                    f"   Det {det_idx + 1}/{n_det}: no results field — skipping"
                )
                continue

            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            confidence = det.results[0].hypothesis.score
            cls_id = det.results[0].hypothesis.class_id
            bbox_w = det.bbox.size_x
            bbox_h = det.bbox.size_y

            is_prime = margin_x < u < (img_w - margin_x) and margin_y < v < (
                img_h - margin_y
            )
            zone_label = "PRIME" if is_prime else "EDGE"

            self.get_logger().info(
                f"   Det {det_idx + 1}/{n_det}: "
                f"class={cls_id} conf={confidence:.3f} "
                f"pixel=({u:.0f}, {v:.0f}) zone={zone_label}"
            )

            # Process V1 (Legacy)
            gps_v1 = self._raycast_to_gps(u, v, pose_v1) if pose_v1 else None
            # Process V2 (Interpolated)
            gps_v2 = self._raycast_to_gps(u, v, pose_v2) if pose_v2 else None

            if gps_v1 is None and gps_v2 is None:
                self.get_logger().warn(
                    f"   ↳ ❌ Raycast returned None for pixel ({u:.0f}, {v:.0f})"
                )
                continue

            image_filename = f"target_{self._target_counter:03d}.jpg"
            time_utc = datetime.utcnow().strftime("%H:%M:%S")

            # Write V1 Results
            if gps_v1:
                lat_v1, lon_v1 = gps_v1
                is_duplicate_v1 = any(
                    self._calculate_distance_m(lat_v1, lon_v1, lat, lon)
                    < self.min_dist_m
                    for lat, lon in self.saved_target_locations_v1
                )

                if is_duplicate_v1:
                    self.get_logger().info(
                        f"   ↳ ⏭️  Duplicate (v1) [{zone_label}] — "
                        f"within {self.min_dist_m}m of a known prime target"
                    )

                if not is_duplicate_v1:
                    row_v1 = [
                        image_filename,
                        f"{lat_v1:.7f}",
                        f"{lon_v1:.7f}",
                        time_utc,
                        f"{confidence:.2f}",
                        int(u),
                        int(v),
                        int(bbox_w),
                        int(bbox_h),
                        str(is_prime),
                    ]
                    with open(self.csv_full_v1, mode="a", newline="") as f:
                        csv.writer(f).writerow(row_v1)
                    if is_prime:
                        with open(self.csv_prime_v1, mode="a", newline="") as f:
                            csv.writer(f).writerow(row_v1)
                        self.saved_target_locations_v1.append((lat_v1, lon_v1))
                    self.get_logger().info(
                        f"   ↳ 📝 LOGGED [{zone_label}]: {image_filename} "
                        f"lat={lat_v1:.7f} lon={lon_v1:.7f} conf={confidence:.2f}"
                    )
            # Write V2 Results
            if gps_v2:
                lat_v2, lon_v2 = gps_v2
                is_duplicate_v2 = any(
                    self._calculate_distance_m(lat_v2, lon_v2, lat, lon)
                    < self.min_dist_m
                    for lat, lon in self.saved_target_locations_v2
                )

                if not is_duplicate_v2:
                    row_v2 = [
                        image_filename,
                        f"{lat_v2:.7f}",
                        f"{lon_v2:.7f}",
                        time_utc,
                        f"{confidence:.2f}",
                        int(u),
                        int(v),
                        int(bbox_w),
                        int(bbox_h),
                        str(is_prime),
                    ]
                    with open(self.csv_full_v2, mode="a", newline="") as f:
                        csv.writer(f).writerow(row_v2)
                    if is_prime:
                        with open(self.csv_prime_v2, mode="a", newline="") as f:
                            csv.writer(f).writerow(row_v2)
                        self.saved_target_locations_v2.append((lat_v2, lon_v2))

            # Live Delta Logging
            if gps_v1 and gps_v2:
                delta_m = self._calculate_distance_m(
                    gps_v1[0], gps_v1[1], gps_v2[0], gps_v2[1]
                )
                self.get_logger().info(
                    f"   ↳ 📝 [{zone_label}] Delta between V1/V2 Math: {delta_m:.2f} meters"
                )

            targets_logged_this_frame = True

            # Annotate and save image
            box_color = (0, 255, 0) if is_prime else (0, 165, 255)

            # Annotate frame with zone colour
            x1 = int(u - bbox_w / 2)
            y1 = int(v - bbox_h / 2)
            x2 = int(u + bbox_w / 2)
            y2 = int(v + bbox_h / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 6)

        if targets_logged_this_frame:
            image_path = os.path.join(
                self._targets_dir, f"target_{self._target_counter:03d}.jpg"
            )
            cv2.imwrite(image_path, frame)
            self.get_logger().info(
                f"   ↳ 🖼️  Saved annotated image: target_{self._target_counter:03d}.jpg"
            )
            self._target_counter += 1

    # ------------------------------------------------------------------
    # Raycast
    # ------------------------------------------------------------------
    def _raycast_to_gps(self, u: float, v: float, pose: PoseStamped):
        # Step A: pixel → normalised optical ray
        fx = self.camera_model.fx()
        fy = self.camera_model.fy()
        cx = self.camera_model.cx()
        cy = self.camera_model.cy()

        ray_opt = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        ray_opt /= np.linalg.norm(ray_opt)

        self.get_logger().info(
            f"      [Raycast] pixel=({u:.0f},{v:.0f}) "
            f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} "
            f"ray_opt={ray_opt.round(4).tolist()}"
        )

        ray_body_ned = np.array([-ray_opt[1], ray_opt[0], ray_opt[2]])
        mount_r = R_scipy.from_euler(
            "xyz", [self._mount_roll, self._mount_pitch, self._mount_yaw], degrees=True
        )
        ray_body_ned = mount_r.apply(ray_body_ned)

        self.get_logger().info(
            f"      [Raycast] ray_body_ned={ray_body_ned.round(4).tolist()}"
        )

        q = pose.pose.orientation
        r_enu = R_scipy.from_quat([q.x, q.y, q.z, q.w])
        roll_enu, pitch_enu, yaw_enu = r_enu.as_euler("xyz", degrees=False)

        roll = roll_enu
        pitch = -pitch_enu
        yaw = math.pi / 2.0 - yaw_enu

        self.get_logger().info(
            f"      [Raycast] drone euler NED — "
            f"roll={math.degrees(roll):.2f}° "
            f"pitch={math.degrees(pitch):.2f}° "
            f"yaw={math.degrees(yaw):.2f}°"
        )

        drone_r_ned = R_scipy.from_euler(
            "ZYX", [math.pi / 2.0 - yaw_enu, -pitch_enu, roll_enu], degrees=False
        )
        ray_world_ned = drone_r_ned.apply(ray_body_ned)
        ray_world_enu = np.array(
            [ray_world_ned[1], ray_world_ned[0], -ray_world_ned[2]]
        )

        self.get_logger().info(
            f"      [Raycast] ray_world_enu={ray_world_enu.round(4).tolist()}"
        )

        mount_offset_ned = np.array([self._mount_x, self._mount_y, self._mount_z])
        cam_offset_ned = drone_r_ned.apply(mount_offset_ned)
        cam_offset_enu = np.array(
            [cam_offset_ned[1], cam_offset_ned[0], -cam_offset_ned[2]]
        )

        drone_pos = pose.pose.position
        cam_z = drone_pos.z + cam_offset_enu[2]

        # Standard YAML Ground altitude (LiDAR removed per request)
        ground_z = (
            self.get_parameter("ground_altitude_m").get_parameter_value().double_value
        )

        self.get_logger().info(
            f"      [Raycast] drone_z={drone_pos.z:.2f}m  "
            f"cam_z={cam_z:.2f}m  ground_z={ground_z:.2f}m"
        )

        if abs(ray_world_enu[2]) < 1e-6:
            self.get_logger().warn("      [Raycast] ❌ dropped — ray nearly horizontal")
            return None
        t = (ground_z - cam_z) / ray_world_enu[2]
        if t < 0:
            self.get_logger().warn(
                f"      [Raycast] ❌ dropped — ray points skyward (t={t:.2f}). "
                "Check ground_altitude_m and that drone_pos.z is positive."
            )
            return None

        self.get_logger().info(f"      [Raycast] t={t:.2f} m (ray length to ground)")

        target_x = drone_pos.x + cam_offset_enu[0] + t * ray_world_enu[0]
        target_y = drone_pos.y + cam_offset_enu[1] + t * ray_world_enu[1]

        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude
        lat_offset = (target_y / self.R_EARTH) * (180.0 / math.pi)
        lon_scale = math.cos(math.radians(lat0))
        lon_offset = (target_x / (self.R_EARTH * lon_scale)) * (180.0 / math.pi)

        final_lat = lat0 + lat_offset
        final_lon = lon0 + lon_offset

        self.get_logger().info(
            f"      [Raycast] home=({lat0:.7f},{lon0:.7f})  "
            f"offset=({target_x:.2f}m E, {target_y:.2f}m N)  "
            f"final=({final_lat:.7f},{final_lon:.7f})"
        )

        return (final_lat, final_lon)

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
