"""
Mission Logger Node

Runs two tasks in parallel:

1. CONTINUOUS RECORDER  – saves every single incoming frame as a JPEG
   to  <save_dir>/frames/  so you have a complete flight record.

2. TARGET GEOLOCATOR    – time-synchronises images with YOLO detections,
   raycasts each detection to the ground plane, and writes GPS coordinates
   to mission_log.csv.  Annotated images of new targets are saved separately
   to  <save_dir>/targets/.
"""

import csv
import math
import os
import time
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

        self.csv_path = os.path.join(self.save_dir, "mission_log.csv")
        self._init_csv()

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._frame_counter = 0
        self._target_counter = 1
        self._last_continuous_save_time = 0.0

        self.saved_target_locations: List[Tuple[float, float]] = []
        self.min_dist_m = 10.0

        self.cv_bridge = CvBridge()
        self.R_EARTH = 6378137.0
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False

        # Sync diagnostics
        self._synced_callbacks = 0
        self._successful_raycasts = 0
        self._last_pose_log_time = 0.0

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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._drone_pose = None
        self._home = None

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
            Image, "/camera/image_raw", self._on_every_frame, qos_profile
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

        # --- The Synchronizer Setup ---
        # Provide the QoS profile so it can hear the Best Effort camera
        img_sub = message_filters.Subscriber(
            self, Image, "/camera/image_raw", qos_profile=qos_profile
        )

        # YOLO publishes Reliably, so it does not need a special QoS profile here
        det_sub = message_filters.Subscriber(
            self, Detection2DArray, "/drone_control/detection"
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [img_sub, det_sub], queue_size=10, slop=0.5
        )
        self.ts.registerCallback(self._on_synced_data)

        # Periodic readiness check timer — fires every 3s until system is ready
        self._readiness_timer = self.create_timer(3.0, self._log_readiness)

        self.get_logger().info(
            f"🚀 MissionLogger ready.\n"
            f"   frames  → {self._frames_dir}\n"
            f"   targets → {self._targets_dir}\n"
            f"   CSV     → {self.csv_path}\n"
            f"   mount offset (m)  : x={self._mount_x} y={self._mount_y} z={self._mount_z}\n"
            f"   mount rotation(°) : roll={self._mount_roll} "
            f"pitch={self._mount_pitch} yaw={self._mount_yaw}\n"
            f"   sync slop: 0.5 s | min_dist_m: {self.min_dist_m}"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _init_csv(self):
        with open(self.csv_path, mode="w", newline="") as f:
            csv.writer(f).writerow(
                ["Image_Name", "Latitude", "Longitude", "Time_UTC", "YOLO_Confidence"]
            )

    def _log_readiness(self):
        """Fires every 3 s so you can see in the terminal exactly which
        prerequisite is still missing before GPS logging can begin."""
        if (
            self.camera_info_received
            and self._drone_pose is not None
            and self._home is not None
        ):
            self.get_logger().info("✅ All systems ready — GPS logging is ACTIVE.")
            self._readiness_timer.cancel()
            return

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
        self._drone_pose = msg

        # Log pose at most once per second so terminal isn't flooded
        now = time.time()
        if now - self._last_pose_log_time >= 1.0:
            p = msg.pose.position
            q = msg.pose.orientation
            # Convert quaternion to roll/pitch/yaw for human-readable output
            r = R_scipy.from_quat([q.x, q.y, q.z, q.w])
            rpy = r.as_euler("xyz", degrees=True)
            self.get_logger().info(
                f"🛸 Pose — pos: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m  "
                f"rpy: ({rpy[0]:.1f}°, {rpy[1]:.1f}°, {rpy[2]:.1f}°)"
            )
            self._last_pose_log_time = now

    def _on_home(self, msg: HomePosition) -> None:
        if self._home is None:
            self.get_logger().info(
                f"✅ Home position locked!\n"
                f"   lat={msg.geo.latitude:.7f}  "
                f"lon={msg.geo.longitude:.7f}  "
                f"alt={msg.geo.altitude:.2f} m MSL"
            )
        self._home = msg

    # ------------------------------------------------------------------
    # Continuous frame recorder (throttled to 1 Hz)
    # ------------------------------------------------------------------

    def _on_every_frame(self, msg: Image) -> None:
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self._last_continuous_save_time) < 1.0:
            return

        self._frame_counter += 1
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        stamp_sec = msg.header.stamp.sec
        stamp_ns = msg.header.stamp.nanosec
        filename = f"frame_{self._frame_counter:06d}_{stamp_sec}_{stamp_ns}.jpg"
        path = os.path.join(self._frames_dir, filename)
        cv2.imwrite(path, frame)
        self._last_continuous_save_time = current_time

        self.get_logger().info(
            f"💾 Continuous save: {filename} (frame #{self._frame_counter})"
        )

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
        targets_logged_this_frame = False

        for det_idx, det in enumerate(det_array_msg.detections):
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            confidence = det.results[0].hypothesis.score
            cls_id = det.results[0].hypothesis.class_id

            self.get_logger().info(
                f"   Det {det_idx + 1}/{n_det}: "
                f"class={cls_id} conf={confidence:.3f} "
                f"pixel=({u:.0f}, {v:.0f})"
            )

            gps_coord = self._raycast_to_gps(u, v)
            if gps_coord is None:
                self.get_logger().warn(
                    f"   ↳ ❌ Raycast returned None for pixel ({u:.0f}, {v:.0f})"
                )
                continue

            target_lat, target_lon = gps_coord
            self._successful_raycasts += 1
            self.get_logger().info(
                f"   ↳ ✅ Raycast → lat={target_lat:.7f}  lon={target_lon:.7f} "
                f"(total successful raycasts: {self._successful_raycasts})"
            )

            is_duplicate = any(
                self._calculate_distance_m(target_lat, target_lon, lat, lon)
                < self.min_dist_m
                for lat, lon in self.saved_target_locations
            )
            if is_duplicate:
                self.get_logger().info(
                    f"   ↳ ⏭️  Duplicate — within {self.min_dist_m}m of a known target"
                )
                continue

            # New target — annotate frame and log
            cv2.circle(frame, (int(u), int(v)), 50, (0, 0, 255), 4)
            text = f"Lat: {target_lat:.6f}, Lon: {target_lon:.6f}"
            cv2.putText(
                frame,
                text,
                (int(u) - 100, int(v) - 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5,
                (0, 255, 0),
                4,
            )

            image_filename = f"target_{self._target_counter:03d}.jpg"
            time_utc = datetime.utcnow().strftime("%H:%M:%S")
            with open(self.csv_path, mode="a", newline="") as f:
                csv.writer(f).writerow(
                    [
                        image_filename,
                        f"{target_lat:.7f}",
                        f"{target_lon:.7f}",
                        time_utc,
                        f"{confidence:.2f}",
                    ]
                )

            self.get_logger().info(
                f"   ↳ 📝 LOGGED to CSV: {image_filename} "
                f"lat={target_lat:.7f} lon={target_lon:.7f} conf={confidence:.2f}"
            )

            self.saved_target_locations.append((target_lat, target_lon))
            targets_logged_this_frame = True

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

    def _raycast_to_gps(self, u: float, v: float):
        """
        Mirrors offline_photogrammetry.py exactly:
          pixel → optical ray → NED body (hardcoded nadir axis swap)
          → mount correction → world NED (ArduPilot ZYX)
          → world ENU → ground intersection → GPS offset

        Does NOT call rectifyPoint because image_grabber already applied
        cv2.remap (full undistortion) before publishing.
        """

        # Step A: pixel → optical ray using K directly (image already flat)
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

        # Step B: optical → NED body (nadir axis swap + mount correction)
        ray_body_ned = np.array([-ray_opt[1], ray_opt[0], ray_opt[2]])

        mount_r = R_scipy.from_euler(
            "xyz",
            [self._mount_roll, self._mount_pitch, self._mount_yaw],
            degrees=True,
        )
        ray_body_ned = mount_r.apply(ray_body_ned)

        self.get_logger().info(
            f"      [Raycast] ray_body_ned={ray_body_ned.round(4).tolist()}"
        )

        # Step C: NED body → world NED (ArduPilot ZYX from MAVROS ENU quaternion)
        q = self._drone_pose.pose.orientation
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

        drone_r_ned = R_scipy.from_euler("ZYX", [yaw, pitch, roll], degrees=False)
        ray_world_ned = drone_r_ned.apply(ray_body_ned)

        # Step D: NED → ENU
        ray_world_enu = np.array(
            [ray_world_ned[1], ray_world_ned[0], -ray_world_ned[2]]
        )

        self.get_logger().info(
            f"      [Raycast] ray_world_enu={ray_world_enu.round(4).tolist()}"
        )

        # Camera offset in world ENU
        mount_offset_ned = np.array([self._mount_x, self._mount_y, self._mount_z])
        cam_offset_ned = drone_r_ned.apply(mount_offset_ned)
        cam_offset_enu = np.array(
            [cam_offset_ned[1], cam_offset_ned[0], -cam_offset_ned[2]]
        )

        drone_pos = self._drone_pose.pose.position
        cam_z = drone_pos.z + cam_offset_enu[2]
        ground_z = (
            self.get_parameter("ground_altitude_m").get_parameter_value().double_value
        )

        self.get_logger().info(
            f"      [Raycast] drone_z={drone_pos.z:.2f}m  "
            f"cam_z={cam_z:.2f}m  ground_z={ground_z:.2f}m"
        )

        # Step E: ground intersection
        if abs(ray_world_enu[2]) < 1e-6:
            self.get_logger().warn("      [Raycast] ❌ dropped — ray nearly horizontal")
            return None
        t = (ground_z - cam_z) / ray_world_enu[2]
        if t < 0:
            self.get_logger().warn(
                f"      [Raycast] ❌ dropped — ray points skyward (t={t:.2f}). "
                "Is ground_altitude_m correct? Is drone_pose.z positive (above ground)?"
            )
            return None

        self.get_logger().info(f"      [Raycast] t={t:.2f} m (ray length to ground)")

        target_x = drone_pos.x + cam_offset_enu[0] + t * ray_world_enu[0]
        target_y = drone_pos.y + cam_offset_enu[1] + t * ray_world_enu[1]

        # Step F: metric ENU offset → GPS
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

    # ------------------------------------------------------------------
    # Distance helper
    # ------------------------------------------------------------------

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
