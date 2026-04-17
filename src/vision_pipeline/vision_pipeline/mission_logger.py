import csv
import math
import os
from datetime import datetime
from typing import List, Tuple

import cv2
import image_geometry
import message_filters
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R_scipy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray


class MissionLogger(Node):
    def __init__(self) -> None:
        super().__init__("mission_logger")

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = os.path.expanduser(
            f"~/CUASC_Mission_Data/Flight_{timestamp_str}"
        )
        os.makedirs(self.save_dir, exist_ok=True)
        self.csv_path = os.path.join(self.save_dir, "mission_log.csv")
        self._init_csv()

        self.image_counter = 1
        self.saved_target_locations: List[Tuple[float, float]] = []
        self.min_dist_m = 10.0

        self.cv_bridge = CvBridge()
        self.R_EARTH = 6378137.0
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_info_received = False
        self.declare_parameter("ground_altitude_m", 0.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._drone_pose = None
        self._home = None

        # Standard Subscribers
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._on_camera_info, 10
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            HomePosition, "/mavros/home_position/home", self._on_home, 10
        )

        # UPDATE: Synchronized Subscribers
        img_sub = message_filters.Subscriber(self, Image, "/camera/image_raw")
        det_sub = message_filters.Subscriber(
            self, Detection2DArray, "/drone_control/detection"
        )
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [img_sub, det_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self._on_synced_data)

        self.get_logger().info(
            f"🚀 Mission Logger Ready. Saving data to: {self.save_dir}"
        )

    def _init_csv(self):
        with open(self.csv_path, mode="w", newline="") as file:
            csv.writer(file).writerow(
                ["Image_Name", "Latitude", "Longitude", "Time_UTC", "YOLO_Confidence"]
            )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True

    def _on_pose(self, msg: PoseStamped) -> None:
        self._drone_pose = msg

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    def _on_synced_data(self, img_msg: Image, det_array_msg: Detection2DArray) -> None:
        """Triggers ONLY when an Image and its matching YOLO array arrive together."""
        if (
            not self.camera_info_received
            or self._drone_pose is None
            or self._home is None
        ):
            return
        if not det_array_msg.detections:
            return

        frame = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        targets_logged_this_frame = False

        for det in det_array_msg.detections:
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            confidence = det.results[0].hypothesis.score

            gps_coord = self._raycast_to_gps(u, v)
            if gps_coord is None:
                continue
            target_lat, target_lon = gps_coord

            # Spatial Throttling
            is_duplicate = False
            for saved_lat, saved_lon in self.saved_target_locations:
                if (
                    self._calculate_distance_m(
                        target_lat, target_lon, saved_lat, saved_lon
                    )
                    < self.min_dist_m
                ):
                    is_duplicate = True
                    break

            if is_duplicate:
                continue

            # It's a new target! Draw on the frame
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

            # Write to CSV immediately
            image_filename = f"target_{self.image_counter:03d}.jpg"
            time_utc = datetime.utcnow().strftime("%H:%M:%S")
            with open(self.csv_path, mode="a", newline="") as file:
                csv.writer(file).writerow(
                    [
                        image_filename,
                        f"{target_lat:.7f}",
                        f"{target_lon:.7f}",
                        time_utc,
                        f"{confidence:.2f}",
                    ]
                )

            self.get_logger().info(
                f"✅ LOGGED: {image_filename} -> Lat: {target_lat:.7f}"
            )
            self.saved_target_locations.append((target_lat, target_lon))
            targets_logged_this_frame = True

        # If we drew on the frame, save it to the hard drive once
        if targets_logged_this_frame:
            image_path = os.path.join(
                self.save_dir, f"target_{self.image_counter:03d}.jpg"
            )
            cv2.imwrite(image_path, frame)
            self.image_counter += 1

    # [Keep _raycast_to_gps and _calculate_distance_m exactly the same...]
    def _raycast_to_gps(self, u: float, v: float):
        # 1. Undistort JUST the single center pixel using the camera matrix
        rectified_u, rectified_v = self.camera_model.rectifyPoint((u, v))

        # 2. Shoot the optical ray using the mathematically flat pixel
        ray_opt = self.camera_model.projectPixelTo3dRay((rectified_u, rectified_v))
        # ray_opt = self.camera_model.projectPixelTo3dRay((u, v))

        try:
            t_mount = self.tf_buffer.lookup_transform(
                "base_link", "camera_optical_frame", rclpy.time.Time()
            )
        except Exception:
            return None

        q_m = t_mount.transform.rotation
        r_mount = R_scipy.from_quat([q_m.x, q_m.y, q_m.z, q_m.w])
        cam_offset = (
            t_mount.transform.translation.x,
            t_mount.transform.translation.y,
            t_mount.transform.translation.z,
        )

        q_d = self._drone_pose.pose.orientation
        r_drone = R_scipy.from_quat([q_d.x, q_d.y, q_d.z, q_d.w])
        world_ray = r_drone.apply(r_mount.apply(ray_opt))

        drone_pos = self._drone_pose.pose.position
        cam_world_offset = r_drone.apply(cam_offset)
        cam_z = drone_pos.z + cam_world_offset[2]
        ground_z = (
            self.get_parameter("ground_altitude_m").get_parameter_value().double_value
        )

        if abs(world_ray[2]) < 1e-6:
            return None
        t = (ground_z - cam_z) / world_ray[2]
        if t < 0:
            return None

        target_x = (drone_pos.x + cam_world_offset[0]) + t * world_ray[0]
        target_y = (drone_pos.y + cam_world_offset[1]) + t * world_ray[1]

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


def main():
    rclpy.init()
    node = MissionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
