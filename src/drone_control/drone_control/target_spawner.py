import math
import random
import subprocess
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _build_box_sdf(model_name: str, size, rgba) -> str:
    sx, sy, sz = size
    r, g, b, a = rgba
    return (
        "<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        "<static>true</static>"
        "<link name='link'>"
        "<collision name='collision'>"
        "<geometry>"
        f"<box><size>{sx} {sy} {sz}</size></box>"
        "</geometry>"
        "</collision>"
        "<visual name='visual'>"
        "<geometry>"
        f"<box><size>{sx} {sy} {sz}</size></box>"
        "</geometry>"
        "<material>"
        "<ambient>"
        f"{r} {g} {b} {a}"
        "</ambient>"
        "<diffuse>"
        f"{r} {g} {b} {a}"
        "</diffuse>"
        "</material>"
        "</visual>"
        "</link>"
        "</model>"
        "</sdf>"
    )


class TargetSpawner(Node):
    def __init__(self) -> None:
        super().__init__("target_spawner")

        self.declare_parameter("spawn_model", True)
        self.declare_parameter("world_name", "map")
        self.declare_parameter("model_name", "target_box")
        self.declare_parameter("model_sdf", "")
        self.declare_parameter("box_size", [1.0, 1.0, 1.0])
        self.declare_parameter("box_color_rgba", [1.0, 0.2, 0.2, 1.0])
        self.declare_parameter("object_z", 0.5)  # Half box height so it sits on ground
        self.declare_parameter("target_altitude_m", 20.0)
        self.declare_parameter("radius_m", 30.0)
        self.declare_parameter("hover_radius_m", 2.0)
        self.declare_parameter("hover_height_tolerance_m", 1.5)
        self.declare_parameter("hover_duration_s", 5.0)
        self.declare_parameter("publish_gps", True)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("random_seed", 0)
        # Accuracy simulation parameters
        self.declare_parameter("simulate_accuracy", True)
        self.declare_parameter("max_noise_m", 5.0)  # Max noise when far away
        self.declare_parameter("min_noise_m", 0.1)  # Min noise when close
        self.declare_parameter("noise_falloff_dist_m", 50.0)  # Distance at which noise is max

        seed = self.get_parameter("random_seed").get_parameter_value().integer_value
        if seed != 0:
            random.seed(seed)

        self._origin: Optional[Tuple[float, float, float]] = None
        self._drone_pose: Optional[PoseStamped] = None
        self._target_pose: Optional[PoseStamped] = None  # True position
        self._hover_start = None
        self._home: Optional[HomePosition] = None
        self._spawned_for_target = False
        self._spawn_counter = 0

        self._target_pose_pub = self.create_publisher(
            PoseStamped, "/drone_control/target/pose", 10
        )
        self._target_noisy_pose_pub = self.create_publisher(
            PoseStamped, "/drone_control/target/pose_noisy", 10
        )
        self._target_gps_pub = self.create_publisher(
            NavSatFix, "/drone_control/target/gps", 10
        )
        # GPS waypoint for waypoint_follower (GPS-based navigation)
        self._waypoint_gps_pub = self.create_publisher(
            NavSatFix, "/drone_control/waypoint/gps", 10
        )
        self._status_pub = self.create_publisher(String, "/drone_control/target/status", 10)

        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_local_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            HomePosition,
            "/mavros/home_position/home",
            self._on_home,
            10,
        )

        # Service for external respawn requests (from waypoint_follower)
        self._respawn_service = self.create_service(
            Trigger,
            "/drone_control/respawn_target",
            self._on_respawn_request,
        )

        self._timer = self.create_timer(0.1, self._on_timer)

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self._drone_pose = msg
        if self._origin is None:
            p = msg.pose.position
            self._origin = (p.x, p.y, p.z)
            self.get_logger().info(
                f"Set local origin at x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}"
            )
            self._spawn_new_target()

    def _on_home(self, msg: HomePosition) -> None:
        self._home = msg

    def _on_respawn_request(self, request, response) -> Trigger.Response:
        """Handle external respawn request from waypoint_follower."""
        if self._origin is None:
            response.success = False
            response.message = "No origin set yet"
            return response

        self.get_logger().info("Respawn requested via service")
        self._delete_current_model()
        self._spawn_new_target()
        
        response.success = True
        response.message = f"Spawned target #{self._spawn_counter}"
        return response

    def _delete_current_model(self) -> None:
        """Delete the current target model from Gazebo."""
        if self._spawn_counter == 0:
            return

        world_name = self.get_parameter("world_name").get_parameter_value().string_value
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        entity_name = f"{model_name}_{self._spawn_counter}"

        cmd = [
            "gz", "service",
            "-s", f"/world/{world_name}/remove",
            "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000",
            "--req", f'name: "{entity_name}" type: MODEL'
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
            if result.returncode == 0:
                self.get_logger().info(f"Deleted model '{entity_name}'")
            else:
                self.get_logger().debug(f"Delete may have failed: {result.stderr}")
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            self.get_logger().debug(f"Delete command issue: {e}")

    def _on_timer(self) -> None:
        if self._target_pose is None:
            return

        now = self.get_clock().now().to_msg()
        self._target_pose.header.stamp = now
        self._target_pose_pub.publish(self._target_pose)

        # Publish noisy pose (accuracy degrades with distance)
        if self.get_parameter("simulate_accuracy").get_parameter_value().bool_value:
            noisy_pose = self._get_noisy_pose()
            if noisy_pose is not None:
                noisy_pose.header.stamp = now
                self._target_noisy_pose_pub.publish(noisy_pose)

        if self.get_parameter("publish_gps").get_parameter_value().bool_value:
            gps = self._target_to_gps()
            if gps is not None:
                self._target_gps_pub.publish(gps)
                # Also publish as waypoint for waypoint_follower (at target_altitude above target)
                waypoint_gps = self._target_to_waypoint_gps()
                if waypoint_gps is not None:
                    self._waypoint_gps_pub.publish(waypoint_gps)

        self._check_hover_and_respawn()
        self._try_spawn_current_target()

    def _get_noisy_pose(self) -> Optional[PoseStamped]:
        """Add distance-based noise to simulate sensor accuracy degradation."""
        if self._drone_pose is None or self._target_pose is None:
            return None

        # Calculate distance from drone to target
        dx = self._drone_pose.pose.position.x - self._target_pose.pose.position.x
        dy = self._drone_pose.pose.position.y - self._target_pose.pose.position.y
        dz = self._drone_pose.pose.position.z - self._target_pose.pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Get noise parameters
        max_noise = self.get_parameter("max_noise_m").get_parameter_value().double_value
        min_noise = self.get_parameter("min_noise_m").get_parameter_value().double_value
        falloff_dist = self.get_parameter("noise_falloff_dist_m").get_parameter_value().double_value

        # Linear interpolation: far = max noise, close = min noise
        t = min(distance / max(falloff_dist, 0.1), 1.0)
        noise_std = min_noise + t * (max_noise - min_noise)

        # Create noisy pose
        noisy = PoseStamped()
        noisy.header.frame_id = self._target_pose.header.frame_id
        noisy.pose.position.x = self._target_pose.pose.position.x + random.gauss(0, noise_std)
        noisy.pose.position.y = self._target_pose.pose.position.y + random.gauss(0, noise_std)
        noisy.pose.position.z = self._target_pose.pose.position.z + random.gauss(0, noise_std * 0.5)
        noisy.pose.orientation.w = 1.0
        return noisy

    def _target_to_gps(self) -> Optional[NavSatFix]:
        if self._home is None or self._target_pose is None:
            return None
        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude
        alt0 = self._home.geo.altitude

        x = self._target_pose.pose.position.x
        y = self._target_pose.pose.position.y
        z = self._target_pose.pose.position.z

        lat_rad = math.radians(lat0)
        meters_per_deg = 111111.0
        dlat = y / meters_per_deg
        dlon = x / (meters_per_deg * max(math.cos(lat_rad), 1e-6))

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.latitude = lat0 + dlat
        msg.longitude = lon0 + dlon
        msg.altitude = alt0 + z
        msg.status.status = msg.status.STATUS_FIX
        msg.status.service = msg.status.SERVICE_GPS
        return msg

    def _target_to_waypoint_gps(self) -> Optional[NavSatFix]:
        """Convert target position to GPS waypoint at target_altitude above ground."""
        if self._home is None or self._target_pose is None:
            return None
        lat0 = self._home.geo.latitude
        lon0 = self._home.geo.longitude
        alt0 = self._home.geo.altitude

        x = self._target_pose.pose.position.x
        y = self._target_pose.pose.position.y
        # Waypoint is at target_altitude above ground (not above target object)
        target_alt = self.get_parameter("target_altitude_m").get_parameter_value().double_value

        lat_rad = math.radians(lat0)
        meters_per_deg = 111111.0
        dlat = y / meters_per_deg
        dlon = x / (meters_per_deg * max(math.cos(lat_rad), 1e-6))

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.latitude = lat0 + dlat
        msg.longitude = lon0 + dlon
        msg.altitude = alt0 + target_alt  # Altitude is absolute (home + target altitude)
        msg.status.status = msg.status.STATUS_FIX
        msg.status.service = msg.status.SERVICE_GPS
        return msg

    def _check_hover_and_respawn(self) -> None:
        if self._drone_pose is None or self._target_pose is None:
            return

        hover_radius = self.get_parameter("hover_radius_m").get_parameter_value().double_value
        hover_height_tol = (
            self.get_parameter("hover_height_tolerance_m")
            .get_parameter_value()
            .double_value
        )
        hover_duration = self.get_parameter("hover_duration_s").get_parameter_value().double_value
        target_altitude = (
            self._target_pose.pose.position.z
            + self.get_parameter("target_altitude_m").get_parameter_value().double_value
        )

        dx = self._drone_pose.pose.position.x - self._target_pose.pose.position.x
        dy = self._drone_pose.pose.position.y - self._target_pose.pose.position.y
        dz = self._drone_pose.pose.position.z - target_altitude
        horizontal = math.hypot(dx, dy)

        within = horizontal <= hover_radius and abs(dz) <= hover_height_tol
        now = self.get_clock().now()
        if within:
            if self._hover_start is None:
                self._hover_start = now
            else:
                elapsed = (now - self._hover_start).nanoseconds / 1e9
                if elapsed >= hover_duration:
                    self.get_logger().info("Hover complete, spawning new target")
                    self._spawn_new_target()
                    self._hover_start = None
        else:
            self._hover_start = None

    def _spawn_new_target(self) -> None:
        if self._origin is None:
            return

        radius = self.get_parameter("radius_m").get_parameter_value().double_value
        r = math.sqrt(random.random()) * radius
        angle = random.random() * 2.0 * math.pi
        ox, oy, oz = self._origin

        x = ox + r * math.cos(angle)
        y = oy + r * math.sin(angle)
        obj_z = oz + self.get_parameter("object_z").get_parameter_value().double_value

        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = obj_z
        pose.pose.orientation.w = 1.0
        self._target_pose = pose

        self._publish_status(f"New target at x={x:.2f}, y={y:.2f}, z={obj_z:.2f}")
        self._spawned_for_target = False
        self._spawn_counter += 1

    def _try_spawn_current_target(self) -> None:
        if self._spawned_for_target:
            return
        if self._target_pose is None:
            return
        if not self.get_parameter("spawn_model").get_parameter_value().bool_value:
            return
        self._spawn_via_gz_cli()

    def _spawn_via_gz_cli(self) -> None:
        """Spawn model using gz service CLI (works without ROS bridge)."""
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        world_name = self.get_parameter("world_name").get_parameter_value().string_value
        model_sdf = self.get_parameter("model_sdf").get_parameter_value().string_value

        if not model_sdf:
            size = self.get_parameter("box_size").get_parameter_value().double_array_value
            rgba = self.get_parameter("box_color_rgba").get_parameter_value().double_array_value
            if len(size) != 3:
                size = [1.0, 1.0, 1.0]
            if len(rgba) != 4:
                rgba = [1.0, 0.2, 0.2, 1.0]
            model_sdf = _build_box_sdf(model_name, size, rgba)

        # Unique name for each spawn
        unique_name = f"{model_name}_{self._spawn_counter}"

        x = self._target_pose.pose.position.x
        y = self._target_pose.pose.position.y
        z = self._target_pose.pose.position.z

        # Escape quotes for shell
        sdf_escaped = model_sdf.replace('"', '\\"')

        req_msg = (
            f'sdf: "{sdf_escaped}" '
            f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}}} '
            f'name: "{unique_name}" '
            f'allow_renaming: true'
        )

        cmd = [
            "gz", "service",
            "-s", f"/world/{world_name}/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", req_msg
        ]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
            if result.returncode == 0 and "true" in result.stdout.lower():
                self.get_logger().info(f"Spawned '{unique_name}' at ({x:.2f}, {y:.2f}, {z:.2f})")
                self._spawned_for_target = True
            else:
                self.get_logger().warn(f"Spawn failed: {result.stderr or result.stdout}")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Spawn command timed out")
        except FileNotFoundError:
            self.get_logger().error("'gz' command not found - is Gazebo installed?")

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TargetSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()