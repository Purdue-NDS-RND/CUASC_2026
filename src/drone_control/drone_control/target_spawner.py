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


def _build_digit_sdf(digit: int, cx: float, cy: float, size: float, z: float) -> str:
    """Build SDF elements for a 7-segment style digit.
    
    Segments layout:
       AAA
      F   B
       GGG
      E   C
       DDD
    
    Args:
        digit: 0-9
        cx, cy: center position
        size: overall height of digit
        z: z position
    """
    # Segment dimensions
    seg_len = size * 0.4   # Horizontal segment length
    seg_w = size * 0.12    # Segment width/thickness
    v_len = size * 0.35    # Vertical segment length
    
    # Positions relative to center
    top_y = cy + size * 0.35
    mid_y = cy
    bot_y = cy - size * 0.35
    left_x = cx - seg_len * 0.4
    right_x = cx + seg_len * 0.4
    
    # Which segments are on for each digit (A,B,C,D,E,F,G)
    segments = {
        0: [1,1,1,1,1,1,0],
        1: [0,1,1,0,0,0,0],
        2: [1,1,0,1,1,0,1],
        3: [1,1,1,1,0,0,1],
        4: [0,1,1,0,0,1,1],
        5: [1,0,1,1,0,1,1],
        6: [1,0,1,1,1,1,1],
        7: [1,1,1,0,0,0,0],
        8: [1,1,1,1,1,1,1],
        9: [1,1,1,1,0,1,1],
    }
    
    segs = segments.get(digit, segments[0])
    result = ""
    mat = "<material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>"
    
    # A - top horizontal
    if segs[0]:
        result += (
            f"<link name='seg_a'><pose>{cx:.4f} {top_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # B - top right vertical
    if segs[1]:
        result += (
            f"<link name='seg_b'><pose>{right_x:.4f} {(top_y+mid_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # C - bottom right vertical
    if segs[2]:
        result += (
            f"<link name='seg_c'><pose>{right_x:.4f} {(mid_y+bot_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # D - bottom horizontal
    if segs[3]:
        result += (
            f"<link name='seg_d'><pose>{cx:.4f} {bot_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # E - bottom left vertical
    if segs[4]:
        result += (
            f"<link name='seg_e'><pose>{left_x:.4f} {(mid_y+bot_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # F - top left vertical
    if segs[5]:
        result += (
            f"<link name='seg_f'><pose>{left_x:.4f} {(top_y+mid_y)/2:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_w:.4f} {v_len:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    # G - middle horizontal
    if segs[6]:
        result += (
            f"<link name='seg_g'><pose>{cx:.4f} {mid_y:.4f} {z:.4f} 0 0 0</pose>"
            f"<visual name='v'><geometry><box><size>{seg_len:.4f} {seg_w:.4f} 0.002</size></box></geometry>{mat}</visual></link>"
        )
    
    return result


def _build_bw_target_sdf(model_name: str, size: float = 0.61, digit: int = 1) -> str:
    """Build SDF for black-white GCP target with X pattern and number.
    
    Creates a 24" (0.61m) square target with:
    - White base
    - Black triangles on top and bottom (created with strips)
    - White triangles on left and right  
    - A digit (0-9) with underline in bottom-right quadrant
    
    Args:
        model_name: Name for the model
        size: Side length in meters (default 0.61m = 24")
        digit: Number to display (0-9)
    """
    half = size / 2.0
    thickness = 0.01  # 1cm thick plate
    layer_z = thickness / 2 + 0.001  # Z position for pattern layers
    
    # Build triangles using horizontal strips
    # Each strip gets narrower as we approach the center
    num_strips = 10
    strip_height = half / num_strips
    
    # Generate strips for top triangle (black)
    top_strips = ""
    for i in range(num_strips):
        # y position from center toward top
        y_pos = (i + 0.5) * strip_height
        # Width decreases linearly from full width at top to 0 at center
        # At y=half (top edge), width = size
        # At y=0 (center), width = 0
        progress = 1.0 - (y_pos / half)  # 1 at center, 0 at top
        strip_width = size * (1.0 - progress * 0.95)  # Don't go to 0, leave small tip
        
        top_strips += (
            f"<link name='top_strip_{i}'>"
            f"<pose>0 {y_pos:.4f} {layer_z:.4f} 0 0 0</pose>"
            f"<visual name='visual'>"
            f"<geometry><box><size>{strip_width:.4f} {strip_height:.4f} 0.002</size></box></geometry>"
            f"<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>"
            f"</visual>"
            f"</link>"
        )
    
    # Generate strips for bottom triangle (black)
    bottom_strips = ""
    for i in range(num_strips):
        # y position from center toward bottom (negative y)
        y_pos = -((i + 0.5) * strip_height)
        progress = 1.0 - (abs(y_pos) / half)
        strip_width = size * (1.0 - progress * 0.95)
        
        bottom_strips += (
            f"<link name='bottom_strip_{i}'>"
            f"<pose>0 {y_pos:.4f} {layer_z:.4f} 0 0 0</pose>"
            f"<visual name='visual'>"
            f"<geometry><box><size>{strip_width:.4f} {strip_height:.4f} 0.002</size></box></geometry>"
            f"<material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>"
            f"</visual>"
            f"</link>"
        )
    
    # Number in bottom-right white area (positioned in right triangle)
    num_x = half * 0.65  # More to the right
    num_y = -half * 0.05
    num_size = size * 0.42  # Bigger digit
    
    # Build digit using 7-segment style boxes
    number_elements = _build_digit_sdf(digit, num_x, num_y, num_size, layer_z + 0.001)
    
    # Add underline below the digit (fixed size, not tied to num_size)
    underline_y = num_y - num_size * 0.75
    underline_w = size * 0.18  # Fixed width
    underline_h = size * 0.025  # Fixed height
    number_elements += (
        f"<link name='underline'>"
        f"<pose>{num_x:.4f} {underline_y:.4f} {layer_z + 0.001:.4f} 0 0 0</pose>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{underline_w:.4f} {underline_h:.4f} 0.002</size></box></geometry>"
        f"<material><ambient>0.02 0.02 0.02 1</ambient><diffuse>0.02 0.02 0.02 1</diffuse></material>"
        f"</visual>"
        f"</link>"
    )
    
    return (
        f"<sdf version='1.7'>"
        f"<model name='{model_name}'>"
        f"<static>true</static>"
        
        # Base white plate
        f"<link name='base'>"
        f"<pose>0 0 0 0 0 0</pose>"
        f"<collision name='collision'>"
        f"<geometry><box><size>{size} {size} {thickness}</size></box></geometry>"
        f"</collision>"
        f"<visual name='visual'>"
        f"<geometry><box><size>{size} {size} {thickness}</size></box></geometry>"
        f"<material>"
        f"<ambient>0.95 0.95 0.95 1</ambient>"
        f"<diffuse>0.95 0.95 0.95 1</diffuse>"
        f"</material>"
        f"</visual>"
        f"</link>"
        
        # Top triangle strips
        f"{top_strips}"
        
        # Bottom triangle strips  
        f"{bottom_strips}"
        
        # Number and underline
        f"{number_elements}"
        
        f"</model>"
        f"</sdf>"
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
        # Target type: "red_box" or "bw_target" (black-white GCP marker)
        self.declare_parameter("target_type", "bw_target")
        self.declare_parameter("bw_target_size_m", 0.61)  # 24 inches = 0.61m
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
        self._current_digit = 0  # Random digit for bw_target

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
        target_type = self.get_parameter("target_type").get_parameter_value().string_value
        
        r = math.sqrt(random.random()) * radius
        angle = random.random() * 2.0 * math.pi
        ox, oy, oz = self._origin

        x = ox + r * math.cos(angle)
        y = oy + r * math.sin(angle)
        
        # Set z height based on target type
        if target_type == "bw_target":
            # Flat target sits just above ground (0.01m to avoid z-fighting)
            obj_z = oz + 0.01
        else:
            # Box uses object_z parameter (typically half box height)
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
        target_type = self.get_parameter("target_type").get_parameter_value().string_value

        if not model_sdf:
            if target_type == "bw_target":
                # Black-white GCP target with random digit
                self._current_digit = random.randint(0, 9)
                bw_size = self.get_parameter("bw_target_size_m").get_parameter_value().double_value
                model_sdf = _build_bw_target_sdf(model_name, bw_size, self._current_digit)
                self.get_logger().info(f"Spawning BW target with digit {self._current_digit}")
            else:
                # Default red box
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