"""
GPS Waypoint Follower Node

Flies the drone through GPS waypoints using RTK-GPS navigation.
Publishes to /mavros/setpoint_position/global for direct GPS navigation.

Flow:
  1. Wait for MAVROS connection and GPS fix
  2. Takeoff via simple_takeoff service
  3. Subscribe to GPS waypoints
  4. Fly to waypoint using global setpoints
  5. When within arrival radius, call respawn service for next waypoint
  6. Repeat until shutdown

Publications:
  /mavros/setpoint_position/global (GeoPoseStamped) - GPS position setpoints

Subscriptions:
  /drone_control/waypoint/gps (NavSatFix) - GPS waypoint to fly to
  /mavros/global_position/global (NavSatFix) - drone GPS position
  /mavros/state (State) - FCU state

Services Used:
  /drone_control/takeoff (CommandTOL) - takeoff via simple_takeoff node
  /drone_control/respawn_target (Trigger) - request new waypoint
"""

import math
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandTOL, GimbalManagerPitchyaw
from std_srvs.srv import Trigger


# Earth radius in meters (for distance calculations)
EARTH_RADIUS_M = 6371000.0


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate horizontal distance between two GPS points in meters."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)

    a = (math.sin(delta_lat / 2) ** 2 +
         math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return EARTH_RADIUS_M * c


class FlightState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_GPS = auto()
    WAITING_FOR_TAKEOFF_SERVICE = auto()
    TAKING_OFF = auto()
    WAITING_FOR_ALTITUDE = auto()
    WAITING_FOR_WAYPOINT = auto()
    FLYING_TO_WAYPOINT = auto()
    AT_WAYPOINT = auto()
    REQUESTING_NEXT = auto()


class WaypointFollower(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_follower")

        # Parameters
        self.declare_parameter("takeoff_altitude_m", 20.0)
        self.declare_parameter("waypoint_altitude_m", 20.0)  # Altitude for waypoint navigation
        self.declare_parameter("arrival_radius_m", 3.0)  # Horizontal radius to consider "arrived"
        self.declare_parameter("arrival_height_tolerance_m", 2.0)
        self.declare_parameter("hover_time_s", 3.0)  # Time to hover at waypoint before next
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("max_waypoints", 0)  # 0 = unlimited
        self.declare_parameter("return_to_launch", False)  # RTL after max waypoints
        self.declare_parameter("use_noisy_gps", False)  # Use noisy GPS for navigation

        # State
        self._flight_state = FlightState.INIT
        self._mavros_state: Optional[State] = None
        self._drone_gps: Optional[NavSatFix] = None
        self._drone_local_pose: Optional[PoseStamped] = None  # For altitude during takeoff
        self._current_waypoint_gps: Optional[NavSatFix] = None
        self._current_setpoint: Optional[GeoPoseStamped] = None
        self._launch_gps: Optional[NavSatFix] = None
        self._ground_altitude_amsl: Optional[float] = None  # Ground level in AMSL (GPS alt when local z ~= 0)
        self._waypoint_count = 0
        self._hover_start_time = None
        self._takeoff_requested = False
        self._gimbal_set = False
        self._last_state_log_time = None

        # Subscribers
        self.create_subscription(
            State, "/mavros/state", self._on_mavros_state, 10
        )
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._on_drone_gps,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_drone_local_pose,
            qos_profile_sensor_data,
        )
        # Subscribe to GPS waypoints (noisy or true based on parameter)
        use_noisy = self.get_parameter("use_noisy_gps").get_parameter_value().bool_value
        waypoint_topic = "/drone_control/waypoint/gps_noisy" if use_noisy else "/drone_control/waypoint/gps"
        self.create_subscription(
            NavSatFix,
            waypoint_topic,
            self._on_waypoint_gps,
            10,
        )
        self.get_logger().info(f"Subscribing to GPS waypoints on: {waypoint_topic} (noisy={use_noisy})")

        # Publishers - use setpoint_raw/global for GPS navigation with altitude frame control
        self._setpoint_pub = self.create_publisher(
            GlobalPositionTarget, "/mavros/setpoint_raw/global", 10
        )

        # Service clients
        self._takeoff_client = self.create_client(CommandTOL, "/drone_control/takeoff")
        self._respawn_client = self.create_client(Trigger, "/drone_control/respawn_target")
        self._gimbal_client = self.create_client(
            GimbalManagerPitchyaw, "/mavros/gimbal_control/manager/pitchyaw"
        )

        # Timer
        rate = self.get_parameter("setpoint_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._control_loop)

        self.get_logger().info("GPS Waypoint follower initialized")

    def _on_mavros_state(self, msg: State) -> None:
        self._mavros_state = msg

    def _on_drone_gps(self, msg: NavSatFix) -> None:
        self._drone_gps = msg
        if self._launch_gps is None and msg.status.status >= 0:  # Valid fix
            self._launch_gps = msg
            self.get_logger().info(
                f"Launch GPS set: ({msg.latitude:.7f}, {msg.longitude:.7f}, alt={msg.altitude:.1f}m AMSL)"
            )
        
        # Calculate ground AMSL = GPS altitude when drone is ON THE GROUND (local z near 0)
        # IMPORTANT: Only calculate this BEFORE takeoff when local_z is small!
        # If we calculate after takeoff, we get wrong reference altitude.
        if self._ground_altitude_amsl is None and self._drone_local_pose is not None:
            local_z = self._drone_local_pose.pose.position.z
            # Only calibrate when on the ground (local_z < 1m) to avoid calibrating mid-flight
            if abs(local_z) < 1.0:
                self._ground_altitude_amsl = msg.altitude
                self.get_logger().info(
                    f"Ground AMSL calibrated ON GROUND: {self._ground_altitude_amsl:.1f}m (local_z={local_z:.2f}m)"
                )
            else:
                self.get_logger().warn(
                    f"Skipping ground AMSL calibration - drone not on ground (local_z={local_z:.1f}m)"
                )

    def _on_drone_local_pose(self, msg: PoseStamped) -> None:
        self._drone_local_pose = msg

    def _on_waypoint_gps(self, msg: NavSatFix) -> None:
        """Receive GPS waypoint."""
        if self._current_waypoint_gps is None:
            self.get_logger().info(
                f"First GPS waypoint received: ({msg.latitude:.7f}, {msg.longitude:.7f})"
            )
        self._current_waypoint_gps = msg

    def _control_loop(self) -> None:
        """Main state machine."""
        now = self.get_clock().now()
        
        # Log state changes
        self._log_state_periodically(now)

        # Publish GPS setpoints when we're navigating (after takeoff)
        if self._flight_state in (
            FlightState.WAITING_FOR_WAYPOINT,
            FlightState.FLYING_TO_WAYPOINT,
            FlightState.AT_WAYPOINT,
            FlightState.REQUESTING_NEXT,
        ):
            self._publish_setpoint()

        # State machine
        if self._flight_state == FlightState.INIT:
            self._handle_init()
        elif self._flight_state == FlightState.WAITING_FOR_CONNECTION:
            self._handle_waiting_for_connection()
        elif self._flight_state == FlightState.WAITING_FOR_GPS:
            self._handle_waiting_for_gps()
        elif self._flight_state == FlightState.WAITING_FOR_TAKEOFF_SERVICE:
            self._handle_waiting_for_takeoff_service()
        elif self._flight_state == FlightState.TAKING_OFF:
            self._handle_taking_off()
        elif self._flight_state == FlightState.WAITING_FOR_ALTITUDE:
            self._handle_waiting_for_altitude()
        elif self._flight_state == FlightState.WAITING_FOR_WAYPOINT:
            self._handle_waiting_for_waypoint()
        elif self._flight_state == FlightState.FLYING_TO_WAYPOINT:
            self._handle_flying_to_waypoint()
        elif self._flight_state == FlightState.AT_WAYPOINT:
            self._handle_at_waypoint()
        elif self._flight_state == FlightState.REQUESTING_NEXT:
            self._handle_requesting_next()

    def _log_state_periodically(self, now) -> None:
        """Log current state every 5 seconds."""
        if self._last_state_log_time is None:
            self._last_state_log_time = now
            return
        elapsed = (now - self._last_state_log_time).nanoseconds / 1e9
        if elapsed >= 5.0:
            gps_info = ""
            if self._drone_gps:
                gps_info = f", GPS: ({self._drone_gps.latitude:.6f}, {self._drone_gps.longitude:.6f})"
            self.get_logger().info(
                f"State: {self._flight_state.name}, Waypoints: {self._waypoint_count}{gps_info}"
            )
            self._last_state_log_time = now

    def _handle_init(self) -> None:
        """Initialize - wait for GPS."""
        self._transition_to(FlightState.WAITING_FOR_CONNECTION)

    def _handle_waiting_for_connection(self) -> None:
        """Wait for MAVROS connection."""
        if self._mavros_state is None:
            return
        if self._mavros_state.connected:
            self.get_logger().info("MAVROS connected")
            self._transition_to(FlightState.WAITING_FOR_GPS)

    def _handle_waiting_for_gps(self) -> None:
        """Wait for valid GPS fix."""
        if self._drone_gps is None:
            return
        if self._drone_gps.status.status >= 0:  # STATUS_FIX or better
            self.get_logger().info(f"GPS fix acquired: status={self._drone_gps.status.status}")
            self._transition_to(FlightState.WAITING_FOR_TAKEOFF_SERVICE)

    def _handle_waiting_for_takeoff_service(self) -> None:
        """Wait for takeoff service to be available."""
        if not self._takeoff_client.service_is_ready():
            return
        self.get_logger().info("Takeoff service available")
        self._transition_to(FlightState.TAKING_OFF)

    def _handle_taking_off(self) -> None:
        """Call simple_takeoff service to take off."""
        if self._takeoff_requested:
            # Already requested, wait for altitude
            if self._mavros_state is not None and self._mavros_state.armed:
                if self._drone_local_pose is not None and self._drone_local_pose.pose.position.z > 1.0:
                    self.get_logger().info(f"Climbing... alt={self._drone_local_pose.pose.position.z:.1f}m")
                    self._transition_to(FlightState.WAITING_FOR_ALTITUDE)
            return

        # Request takeoff
        alt = self.get_parameter("takeoff_altitude_m").get_parameter_value().double_value
        self.get_logger().info(f"Calling takeoff service for {alt}m...")
        
        req = CommandTOL.Request()
        req.altitude = alt
        future = self._takeoff_client.call_async(req)
        future.add_done_callback(self._on_takeoff_response)
        self._takeoff_requested = True

    def _on_takeoff_response(self, future) -> None:
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Takeoff command accepted")
            else:
                self.get_logger().warn("Takeoff command rejected, will retry")
                self._takeoff_requested = False
        except Exception as e:
            self.get_logger().error(f"Takeoff service call failed: {e}")
            self._takeoff_requested = False

    def _handle_waiting_for_altitude(self) -> None:
        """Wait until we reach takeoff altitude."""
        if self._drone_local_pose is None:
            return

        alt = self.get_parameter("takeoff_altitude_m").get_parameter_value().double_value
        current_alt = self._drone_local_pose.pose.position.z
        
        if current_alt >= alt * 0.9:  # Within 90% of target
            self.get_logger().info(f"Reached altitude {current_alt:.1f}m (relative)")
            
            # Set gimbal to point down
            if not self._gimbal_set:
                self._set_gimbal_down()
            
            # Set initial GPS setpoint to hover at current position with target relative altitude
            if self._drone_gps is not None:
                self._current_setpoint = self._create_gps_setpoint(
                    self._drone_gps.latitude,
                    self._drone_gps.longitude,
                    alt,  # Relative altitude (now handled by FRAME_GLOBAL_REL_ALT)
                )
                self.get_logger().info(
                    f"Hover setpoint: lat={self._drone_gps.latitude:.6f}, lon={self._drone_gps.longitude:.6f}, "
                    f"alt_rel={alt:.1f}m"
                )
            self._transition_to(FlightState.WAITING_FOR_WAYPOINT)

    def _handle_waiting_for_waypoint(self) -> None:
        """Wait for first waypoint."""
        if self._current_waypoint_gps is not None:
            self.get_logger().info("GPS waypoint available, starting navigation")
            self._update_setpoint_to_waypoint()
            self._transition_to(FlightState.FLYING_TO_WAYPOINT)

    def _handle_flying_to_waypoint(self) -> None:
        """Fly towards current waypoint."""
        if self._current_waypoint_gps is None or self._drone_gps is None:
            return

        # Update setpoint
        self._update_setpoint_to_waypoint()

        # Check if arrived
        if self._check_arrival():
            self._waypoint_count += 1
            self.get_logger().info(f"Arrived at GPS waypoint #{self._waypoint_count}")
            self._hover_start_time = self.get_clock().now()
            self._transition_to(FlightState.AT_WAYPOINT)

    def _handle_at_waypoint(self) -> None:
        """Hover at waypoint for configured time."""
        if self._hover_start_time is None:
            self._hover_start_time = self.get_clock().now()
            return

        hover_time = self.get_parameter("hover_time_s").get_parameter_value().double_value
        elapsed = (self.get_clock().now() - self._hover_start_time).nanoseconds / 1e9

        if elapsed >= hover_time:
            # Check if we've hit max waypoints
            max_wp = self.get_parameter("max_waypoints").get_parameter_value().integer_value
            if max_wp > 0 and self._waypoint_count >= max_wp:
                self.get_logger().info(f"Reached max waypoints ({max_wp})")
                if self.get_parameter("return_to_launch").get_parameter_value().bool_value:
                    self._return_to_launch()
                return

            self._transition_to(FlightState.REQUESTING_NEXT)

    def _handle_requesting_next(self) -> None:
        """Request next waypoint from spawner."""
        if not self._respawn_client.service_is_ready():
            self.get_logger().warn("Respawn service not available, waiting...")
            return

        req = Trigger.Request()
        future = self._respawn_client.call_async(req)
        future.add_done_callback(self._on_respawn_response)
        self._transition_to(FlightState.WAITING_FOR_WAYPOINT)
        self._current_waypoint_gps = None  # Clear current waypoint

    def _on_respawn_response(self, future) -> None:
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"New waypoint requested: {result.message}")
            else:
                self.get_logger().warn(f"Respawn failed: {result.message}")
        except Exception as e:
            self.get_logger().error(f"Respawn service call failed: {e}")

    def _update_setpoint_to_waypoint(self) -> None:
        """Update setpoint based on current GPS waypoint."""
        if self._current_waypoint_gps is None:
            return

        # Now using FRAME_GLOBAL_REL_ALT, so altitude is relative to home
        wp_alt_rel = self.get_parameter("waypoint_altitude_m").get_parameter_value().double_value
        
        # Only log occasionally to reduce spam
        if not hasattr(self, '_wp_logged') or not self._wp_logged:
            self.get_logger().info(
                f"Waypoint nav: lat={self._current_waypoint_gps.latitude:.6f}, "
                f"lon={self._current_waypoint_gps.longitude:.6f}, alt_rel={wp_alt_rel:.1f}m"
            )
            self._wp_logged = True
        
        self._current_setpoint = self._create_gps_setpoint(
            self._current_waypoint_gps.latitude,
            self._current_waypoint_gps.longitude,
            wp_alt_rel,  # Relative altitude directly
        )

    def _check_arrival(self) -> bool:
        """Check if drone has arrived at GPS waypoint."""
        if self._drone_gps is None or self._current_waypoint_gps is None:
            return False
        if self._drone_local_pose is None:
            return False

        arrival_radius = self.get_parameter("arrival_radius_m").get_parameter_value().double_value
        height_tol = self.get_parameter("arrival_height_tolerance_m").get_parameter_value().double_value
        wp_alt = self.get_parameter("waypoint_altitude_m").get_parameter_value().double_value

        # Horizontal distance using haversine
        horizontal_dist = haversine_distance(
            self._drone_gps.latitude, self._drone_gps.longitude,
            self._current_waypoint_gps.latitude, self._current_waypoint_gps.longitude
        )
        
        # Altitude check (using local pose for relative altitude)
        current_alt = self._drone_local_pose.pose.position.z
        dz = abs(current_alt - wp_alt)

        # Debug logging every few seconds
        if hasattr(self, '_last_arrival_log') and self._last_arrival_log is not None:
            elapsed = (self.get_clock().now() - self._last_arrival_log).nanoseconds / 1e9
            if elapsed >= 2.0:
                self.get_logger().info(
                    f"Arrival check: dist={horizontal_dist:.1f}m (need <={arrival_radius}m), "
                    f"alt={current_alt:.1f}m vs {wp_alt:.1f}m (dz={dz:.1f}m, need <={height_tol}m)"
                )
                self._last_arrival_log = self.get_clock().now()
        else:
            self._last_arrival_log = self.get_clock().now()

        return horizontal_dist <= arrival_radius and dz <= height_tol

    def _create_gps_setpoint(self, lat: float, lon: float, alt_rel: float) -> GlobalPositionTarget:
        """Create a GPS setpoint message using relative altitude frame.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees  
            alt_rel: Altitude RELATIVE to home/takeoff point in meters
        """
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Use FRAME_GLOBAL_REL_ALT (6) for altitude relative to home
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt_rel  # Now interpreted as relative altitude!
        
        # type_mask: ignore velocity, acceleration, yaw fields
        # Bits: vx(1), vy(2), vz(4), ax(8), ay(16), az(32), force(64), yaw(128), yaw_rate(256)
        # We only want position control, so ignore all velocity/accel/yaw
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY | GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ |
            GlobalPositionTarget.IGNORE_YAW | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        
        return msg

    def _publish_setpoint(self) -> None:
        """Publish current GPS setpoint."""
        if self._current_setpoint is None:
            return
        self._current_setpoint.header.stamp = self.get_clock().now().to_msg()
        self._setpoint_pub.publish(self._current_setpoint)
        
        # Debug: log setpoint altitude periodically
        if not hasattr(self, '_last_setpoint_log') or self._last_setpoint_log is None:
            self._last_setpoint_log = self.get_clock().now()
            alt = self._current_setpoint.altitude
            self.get_logger().info(f"Publishing GPS setpoint with RELATIVE altitude={alt:.1f}m (frame=REL_ALT)")

    def _return_to_launch(self) -> None:
        """Return to launch GPS position."""
        if self._launch_gps is None:
            return
        self.get_logger().info("Returning to launch GPS position")
        alt_rel = self.get_parameter("takeoff_altitude_m").get_parameter_value().double_value
        self._current_setpoint = self._create_gps_setpoint(
            self._launch_gps.latitude,
            self._launch_gps.longitude,
            alt_rel,  # Relative altitude with FRAME_GLOBAL_REL_ALT
        )

    def _transition_to(self, new_state: FlightState) -> None:
        """Transition to a new flight state."""
        if new_state != self._flight_state:
            self.get_logger().info(f"State: {self._flight_state.name} -> {new_state.name}")
            self._flight_state = new_state

    def _set_gimbal_down(self) -> None:
        """Set gimbal to point straight down (-90 degrees pitch)."""
        if not self._gimbal_client.service_is_ready():
            self.get_logger().warn("Gimbal service not available")
            return

        req = GimbalManagerPitchyaw.Request()
        req.pitch = -90.0  # Point straight down
        req.yaw = 0.0
        req.pitch_rate = float('nan')  # NaN means use angle, not rate
        req.yaw_rate = float('nan')
        req.flags = 0

        future = self._gimbal_client.call_async(req)
        future.add_done_callback(self._on_gimbal_response)
        self.get_logger().info("Setting gimbal to point down (-90 deg pitch)")

    def _on_gimbal_response(self, future) -> None:
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Gimbal set successfully")
                self._gimbal_set = True
            else:
                self.get_logger().warn("Gimbal command failed")
        except Exception as e:
            self.get_logger().warn(f"Gimbal service call failed: {e}")


def main() -> None:
    rclpy.init()
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
