"""
Payload Drop Mission — State Machine Node

Executes a precision payload drop using GPS navigation and visual
target tracking.  The drone always holds yaw = 0 (north) so the
downward-facing camera stays orientation-stable.

Assumes the drone is ALREADY AIRBORNE at ~20 m (e.g. via simple_takeoff).
Launch this node after takeoff is complete.

State flow:
  INIT ──► WAITING_FOR_CONNECTION ──► WAITING_FOR_GPS
    ──► TAKEOFF ──► TRANSIT_TO_TARGET ──► ACQUIRE_TARGET
        ├─► TARGET_NOT_FOUND  (stub — recovery TBD)
        └─► CENTER_ON_TARGET ──► DESCEND ──► DROP_PAYLOAD
              ──► RETURN_TO_LAUNCH ──► COMPLETE

Parameters:
  target_latitude          GPS latitude of drop zone
  target_longitude         GPS longitude of drop zone
  transit_altitude_m       Altitude for flying to GPS target  (default 20.0)
  takeoff_altitude_m       Altitude to climb to during takeoff (default 20.0)
  drop_altitude_m          Altitude to descend to before drop (default 5.0)
  centering_tolerance_px   Pixel‐distance from image centre   (default 30.0)
  arrival_radius_m         Horizontal GPS arrival radius      (default 3.0)
  arrival_alt_tolerance_m  Vertical tolerance for "arrived"   (default 2.0)
  servo_channel            Servo channel for drop mechanism   (default 9)
  servo_open_pwm           PWM value to open / release        (default 1900)
  servo_close_pwm          PWM value to close / secure        (default 1100)
  setpoint_rate_hz         Control‐loop / setpoint rate       (default 20.0)
  image_width_px           Camera image width in pixels       (default 640)
  image_height_px          Camera image height in pixels      (default 480)
  guided_mode_name         ArduPilot GUIDED mode string       (default "GUIDED")

Subscriptions:
  /mavros/state                         (State)
  /mavros/global_position/global        (NavSatFix)
  /mavros/local_position/pose           (PoseStamped)
  /payload_drop/target_detection        (PointStamped)  — x,y pixel of target

Publications:
  /mavros/setpoint_raw/global           (GlobalPositionTarget)

Services used:
  drone_utils/takeoff                   (CommandTOL)              — arm + takeoff via simple_takeoff_service
  /mavros/set_mode                      (SetMode)
  /mavros/cmd/command                   (CommandLong)             — MAV_CMD_DO_SET_SERVO
  drone_utils/set_gimbal_point          (GimbalManagerPitchyaw)   — point camera straight down

Extra parameters:
  not_found_ascent_m    How many metres to climb when target not found  (default 5.0)
"""

import math
from enum import Enum, auto
from typing import Optional

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GlobalPositionTarget, PositionTarget
from mavros_msgs.srv import SetMode, CommandLong, CommandTOL, GimbalManagerPitchyaw

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------

EARTH_RADIUS_M = 6_371_000.0


def haversine_distance(
    lat1: float, lon1: float, lat2: float, lon2: float
) -> float:
    """Horizontal distance (m) between two GPS points."""
    lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2) ** 2
    )
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ---------------------------------------------------------------------------
#  State enum
# ---------------------------------------------------------------------------


class DropState(Enum):
    """All states for the payload‐drop mission."""

    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_GPS = auto()
    TAKEOFF = auto()
    TRANSIT_TO_TARGET = auto()
    ACQUIRE_TARGET = auto()
    TARGET_NOT_FOUND = auto()
    CENTER_ON_TARGET = auto()
    DESCEND = auto()
    DROP_PAYLOAD = auto()
    RETURN_TO_LAUNCH = auto()
    COMPLETE = auto()


# ---------------------------------------------------------------------------
#  Node
# ---------------------------------------------------------------------------


class PayloadDrop(Node):
    def __init__(self) -> None:
        super().__init__("payload_drop")

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter("target_latitude", 0.0)
        self.declare_parameter("target_longitude", 0.0)
        self.declare_parameter("transit_altitude_m", 20.0)
        self.declare_parameter("takeoff_altitude_m", 20.0)
        self.declare_parameter("drop_altitude_m", 5.0)
        self.declare_parameter("centering_tolerance_px", 30.0)
        self.declare_parameter("arrival_radius_m", 3.0)
        self.declare_parameter("arrival_alt_tolerance_m", 2.0)
        self.declare_parameter("servo_channel", 9)
        self.declare_parameter("servo_open_pwm", 1900)
        self.declare_parameter("servo_close_pwm", 1100)
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("guided_mode_name", "GUIDED")
        self.declare_parameter("not_found_ascent_m", 5.0)

        # ── Internal state ───────────────────────────────────────────
        self._drop_state = DropState.INIT
        self._mavros_state: Optional[State] = None
        self._drone_gps: Optional[NavSatFix] = None
        self._drone_local_pose: Optional[PoseStamped] = None
        self._target_pixel: Optional[PointStamped] = None   # latest detection
        self._image_dims: Optional[tuple[int, int]] = None  # (w, h) from stream
        self._current_setpoint: Optional[GlobalPositionTarget] = None
        self._last_state_log_time = None
        self._takeoff_requested = False
        self._gimbal_pointed = False
        self._drop_actuated = False
        self._recovery_altitude: Optional[float] = None

        # ── Subscribers ──────────────────────────────────────────────
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
            self._on_local_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            "/drone_package_drop/target_detection",
            self._on_target_detection,
            10,
        )
        self.create_subscription(
            PointStamped,
            "/drone_package_drop/image_size",
            self._on_image_size,
            10,
        )

        # ── Publishers ───────────────────────────────────────────
        self._setpoint_pub = self.create_publisher(
            GlobalPositionTarget, "/mavros/setpoint_raw/global", 10
        )
        # Used during CENTER_ON_TARGET: local NED velocity + yaw-lock
        self._local_setpoint_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )

        # ── Service clients ──────────────────────────────────────────
        self._takeoff_client = self.create_client(CommandTOL, "drone_utils/takeoff")
        self._mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._command_client = self.create_client(CommandLong, "/mavros/cmd/command")
        self._gimbal_client = self.create_client(
            GimbalManagerPitchyaw, "drone_utils/set_gimbal_point"
        )

        # ── Timer (main control loop) ────────────────────────────────
        rate = (
            self.get_parameter("setpoint_rate_hz")
            .get_parameter_value()
            .double_value
        )
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._control_loop)

        self.get_logger().info("PayloadDrop node initialised — waiting for data")

    # ==================================================================
    #  Topic callbacks
    # ==================================================================

    def _on_mavros_state(self, msg: State) -> None:
        self._mavros_state = msg

    def _on_drone_gps(self, msg: NavSatFix) -> None:
        self._drone_gps = msg

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self._drone_local_pose = msg

    def _on_target_detection(self, msg: PointStamped) -> None:
        """Vision pipeline publishes pixel (x, y) of the target centre."""
        self._target_pixel = msg

    def _on_image_size(self, msg: PointStamped) -> None:
        """Receive image dimensions published by target_cv."""
        self._image_dims = (int(msg.point.x), int(msg.point.y))

    # ==================================================================
    #  Main control loop
    # ==================================================================

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        self._log_state_periodically(now)

        # Always republish the setpoint so MAVROS doesn't time out
        self._publish_setpoint()

        # ── State dispatch ────────────────────────────────────────
        handler = {
            DropState.INIT: self._handle_init,
            DropState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            DropState.WAITING_FOR_GPS: self._handle_waiting_for_gps,
            DropState.TAKEOFF: self._handle_takeoff,
            DropState.TRANSIT_TO_TARGET: self._handle_transit_to_target,
            DropState.ACQUIRE_TARGET: self._handle_acquire_target,
            DropState.TARGET_NOT_FOUND: self._handle_target_not_found,
            DropState.CENTER_ON_TARGET: self._handle_center_on_target,
            DropState.DESCEND: self._handle_descend,
            DropState.DROP_PAYLOAD: self._handle_drop_payload,
            DropState.RETURN_TO_LAUNCH: self._handle_return_to_launch,
            DropState.COMPLETE: lambda: None,
        }.get(self._drop_state, lambda: None)
        handler()

    # ==================================================================
    #  State handlers  (fill these in!)
    # ==================================================================

    def _handle_init(self) -> None:
        """Point gimbal down and wait for MAVROS connection."""
        if self._mavros_state is None:
            return

        # ── Point gimbal straight down (once) ────────────────────
        if not self._gimbal_pointed and self._gimbal_client.service_is_ready():
            req = GimbalManagerPitchyaw.Request()
            req.pitch = float(-90.0)  # = straight down
            req.yaw = float(0.0)
            req.pitch_rate = float('nan')
            req.yaw_rate = float('nan')
            req.flags = 0
            self._gimbal_client.call_async(req)
            self._gimbal_pointed = True
            self.get_logger().info("Gimbal pointed straight down")

        self._transition_to(DropState.WAITING_FOR_CONNECTION)

    def _handle_waiting_for_connection(self) -> None:
        """Wait until MAVROS reports connected."""
        if self._mavros_state is None:
            return
        if not self._mavros_state.connected:
            return

        self.get_logger().info("MAVROS connected")
        self._transition_to(DropState.WAITING_FOR_GPS)

    def _handle_waiting_for_gps(self) -> None:
        """Wait for a valid GPS fix."""
        if self._drone_gps is None:
            return
        if self._drone_gps.status.status < 0:  # no fix yet
            return

        self.get_logger().info(
            f"GPS fix acquired — "
            f"({self._drone_gps.latitude:.7f}, {self._drone_gps.longitude:.7f})"
        )
        self._transition_to(DropState.TAKEOFF)

    # ── TAKEOFF ───────────────────────────────────────────────────────

    def _handle_takeoff(self) -> None:
        """Call simple_takeoff service and wait until takeoff altitude is reached.

        Matches the waypoint_follower pattern:
          1. Wait for service ready
          2. Call service once; retry automatically if rejected
          3. Once armed and climbing, wait for >= 90 % of target altitude
          4. Transition to TRANSIT_TO_TARGET
        """
        takeoff_alt = (
            self.get_parameter("takeoff_altitude_m")
            .get_parameter_value()
            .double_value
        )

        # Already airborne and near takeoff altitude → skip
        if self._drone_local_pose is not None:
            current_alt = self._drone_local_pose.pose.position.z
            if current_alt >= takeoff_alt * 0.9:
                self.get_logger().info(
                    f"Already at {current_alt:.1f} m — skipping takeoff"
                )
                self._transition_to(DropState.TRANSIT_TO_TARGET)
                return

        if self._takeoff_requested:
            # Already sent — wait for armed + climbing, then check altitude
            if self._mavros_state is not None and self._mavros_state.armed:
                if self._drone_local_pose is not None:
                    current_alt = self._drone_local_pose.pose.position.z
                    self.get_logger().info(
                        f"Climbing... alt={current_alt:.1f} m / {takeoff_alt:.1f} m"
                    )
                    if current_alt >= takeoff_alt * 0.9:
                        self.get_logger().info(
                            f"Reached takeoff altitude {current_alt:.1f} m — proceeding to target"
                        )
                        self._transition_to(DropState.TRANSIT_TO_TARGET)
            return

        # Service not ready yet — keep waiting
        if not self._takeoff_client.service_is_ready():
            self.get_logger().warn("Waiting for drone_utils/takeoff service...")
            return

        # Send the takeoff request
        req = CommandTOL.Request()
        req.altitude = takeoff_alt
        self.get_logger().info(f"Calling takeoff service for {takeoff_alt:.1f} m...")
        future = self._takeoff_client.call_async(req)
        future.add_done_callback(self._on_takeoff_response)
        self._takeoff_requested = True

    def _on_takeoff_response(self, future) -> None:
        """Handle response from drone_utils/takeoff — retry on rejection."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Takeoff command accepted")
            else:
                self.get_logger().warn("Takeoff command rejected — will retry")
                self._takeoff_requested = False
        except Exception as e:
            self.get_logger().error(f"Takeoff service call failed: {e}")
            self._takeoff_requested = False

    # ── TRANSIT_TO_TARGET ─────────────────────────────────────────────

    def _handle_transit_to_target(self) -> None:
        """Fly to the target GPS coordinate at transit altitude."""

        # ── read params ──────────────────────────────────────────
        target_lat = self.get_parameter("target_latitude").get_parameter_value().double_value
        target_lon = self.get_parameter("target_longitude").get_parameter_value().double_value
        transit_alt = self.get_parameter("transit_altitude_m").get_parameter_value().double_value
        arrival_r = self.get_parameter("arrival_radius_m").get_parameter_value().double_value
        alt_tol = self.get_parameter("arrival_alt_tolerance_m").get_parameter_value().double_value

        # ── create / update setpoint (yaw free during long transit) ─
        self._current_setpoint = self._create_gps_setpoint(
            target_lat, target_lon, transit_alt, yaw=math.pi / 2, lock_yaw=False
        )

        # ── check arrival ────────────────────────────────────────
        if self._drone_gps is None or self._drone_local_pose is None:
            return

        ground_distance = haversine_distance(
            self._drone_gps.latitude, self._drone_gps.longitude,
            target_lat, target_lon,
        )
        dz = abs(self._drone_local_pose.pose.position.z - transit_alt)

        if ground_distance <= arrival_r and dz <= alt_tol:
            self.get_logger().info("Arrived at target GPS — acquiring target")
            self._transition_to(DropState.ACQUIRE_TARGET)

    # ── ACQUIRE_TARGET ────────────────────────────────────────────────

    def _handle_acquire_target(self) -> None:
        """Check whether the visual target is in the camera frame."""
        if self._target_pixel is not None:
            # Check the detection is recent (< 2 s old)
            age = (
                self.get_clock().now() - rclpy.time.Time.from_msg(self._target_pixel.header.stamp)
            ).nanoseconds / 1e9
            if age < 2.0:
                self.get_logger().info("Target detected in frame — centering")
                self._transition_to(DropState.CENTER_ON_TARGET)
                return

        self.get_logger().info("Target not visible — entering recovery")
        self._transition_to(DropState.TARGET_NOT_FOUND)

    # ── TARGET_NOT_FOUND ──────────────────────────────────────────────

    def _handle_target_not_found(self) -> None:
        """Climb to gain more field of view, then retry acquisition."""
        if self._drone_local_pose is None:
            return

        ascent = (
            self.get_parameter("not_found_ascent_m")
            .get_parameter_value()
            .double_value
        )
        alt_tol = (
            self.get_parameter("arrival_alt_tolerance_m")
            .get_parameter_value()
            .double_value
        )

        # Set the recovery altitude once on entry
        if self._recovery_altitude is None:
            current_alt = self._drone_local_pose.pose.position.z
            self._recovery_altitude = current_alt + ascent
            self.get_logger().info(
                f"Target not found — climbing {ascent:.1f} m to "
                f"{self._recovery_altitude:.1f} m to widen FOV"
            )

        # Keep commanding the recovery altitude (lat/lon stays from last setpoint)
        if self._current_setpoint is not None:
            self._current_setpoint.altitude = self._recovery_altitude

        # Wait until we reach it, then retry
        dz = abs(self._drone_local_pose.pose.position.z - self._recovery_altitude)
        if dz <= alt_tol:
            self.get_logger().info(
                f"Reached {self._recovery_altitude:.1f} m — retrying target acquisition"
            )
            self._recovery_altitude = None
            self._transition_to(DropState.ACQUIRE_TARGET)

    # ── CENTER_ON_TARGET ──────────────────────────────────────────────

    def _handle_center_on_target(self) -> None:
        """Centre the target in the image using local-frame velocity commands.

        Drone is always pointed north (yaw = 0 in NED) so the camera axes map
        cleanly to NED:
          • +pixel X (target right of centre) → fly East  (+vy in NED)
          • +pixel Y (target below centre)    → fly South (-vx in NED)

        Publishes PositionTarget on /mavros/setpoint_raw/local instead of
        touching the GPS setpoint, which makes fine centering far more precise.
        The global setpoint publisher is suppressed while in this state.
        """
        tol = self.get_parameter("centering_tolerance_px").get_parameter_value().double_value

        # Lost the target → hover in place and go back to ACQUIRE
        if self._target_pixel is None:
            self.get_logger().warn("Lost target during centering")
            self._publish_zero_local_velocity()
            self._transition_to(DropState.ACQUIRE_TARGET)
            return

        cx, cy = self._image_centre()
        ex = self._target_pixel.point.x - cx   # positive = target is right of centre
        ey = self._target_pixel.point.y - cy   # positive = target is below centre

        pixel_error = math.hypot(ex, ey)
        if pixel_error <= tol:
            self.get_logger().info("Target centred — beginning descent")
            self._publish_zero_local_velocity()
            self._transition_to(DropState.DESCEND)
            return

        # ── Proportional velocity command (local NED, yaw = 0 = North) ──
        # Tune gain_mps_per_px for your camera FOV and altitude.
        gain_mps_per_px = 0.01        # m/s per pixel of error
        max_centering_speed_mps = 1.0  # cap so we don't overshoot

        vx =  -ey * gain_mps_per_px   # NED North (+ey → south → negative vx)
        vy =   ex * gain_mps_per_px   # NED East  (+ex → east → positive vy)

        speed = math.hypot(vx, vy)
        if speed > max_centering_speed_mps:
            scale = max_centering_speed_mps / speed
            vx *= scale
            vy *= scale

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # command velocity only; hold altitude (vz=0); lock yaw to north
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = 0.0   # hold altitude
        msg.yaw = math.pi / 2  # ENU: π/2 = North — always face north
        self._local_setpoint_pub.publish(msg)
        self.get_logger().info(
            f"Centering on target: pixel error={pixel_error:.1f} px, "
            f"commanding velocity (vx={vx:.2f} m/s, vy={vy:.2f} m/s)"
        )

    # ── DESCEND ───────────────────────────────────────────────────────

    def _handle_descend(self) -> None:
        """Descend straight down to drop_altitude_m, keeping lat/lon locked."""
        drop_alt = self.get_parameter("drop_altitude_m").get_parameter_value().double_value
        alt_tol = self.get_parameter("arrival_alt_tolerance_m").get_parameter_value().double_value

        # Update only the altitude in the existing setpoint
        if self._current_setpoint is not None:
            self._current_setpoint.altitude = drop_alt

        if self._drone_local_pose is None:
            return

        dz = abs(self._drone_local_pose.pose.position.z - drop_alt)
        if dz <= alt_tol:
            self.get_logger().info(f"At drop altitude ({drop_alt:.1f} m) — releasing payload")
            self._transition_to(DropState.DROP_PAYLOAD)

    # ── DROP_PAYLOAD ──────────────────────────────────────────────────

    def _handle_drop_payload(self) -> None:
        """Actuate the servo to release the payload."""
        if not self._drop_actuated:
            ch  = self.get_parameter("servo_channel").get_parameter_value().integer_value
            pwm = self.get_parameter("servo_open_pwm").get_parameter_value().integer_value
            self._send_servo_command(ch, pwm)
            self._drop_actuated = True
            self.get_logger().info("Payload released!")
            self._transition_to(DropState.RETURN_TO_LAUNCH)

    # ── RETURN_TO_LAUNCH ──────────────────────────────────────────────

    def _handle_return_to_launch(self) -> None:
        """Switch flight mode to RTL."""
        self._request_mode("RTL")
        self.get_logger().info("RTL requested — mission complete")
        self._transition_to(DropState.COMPLETE)

    # ==================================================================
    #  Helper utilities
    # ==================================================================

    def _create_gps_setpoint(
        self, lat: float, lon: float, alt_rel: float,
        yaw: float = math.pi / 2, lock_yaw: bool = True,
    ) -> GlobalPositionTarget:
        """Build a GlobalPositionTarget (FRAME_GLOBAL_REL_ALT).

        yaw is in radians — 0.0 = North.
        lock_yaw=True  → enforce the yaw value (north by default).
        lock_yaw=False → ignore yaw, let the autopilot choose heading
                         (better for long transits).
        """
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt_rel
        msg.yaw = yaw  # ENU convention: 0 = East, π/2 = North

        # Ignore velocity / acceleration / yaw-rate; optionally ignore yaw too
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        if not lock_yaw:
            msg.type_mask |= GlobalPositionTarget.IGNORE_YAW
        return msg

    def _publish_setpoint(self) -> None:
        """Re-stamp and publish the current setpoint to keep MAVROS alive.

        Suppressed during CENTER_ON_TARGET because that state drives the drone
        with local-frame velocity commands instead of a global GPS setpoint.
        """
        if self._drop_state == DropState.CENTER_ON_TARGET:
            return
        if self._current_setpoint is None:
            return
        self._current_setpoint.header.stamp = self.get_clock().now().to_msg()
        self._setpoint_pub.publish(self._current_setpoint)

    def _publish_zero_local_velocity(self) -> None:
        """Publish a zero-velocity local command to hold position (hover)."""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX
            | PositionTarget.IGNORE_PY
            | PositionTarget.IGNORE_PZ
            | PositionTarget.IGNORE_AFX
            | PositionTarget.IGNORE_AFY
            | PositionTarget.IGNORE_AFZ
            | PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.yaw = float(90.0)  # ENU: 90° = North
        self._local_setpoint_pub.publish(msg)

    def _request_mode(self, mode: str) -> None:
        """Ask MAVROS to switch flight mode (e.g. GUIDED, RTL)."""
        if not self._mode_client.service_is_ready():
            self.get_logger().warn(f"SetMode service not ready — cannot set {mode}")
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Mode '{mode}' request result: {f.result()}"
            )
        )

    def _send_servo_command(self, channel: int, pwm: int) -> None:
        """Send MAV_CMD_DO_SET_SERVO (cmd 183) via MAVROS CommandLong."""
        if not self._command_client.service_is_ready():
            self.get_logger().warn("Command service not ready — cannot set servo")
            return
        req = CommandLong.Request()
        req.command = 183  # MAV_CMD_DO_SET_SERVO
        req.param1 = float(channel)
        req.param2 = float(pwm)
        future = self._command_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"Servo cmd result: {f.result()}")
        )

    def _image_centre(self) -> tuple[float, float]:
        """Return (cx, cy) pixel coordinates of the image centre.

        Uses live dimensions received from target_cv via
        /drone_package_drop/image_size.  Logs a warning and returns a
        640x480 fallback if no message has arrived yet.
        """
        if self._image_dims is not None:
            w, h = self._image_dims
        else:
            self.get_logger().warn(
                "Image dims not yet received — falling back to 640x480",
                throttle_duration_sec=5.0,
            )
            w, h = 640, 480
        return w / 2.0, h / 2.0

    def _transition_to(self, new_state: DropState) -> None:
        if new_state != self._drop_state:
            self.get_logger().info(
                f"State: {self._drop_state.name} → {new_state.name}"
            )
            self._drop_state = new_state

    def _log_state_periodically(self, now) -> None:
        """Print current state + telemetry every 5 s."""
        if self._last_state_log_time is None:
            self._last_state_log_time = now
            return
        elapsed = (now - self._last_state_log_time).nanoseconds / 1e9
        if elapsed < 5.0:
            return

        gps = ""
        if self._drone_gps:
            gps = (
                f", GPS: ({self._drone_gps.latitude:.6f}, "
                f"{self._drone_gps.longitude:.6f})"
            )
        alt = ""
        if self._drone_local_pose:
            alt = f", alt={self._drone_local_pose.pose.position.z:.1f}m"
        self.get_logger().info(f"[{self._drop_state.name}]{gps}{alt}")
        self._last_state_log_time = now


# ======================================================================
#  Entry point
# ======================================================================


def main() -> None:
    rclpy.init()
    node = PayloadDrop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
