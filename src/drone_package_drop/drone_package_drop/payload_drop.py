"""
Payload Drop Mission — State Machine Node

Executes a precision payload drop using GPS navigation and visual
target tracking.  The drone always holds yaw = 0 (north) so the
downward-facing camera stays orientation-stable.

Assumes the drone is ALREADY AIRBORNE at ~20 m (e.g. via simple_takeoff).
Launch this node after takeoff is complete.

State flow:
  INIT ──► WAITING_FOR_CONNECTION ──► WAITING_FOR_GPS
    ──► TRANSIT_TO_TARGET ──► ACQUIRE_TARGET
        ├─► TARGET_NOT_FOUND  (stub — recovery TBD)
        └─► CENTER_ON_TARGET ──► DESCEND ──► DROP_PAYLOAD
              ──► RETURN_TO_LAUNCH ──► COMPLETE

Parameters:
  target_latitude          GPS latitude of drop zone
  target_longitude         GPS longitude of drop zone
  transit_altitude_m       Altitude for flying to GPS target  (default 20.0)
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
  /mavros/set_mode                      (SetMode)
  /mavros/cmd/command                   (CommandLong)  — MAV_CMD_DO_SET_SERVO
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
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import SetMode, CommandLong

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
        self.declare_parameter("drop_altitude_m", 5.0)
        self.declare_parameter("centering_tolerance_px", 30.0)
        self.declare_parameter("arrival_radius_m", 3.0)
        self.declare_parameter("arrival_alt_tolerance_m", 2.0)
        self.declare_parameter("servo_channel", 9)
        self.declare_parameter("servo_open_pwm", 1900)
        self.declare_parameter("servo_close_pwm", 1100)
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("image_width_px", 640)
        self.declare_parameter("image_height_px", 480)
        self.declare_parameter("guided_mode_name", "GUIDED")

        # ── Internal state ───────────────────────────────────────────
        self._drop_state = DropState.INIT
        self._mavros_state: Optional[State] = None
        self._drone_gps: Optional[NavSatFix] = None
        self._drone_local_pose: Optional[PoseStamped] = None
        self._target_pixel: Optional[PointStamped] = None   # latest detection
        self._current_setpoint: Optional[GlobalPositionTarget] = None
        self._last_state_log_time = None
        self._guided_requested = False
        self._drop_actuated = False

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
            "/payload_drop/target_detection",
            self._on_target_detection,
            10,
        )

        # ── Publisher ────────────────────────────────────────────────
        self._setpoint_pub = self.create_publisher(
            GlobalPositionTarget, "/mavros/setpoint_raw/global", 10
        )

        # ── Service clients ──────────────────────────────────────────
        self._mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._command_client = self.create_client(
            CommandLong, "/mavros/cmd/command"
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
        """Request GUIDED mode, then wait for MAVROS connection."""
        if not self._guided_requested and self._mode_client.service_is_ready():
            mode = (
                self.get_parameter("guided_mode_name")
                .get_parameter_value()
                .string_value
            )
            self._request_mode(mode)
            self._guided_requested = True
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
            f"GPS fix acquired  "
            f"({self._drone_gps.latitude:.7f}, {self._drone_gps.longitude:.7f})"
        )

        # Ensure we're in GUIDED before moving
        if not self._guided_requested:
            mode = (
                self.get_parameter("guided_mode_name")
                .get_parameter_value()
                .string_value
            )
            self._request_mode(mode)
            self._guided_requested = True

        self._transition_to(DropState.TRANSIT_TO_TARGET)

    # ── TRANSIT_TO_TARGET ─────────────────────────────────────────────

    def _handle_transit_to_target(self) -> None:
        """Fly to the target GPS coordinate at transit altitude."""

        # ── read params ──────────────────────────────────────────
        target_lat = self.get_parameter("target_latitude").get_parameter_value().double_value
        target_lon = self.get_parameter("target_longitude").get_parameter_value().double_value
        transit_alt = self.get_parameter("transit_altitude_m").get_parameter_value().double_value
        arrival_r = self.get_parameter("arrival_radius_m").get_parameter_value().double_value
        alt_tol = self.get_parameter("arrival_alt_tolerance_m").get_parameter_value().double_value

        # ── create / update setpoint ─────────────────────────────
        self._current_setpoint = self._create_gps_setpoint(
            target_lat, target_lon, transit_alt
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
        """Recovery when the target is not visible.

        TODO (you):
          Ideas: ascend to widen the field of view, do a small spiral
          search, go back to ACQUIRE_TARGET, etc.
        """
        self.get_logger().warn_throttle(
            self.get_clock(), 5.0,
            "Target not in frame — recovery not yet implemented",
        )

    # ── CENTER_ON_TARGET ──────────────────────────────────────────────

    def _handle_center_on_target(self) -> None:
        """Nudge the GPS setpoint so the target is centred in the image.

        Camera points straight down, yaw is locked to 0 (north), so:
          • +pixel X (target right of centre) → fly East  → increase longitude
          • +pixel Y (target below centre)    → fly South → decrease latitude
        """
        tol = self.get_parameter("centering_tolerance_px").get_parameter_value().double_value

        # Lost the target → go back to ACQUIRE
        if self._target_pixel is None:
            self.get_logger().warn("Lost target during centering")
            self._transition_to(DropState.ACQUIRE_TARGET)
            return

        cx, cy = self._image_centre()
        ex = self._target_pixel.point.x - cx   # positive = target is right
        ey = self._target_pixel.point.y - cy   # positive = target is below

        pixel_error = math.hypot(ex, ey)
        if pixel_error <= tol:
            self.get_logger().info("Target centred — beginning descent")
            self._transition_to(DropState.DESCEND)
            return

        # ── Proportional nudge ───────────────────────────────────
        # Convert pixel error to a small lat/lon offset.
        # Gain in metres-per-pixel — tune this for your camera/altitude.
        gain_m_per_px = 0.0003   # conservative; increase if centering is slow

        nudge_east_m  =  ex * gain_m_per_px   # +X pixel → east
        nudge_north_m = -ey * gain_m_per_px   # +Y pixel → south (negative north)

        # metres → degrees
        lat = self._current_setpoint.latitude
        d_lat = nudge_north_m / 111_320.0
        d_lon = nudge_east_m / (111_320.0 * math.cos(math.radians(lat)))

        self._current_setpoint.latitude  += d_lat
        self._current_setpoint.longitude += d_lon

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
        self, lat: float, lon: float, alt_rel: float, yaw: float = 0.0
    ) -> GlobalPositionTarget:
        """Build a GlobalPositionTarget (FRAME_GLOBAL_REL_ALT).

        yaw is in radians — 0.0 = North.  Heading is kept north by
        default so the camera never rotates.
        """
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt_rel
        msg.yaw = yaw  # 0 = north

        # Ignore velocity / acceleration / yaw-rate; control position + yaw
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        return msg

    def _publish_setpoint(self) -> None:
        """Re-stamp and publish the current setpoint to keep MAVROS alive."""
        if self._current_setpoint is None:
            return
        self._current_setpoint.header.stamp = self.get_clock().now().to_msg()
        self._setpoint_pub.publish(self._current_setpoint)

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
        """Return (cx, cy) pixel coordinates of the image centre."""
        w = self.get_parameter("image_width_px").get_parameter_value().integer_value
        h = self.get_parameter("image_height_px").get_parameter_value().integer_value
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
