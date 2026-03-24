"""Waypoint Demo Mission Node

Timer-driven state machine that takes off, flies through a list of waypoints
in the local ENU frame, holds position at each one, then commands RTL.

Waypoints are ENU metre offsets from the home/arming position and are
specified as a list of [east_m, north_m] pairs in the mission YAML file
under the top-level ``waypoints`` key.  The file is read directly with
Python's ``yaml`` module so nested lists work naturally.

The local frame origin is the vehicle's home position set at arming, so
no GPS subscription or coordinate conversion is needed.

State Machine:
  INIT → WAITING_FOR_CONNECTION → WAITING_FOR_TAKEOFF_SERVICE
       → TAKING_OFF → WAITING_FOR_ALTITUDE
       → GO_TO_WAYPOINT → HOLD_AT_WAYPOINT → ADVANCE_WAYPOINT
         (repeat GO / HOLD / ADVANCE for each waypoint)
       → SET_RTL → DONE

Key behaviour – HOLD_AT_WAYPOINT:
  Once arrival criteria are met the node records a ROS-time stamp and keeps
  re-publishing the current local setpoint.  After hold_time_s seconds it
  transitions to ADVANCE_WAYPOINT.  No blocking sleep is used.

Publications:
  /mavros/setpoint_position/local  (geometry_msgs/PoseStamped)
      Continuous ENU position setpoint.  Published every control-loop tick
      to prevent MAVROS offboard timeout.

Subscriptions:
  /mavros/state               (mavros_msgs/State)         – connection & armed
  /mavros/local_position/pose (geometry_msgs/PoseStamped) – ENU position

Service Clients:
  drone_utils/takeoff   (mavros_msgs/CommandTOL) – trigger arm + takeoff
  /mavros/set_mode      (mavros_msgs/SetMode)    – switch to RTL at end

Parameters:
  config_file                   (string, "")
      Absolute path to the mission YAML file.  The node reads this file
      directly to extract the ``waypoints`` key.  Set automatically by
      the launch file.
  takeoff_altitude_m          (double, 20.0)
      Altitude passed to the takeoff service and used for the 90 % climb gate.
  waypoint_altitude_m         (double, 20.0)
      ENU Z value (metres above home) at which waypoints are flown.
  arrival_radius_m            (double, 3.0)
      Horizontal distance (m) to consider the drone "arrived" at a waypoint.
  arrival_height_tolerance_m  (double, 2.0)
      Vertical tolerance (m) for the arrival check.
  hold_time_s                 (double, 3.0)
      Seconds to hold position at each waypoint before advancing.
  setpoint_rate_hz            (double, 20.0)
      Rate of the main control-loop timer and setpoint publishing.
  desired_yaw_deg             (double, 90.0)
      Desired yaw angle in degrees (0° = East, 90° = North, 180° = West, 270° = South).
  rtl_mode                    (string, "RTL")
      MAVROS custom-mode string sent after the last waypoint.

Config file (waypoint_square.yaml):
  The file has two sections:
  1. ``waypoint_demo_mission.ros__parameters`` – loaded as ROS params by
     the launch file (all non-waypoint tuning knobs).
  2. ``waypoints`` – a top-level key with a list of [east_m, north_m] pairs,
     read directly by the node.

  Example::

    waypoints:
      - [20.0, 0.0]
      - [20.0, 20.0]
      - [0.0, 20.0]
      - [0.0, 0.0]

Usage:
  ros2 launch drone_demo waypoint_demo.launch.py
  ros2 launch drone_demo waypoint_demo.launch.py config:=/abs/path/custom.yaml
"""

import math
import yaml
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode


# ---------------------------------------------------------------------------
#  State enum
# ---------------------------------------------------------------------------


class MissionState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_TAKEOFF_SERVICE = auto()
    TAKING_OFF = auto()
    WAITING_FOR_ALTITUDE = auto()
    GO_TO_WAYPOINT = auto()
    HOLD_AT_WAYPOINT = auto()
    ADVANCE_WAYPOINT = auto()
    SET_RTL = auto()
    DONE = auto()


# ---------------------------------------------------------------------------
#  Node
# ---------------------------------------------------------------------------


class WaypointDemoMission(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_demo_mission")

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter("config_file", "")
        self.declare_parameter("takeoff_altitude_m", 20.0)
        self.declare_parameter("waypoint_altitude_m", 20.0)
        self.declare_parameter("arrival_radius_m", 3.0)
        self.declare_parameter("arrival_height_tolerance_m", 2.0)
        self.declare_parameter("hold_time_s", 3.0)
        self.declare_parameter("setpoint_rate_hz", 20.0)
        self.declare_parameter("desired_yaw_deg", 90.0)
        self.declare_parameter("rtl_mode", "RTL")

        # ── Internal state ───────────────────────────────────────
        self._state = MissionState.INIT
        self._mavros_state: Optional[State] = None
        self._drone_local_pose: Optional[PoseStamped] = None

        self._waypoints_enu: List[Tuple[float, float, float]] = []
        self._current_waypoint_index = 0
        self._current_setpoint: Optional[PoseStamped] = None

        self._takeoff_requested = False
        self._rtl_requested = False
        self._hold_start_time = None
        self._last_state_log_time = None

        # ── Subscribers ──────────────────────────────────────────
        self.create_subscription(State, "/mavros/state", self._on_mavros_state, 10)
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_drone_local_pose,
            qos_profile_sensor_data,
        )

        # ── Publisher ────────────────────────────────────────────
        self._setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )

        # ── Service clients ──────────────────────────────────────
        self._takeoff_client = self.create_client(CommandTOL, "drone_utils/takeoff")
        self._mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # ── Timer (main control loop) ────────────────────────────
        rate = self.get_parameter("setpoint_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._control_loop)

        # Build waypoints immediately from params (no GPS needed)
        self._build_waypoints()

        self.get_logger().info("Waypoint demo mission initialized (local ENU)")

    # ==================================================================
    #  Topic callbacks
    # ==================================================================

    def _on_mavros_state(self, msg: State) -> None:
        self._mavros_state = msg

    def _on_drone_local_pose(self, msg: PoseStamped) -> None:
        self._drone_local_pose = msg

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
            MissionState.INIT: self._handle_init,
            MissionState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            MissionState.WAITING_FOR_TAKEOFF_SERVICE: self._handle_waiting_for_takeoff_service,
            MissionState.TAKING_OFF: self._handle_taking_off,
            MissionState.WAITING_FOR_ALTITUDE: self._handle_waiting_for_altitude,
            MissionState.GO_TO_WAYPOINT: self._handle_go_to_waypoint,
            MissionState.HOLD_AT_WAYPOINT: self._handle_hold_at_waypoint,
            MissionState.ADVANCE_WAYPOINT: self._handle_advance_waypoint,
            MissionState.SET_RTL: self._handle_set_rtl,
            MissionState.DONE: lambda: None,
        }.get(self._state, lambda: None)
        handler()

    # ==================================================================
    #  State handlers
    # ==================================================================

    def _handle_init(self) -> None:
        self._transition_to(MissionState.WAITING_FOR_CONNECTION)

    def _handle_waiting_for_connection(self) -> None:
        if self._mavros_state is not None and self._mavros_state.connected:
            self._transition_to(MissionState.WAITING_FOR_TAKEOFF_SERVICE)

    def _handle_waiting_for_takeoff_service(self) -> None:
        if self._takeoff_client.service_is_ready():
            self._transition_to(MissionState.TAKING_OFF)

    def _handle_taking_off(self) -> None:
        if self._takeoff_requested:
            if self._mavros_state is not None and self._mavros_state.armed:
                self._transition_to(MissionState.WAITING_FOR_ALTITUDE)
            return

        req = CommandTOL.Request()
        req.altitude = (
            self.get_parameter("takeoff_altitude_m").get_parameter_value().double_value
        )
        future = self._takeoff_client.call_async(req)
        future.add_done_callback(self._on_takeoff_response)
        self._takeoff_requested = True
        self.get_logger().info(f"Requested takeoff to {req.altitude:.1f}m")

    def _on_takeoff_response(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.success:
                self.get_logger().warn("Takeoff service returned failure")
                self._takeoff_requested = False
            else:
                self.get_logger().info("Takeoff service accepted")
        except Exception as exc:
            self.get_logger().error(f"Takeoff service call failed: {exc}")
            self._takeoff_requested = False

    def _handle_waiting_for_altitude(self) -> None:
        if self._drone_local_pose is None:
            return

        target_alt = (
            self.get_parameter("takeoff_altitude_m").get_parameter_value().double_value
        )
        current_alt = self._drone_local_pose.pose.position.z

        if current_alt >= target_alt * 0.9:
            self.get_logger().info(f"Reached altitude {current_alt:.1f}m")
            self._set_current_waypoint_setpoint()
            self._transition_to(MissionState.GO_TO_WAYPOINT)

    def _handle_go_to_waypoint(self) -> None:
        if self._drone_local_pose is None:
            return
        if self._current_waypoint_index >= len(self._waypoints_enu):
            self._transition_to(MissionState.SET_RTL)
            return

        if self._check_arrival():
            self._hold_start_time = self.get_clock().now()
            self.get_logger().info(f"Reached waypoint #{self._current_waypoint_index + 1}")
            self._transition_to(MissionState.HOLD_AT_WAYPOINT)

    def _handle_hold_at_waypoint(self) -> None:
        if self._hold_start_time is None:
            self._hold_start_time = self.get_clock().now()
            return

        hold_time_s = self.get_parameter("hold_time_s").get_parameter_value().double_value
        elapsed = (self.get_clock().now() - self._hold_start_time).nanoseconds / 1e9
        if elapsed >= hold_time_s:
            self._transition_to(MissionState.ADVANCE_WAYPOINT)

    def _handle_advance_waypoint(self) -> None:
        self._current_waypoint_index += 1
        if self._current_waypoint_index >= len(self._waypoints_enu):
            self._transition_to(MissionState.SET_RTL)
            return

        self._set_current_waypoint_setpoint()
        self._transition_to(MissionState.GO_TO_WAYPOINT)

    def _handle_set_rtl(self) -> None:
        if self._rtl_requested:
            self._transition_to(MissionState.DONE)
            return
        if not self._mode_client.service_is_ready():
            return

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = (
            self.get_parameter("rtl_mode").get_parameter_value().string_value
        )
        future = self._mode_client.call_async(req)
        future.add_done_callback(self._on_rtl_response)
        self._rtl_requested = True
        self.get_logger().info(f"Requested RTL mode: {req.custom_mode}")

    def _on_rtl_response(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.mode_sent:
                self.get_logger().warn("RTL mode request was not accepted")
            else:
                self.get_logger().info("RTL mode command sent")
        except Exception as exc:
            self.get_logger().error(f"RTL mode service call failed: {exc}")

    # ==================================================================
    #  Helper utilities
    # ==================================================================

    def _check_arrival(self) -> bool:
        if self._drone_local_pose is None:
            return False
        if self._current_waypoint_index >= len(self._waypoints_enu):
            return False

        tx, ty, tz = self._waypoints_enu[self._current_waypoint_index]
        pos = self._drone_local_pose.pose.position
        horizontal_dist = math.sqrt((pos.x - tx) ** 2 + (pos.y - ty) ** 2)
        dz = abs(pos.z - tz)

        arrival_radius = (
            self.get_parameter("arrival_radius_m").get_parameter_value().double_value
        )
        arrival_height_tol = (
            self.get_parameter("arrival_height_tolerance_m")
            .get_parameter_value()
            .double_value
        )

        return horizontal_dist <= arrival_radius and dz <= arrival_height_tol

    def _build_waypoints(self) -> None:
        config_path = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )
        if not config_path:
            self.get_logger().warn(
                "config_file param is empty — no waypoints loaded.  "
                "Use the launch file or pass  -p config_file:=/path/to/yaml"
            )
            return

        try:
            with open(config_path, "r") as fh:
                raw = yaml.safe_load(fh)
        except Exception as exc:
            self.get_logger().error(f"Failed to read config file: {exc}")
            return

        wp_list = raw.get("waypoints")
        if not isinstance(wp_list, list) or len(wp_list) == 0:
            self.get_logger().warn(
                "No 'waypoints' key found (or list is empty) in config file"
            )
            return

        alt = self.get_parameter("waypoint_altitude_m").get_parameter_value().double_value
        for i, entry in enumerate(wp_list):
            if not isinstance(entry, (list, tuple)) or len(entry) < 2:
                self.get_logger().warn(f"Skipping malformed waypoint #{i}: {entry}")
                continue
            self._waypoints_enu.append((float(entry[0]), float(entry[1]), alt))

        self.get_logger().info(f"Loaded {len(self._waypoints_enu)} local ENU waypoints")

    def _set_current_waypoint_setpoint(self) -> None:
        if self._current_waypoint_index >= len(self._waypoints_enu):
            return
        x, y, z = self._waypoints_enu[self._current_waypoint_index]
        self._current_setpoint = self._create_setpoint(x, y, z)
        self.get_logger().info(
            f"Heading to waypoint #{self._current_waypoint_index + 1}: "
            f"east={x:.1f}, north={y:.1f}, up={z:.1f}"
        )

    def _create_setpoint(self, x: float, y: float, z: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Convert desired yaw (in degrees) to quaternion
        yaw_deg = self.get_parameter("desired_yaw_deg").get_parameter_value().double_value
        yaw_rad = math.radians(yaw_deg)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        
        return msg

    def _publish_setpoint(self) -> None:
        if self._current_setpoint is None:
            return
        self._current_setpoint.header.stamp = self.get_clock().now().to_msg()
        self._setpoint_pub.publish(self._current_setpoint)

    def _log_state_periodically(self, now) -> None:
        if self._last_state_log_time is None:
            self._last_state_log_time = now
            return

        elapsed = (now - self._last_state_log_time).nanoseconds / 1e9
        if elapsed >= 5.0:
            self.get_logger().info(
                f"State={self._state.name}, waypoint_index={self._current_waypoint_index}"
            )
            self._last_state_log_time = now

    def _transition_to(self, new_state: MissionState) -> None:
        if new_state != self._state:
            self.get_logger().info(f"State: {self._state.name} -> {new_state.name}")
            self._state = new_state


def main() -> None:
    rclpy.init()
    node = WaypointDemoMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()