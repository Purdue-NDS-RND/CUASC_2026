"""Simple Takeoff Service Node

Provides a lightweight drone_utils/takeoff service that sequentially arms
the vehicle and sends a takeoff command via MAVROS.  Intentionally minimal
— no mode switching, no retries, no altitude monitoring.  Designed as a
reusable building block for demo launch files.

Flow:
  1. Idle until a service call arrives on drone_utils/takeoff
  2. Send arm command  (/mavros/cmd/arming)
  3. Wait arm_check_delay_s, then verify armed state
  4. Send takeoff command (/mavros/cmd/takeoff) at requested altitude
  5. Return to idle (ready for another call)

Service Provided:
  drone_utils/takeoff (mavros_msgs/CommandTOL)
      request.altitude  – target altitude in metres (0 → uses default param)
      response.success  – True if sequence was accepted

Subscriptions:
  /mavros/state (mavros_msgs/State)
      Used only for the single post-arm check

Service Clients:
  /mavros/cmd/arming   (mavros_msgs/CommandBool)  – arm / disarm
  /mavros/cmd/takeoff   (mavros_msgs/CommandTOL)   – MAVROS takeoff

Parameters:
  default_takeoff_altitude_m  (double, 20.0)
      Altitude used when the caller passes altitude <= 0.
  arm_check_delay_s           (double, 0.5)
      Seconds to wait after the arm command before checking armed state.

Usage:
  ros2 run drone_utils simple_takeoff_service
  ros2 run drone_utils simple_takeoff_service --ros-args \
      -p default_takeoff_altitude_m:=30.0

  # Trigger from another terminal:
  ros2 service call drone_utils/takeoff mavros_msgs/srv/CommandTOL \
      "{altitude: 25.0}"
"""

from typing import Optional

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class SimpleTakeoffService(Node):
    def __init__(self) -> None:
        super().__init__("simple_takeoff_service")

        self.declare_parameter("default_takeoff_altitude_m", 20.0)
        self.declare_parameter("guided_mode_name", "GUIDED")
        self.declare_parameter("mode_retry_s", 2.0)
        self.declare_parameter("arm_retry_s", 5.0)
        self.declare_parameter("max_arm_attempts", 5)
        self.declare_parameter("takeoff_retry_s", 5.0)
        self.declare_parameter("loop_rate_hz", 2.0)

        self._state: Optional[State] = None
        self._pending_altitude_m: Optional[float] = None
        self._takeoff_sent = False

        self._last_mode_request = None
        self._last_arm_request = None
        self._last_takeoff_request = None
        self._arm_attempts = 0
        self._warned_arm_stop = False

        self.create_subscription(State, "/mavros/state", self._on_state, 10)

        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self._mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self._takeoff_service = self.create_service(
            CommandTOL,
            "drone_utils/takeoff",
            self._on_takeoff_service,
        )

        rate = self.get_parameter("loop_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

        self.get_logger().info("Simple takeoff service ready on drone_utils/takeoff")

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_takeoff_service(
        self,
        request: CommandTOL.Request,
        response: CommandTOL.Response,
    ) -> CommandTOL.Response:
        if self._pending_altitude_m is not None and not self._takeoff_sent:
            response.success = False
            response.result = 0
            self.get_logger().warn("Takeoff already in progress")
            return response

        altitude = request.altitude
        if altitude <= 0.0:
            altitude = (
                self.get_parameter("default_takeoff_altitude_m")
                .get_parameter_value()
                .double_value
            )

        self._pending_altitude_m = float(altitude)
        self._takeoff_sent = False
        self._arm_attempts = 0
        self._warned_arm_stop = False
        self._last_mode_request = None
        self._last_arm_request = None
        self._last_takeoff_request = None

        response.success = True
        response.result = 0
        self.get_logger().info(f"Takeoff sequence accepted (altitude={altitude:.1f}m)")
        return response

    # ------------------------------------------------------------------
    #  Timer-driven state machine (mirrors simple_takeoff.py approach)
    # ------------------------------------------------------------------

    def _on_timer(self) -> None:
        if self._pending_altitude_m is None or self._takeoff_sent:
            return
        if self._state is None:
            return

        now = self.get_clock().now()
        guided = self.get_parameter("guided_mode_name").get_parameter_value().string_value

        # Step 1: wait for GUIDED mode to be confirmed by FCU
        if self._state.mode != guided:
            if self._ready_for_request(now, self._last_mode_request, "mode_retry_s"):
                self._request_mode(guided)
            return

        # Step 2: wait for armed state to be confirmed by FCU
        if not self._state.armed:
            max_attempts = self.get_parameter("max_arm_attempts").get_parameter_value().integer_value
            if self._arm_attempts >= max_attempts:
                if not self._warned_arm_stop:
                    self.get_logger().warn("Max arm attempts reached; aborting takeoff sequence")
                    self._warned_arm_stop = True
                    self._pending_altitude_m = None
                return
            if self._ready_for_request(now, self._last_arm_request, "arm_retry_s"):
                self._request_arm(True)
                self._arm_attempts += 1
            return

        # Step 3: send takeoff command once armed
        if self._ready_for_request(now, self._last_takeoff_request, "takeoff_retry_s"):
            self._request_takeoff(self._pending_altitude_m)
            self._takeoff_sent = True

    def _ready_for_request(self, now, last_time, param_name: str) -> bool:
        if last_time is None:
            return True
        retry = self.get_parameter(param_name).get_parameter_value().double_value
        return (now - last_time).nanoseconds / 1e9 >= retry

    def _request_mode(self, mode: str) -> None:
        if not self._mode_client.service_is_ready():
            self.get_logger().warn("Set mode service not available")
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        self._mode_client.call_async(req)
        self._last_mode_request = self.get_clock().now()
        self.get_logger().info(f"Requesting mode: {mode}")

    def _request_arm(self, arm: bool) -> None:
        if not self._arm_client.service_is_ready():
            self.get_logger().warn("Arming service not available")
            return
        req = CommandBool.Request()
        req.value = arm
        self._arm_client.call_async(req)
        self._last_arm_request = self.get_clock().now()
        self.get_logger().info("Arming requested")

    def _request_takeoff(self, altitude_m: float) -> None:
        if not self._takeoff_client.service_is_ready():
            self.get_logger().warn("Takeoff service not available")
            return
        req = CommandTOL.Request()
        req.altitude = float(altitude_m)
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        self._takeoff_client.call_async(req)
        self._last_takeoff_request = self.get_clock().now()
        self.get_logger().info(f"Takeoff command sent (altitude={altitude_m:.1f}m)")


def main() -> None:
    rclpy.init()
    node = SimpleTakeoffService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()