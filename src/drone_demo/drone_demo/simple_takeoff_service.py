"""Simple Takeoff Service Node

Provides a lightweight /drone_demo/takeoff service that sequentially arms
the vehicle and sends a takeoff command via MAVROS.  Intentionally minimal
— no mode switching, no retries, no altitude monitoring.  Designed as a
reusable building block for demo launch files.

Flow:
  1. Idle until a service call arrives on /drone_demo/takeoff
  2. Send arm command  (/mavros/cmd/arming)
  3. Wait arm_check_delay_s, then verify armed state
  4. Send takeoff command (/mavros/cmd/takeoff) at requested altitude
  5. Return to idle (ready for another call)

Service Provided:
  /drone_demo/takeoff (mavros_msgs/CommandTOL)
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
  ros2 run drone_demo simple_takeoff_service
  ros2 run drone_demo simple_takeoff_service --ros-args \
      -p default_takeoff_altitude_m:=30.0

  # Trigger from another terminal:
  ros2 service call /drone_demo/takeoff mavros_msgs/srv/CommandTOL \
      "{altitude: 25.0}"
"""

from typing import Optional

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL


class SimpleTakeoffService(Node):
    def __init__(self) -> None:
        super().__init__("simple_takeoff_service")

        self.declare_parameter("default_takeoff_altitude_m", 20.0)
        self.declare_parameter("arm_check_delay_s", 0.5)

        self._state: Optional[State] = None
        self._pending_altitude_m: Optional[float] = None
        self._arm_check_timer = None

        self.create_subscription(State, "/mavros/state", self._on_state, 10)

        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        self._takeoff_service = self.create_service(
            CommandTOL,
            "/drone_demo/takeoff",
            self._on_takeoff_service,
        )

        self.get_logger().info("Simple takeoff service ready on /drone_demo/takeoff")

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_takeoff_service(
        self,
        request: CommandTOL.Request,
        response: CommandTOL.Response,
    ) -> CommandTOL.Response:
        if self._pending_altitude_m is not None:
            response.success = False
            response.result = 0
            self.get_logger().warn("Takeoff already in progress")
            return response

        if not self._arm_client.service_is_ready() or not self._takeoff_client.service_is_ready():
            response.success = False
            response.result = 0
            self.get_logger().warn("Arming/takeoff service not ready")
            return response

        altitude = request.altitude
        if altitude <= 0.0:
            altitude = (
                self.get_parameter("default_takeoff_altitude_m")
                .get_parameter_value()
                .double_value
            )

        self._pending_altitude_m = float(altitude)

        arm_req = CommandBool.Request()
        arm_req.value = True
        arm_future = self._arm_client.call_async(arm_req)
        arm_future.add_done_callback(self._on_arm_done)

        response.success = True
        response.result = 0
        self.get_logger().info(f"Takeoff sequence accepted (altitude={altitude:.1f}m)")
        return response

    def _on_arm_done(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.success:
                self.get_logger().warn("Arm command failed")
                self._pending_altitude_m = None
                return
        except Exception as exc:
            self.get_logger().error(f"Arm service call failed: {exc}")
            self._pending_altitude_m = None
            return

        delay = (
            self.get_parameter("arm_check_delay_s")
            .get_parameter_value()
            .double_value
        )
        if self._arm_check_timer is not None:
            self._arm_check_timer.cancel()
        self._arm_check_timer = self.create_timer(max(delay, 0.0), self._check_armed_and_takeoff)

    def _check_armed_and_takeoff(self) -> None:
        if self._arm_check_timer is not None:
            self._arm_check_timer.cancel()
            self._arm_check_timer = None

        if self._pending_altitude_m is None:
            return

        if self._state is None or not self._state.armed:
            self.get_logger().warn("Vehicle did not report armed state after arm command")
            self._pending_altitude_m = None
            return

        req = CommandTOL.Request()
        req.altitude = float(self._pending_altitude_m)
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        takeoff_future = self._takeoff_client.call_async(req)
        takeoff_future.add_done_callback(self._on_takeoff_done)

    def _on_takeoff_done(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.success:
                self.get_logger().warn("Takeoff command failed")
            else:
                self.get_logger().info("Takeoff command sent")
        except Exception as exc:
            self.get_logger().error(f"Takeoff service call failed: {exc}")
        finally:
            self._pending_altitude_m = None


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