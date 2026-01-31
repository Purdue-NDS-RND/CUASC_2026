from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, GimbalManagerPitchyaw   


class SimpleTakeoff(Node):
    def __init__(self) -> None:
        super().__init__("simple_takeoff")

        self.declare_parameter("takeoff_altitude_m", 20.0)
        self.declare_parameter("takeoff_min_pitch", 0.0)
        self.declare_parameter("takeoff_yaw", 0.0)
        self.declare_parameter("use_setpoint_stream", False)
        self.declare_parameter("setpoint_rate_hz", 10.0)
        self.declare_parameter("setpoint_warmup_s", 2.0)
        self.declare_parameter("arm_on_start", True)
        self.declare_parameter("set_guided_mode", True)
        self.declare_parameter("guided_mode_name", "GUIDED")
        self.declare_parameter("arm_retry_s", 5.0)
        self.declare_parameter("mode_retry_s", 2.0)
        self.declare_parameter("max_arm_attempts", 5)
        self.declare_parameter("auto_takeoff", True)
        self.declare_parameter("takeoff_retry_s", 5.0)
        self.declare_parameter("max_takeoff_attempts", 5)

        self._state: Optional[State] = None
        self._pose: Optional[PoseStamped] = None
        self._target_pose: Optional[PoseStamped] = None
        self._last_arm_request = None
        self._last_mode_request = None
        self._arm_attempts = 0
        self._warned_arm_stop = False
        self._takeoff_sent = False
        self._last_takeoff_request = None
        self._takeoff_attempts = 0
        self._warned_takeoff_stop = False
        self._takeoff_requested = False
        self._requested_altitude_m = None
        self._requested_min_pitch = 0.0
        self._requested_yaw = 0.0
        self._setpoint_sent = False
        self._gimbal_down_sent = False

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_pose,
            qos_profile_sensor_data,
        )

        self._setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )
        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        self._takeoff_service = self.create_service(
            CommandTOL, "/drone_control/takeoff", self._on_takeoff_service
        )

        rate = self.get_parameter("setpoint_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._on_timer)

    def _on_state(self, msg: State) -> None:
        self._state = msg

    def _on_pose(self, msg: PoseStamped) -> None:
        if self._takeoff_requested and self._target_pose is None and self._use_setpoint_stream():
            self._target_pose = PoseStamped()
            self._target_pose.header.frame_id = msg.header.frame_id
            self._target_pose.pose.position.x = msg.pose.position.x
            self._target_pose.pose.position.y = msg.pose.position.y
            self._target_pose.pose.position.z = msg.pose.position.z + float(
                self._requested_altitude_m or self.get_parameter("takeoff_altitude_m")
                .get_parameter_value()
                .double_value
            )
            self._target_pose.pose.orientation.w = 1.0
            self.get_logger().info(
                "Takeoff target set at x=%.2f, y=%.2f, z=%.2f"
                % (
                    self._target_pose.pose.position.x,
                    self._target_pose.pose.position.y,
                    self._target_pose.pose.position.z,
                )
            )
        self._pose = msg

    def _on_timer(self) -> None:
        self._publish_setpoint()
        self._handle_mode_and_arming()
        self._set_gimbal_down()
    def _set_gimbal_down(self) -> None:
        if self._gimbal_down_sent:
            return
        if self._state is None:
            return
        if not self._state.armed:
            return
        if self._pose is None:
            return
        
        gimbal_client = self.create_client(GimbalManagerPitchyaw, "/mavros/gimbal_control/manager/pitchyaw")
        if not gimbal_client.service_is_ready():
            self.get_logger().warn("Gimbal control service not available")
            return
        req = GimbalManagerPitchyaw.Request()
        req.pitch = -90.0
        req.yaw = 0.0
        req.pitch_rate = float('nan')
        req.yaw_rate = float('nan')
        req.flags = 0
        gimbal_client.call_async(req)
        self._gimbal_down_sent = True   




    def _publish_setpoint(self) -> None:
        if not self._use_setpoint_stream():
            return
        if self._target_pose is None:
            return
        if self._setpoint_sent:
            return
        self._target_pose.header.stamp = self.get_clock().now().to_msg()
        self._setpoint_pub.publish(self._target_pose)
        self._setpoint_sent = True

    def _handle_mode_and_arming(self) -> None:
        if not self._takeoff_requested:
            return
        if self._state is None:
            return

        now = self.get_clock().now()

        if self.get_parameter("set_guided_mode").get_parameter_value().bool_value:
            guided = self.get_parameter("guided_mode_name").get_parameter_value().string_value
            if self._state.mode != guided:
                if self._ready_for_mode_request(now):
                    self._request_mode(guided)
                return

        if self.get_parameter("arm_on_start").get_parameter_value().bool_value:
            if not self._state.armed:
                max_attempts = (
                    self.get_parameter("max_arm_attempts")
                    .get_parameter_value()
                    .integer_value
                )
                if self._arm_attempts >= max_attempts:
                    if not self._warned_arm_stop:
                        self.get_logger().warn(
                            "Max arm attempts reached; stopping auto-arm"
                        )
                        self._warned_arm_stop = True
                    return
                if self._ready_for_arm_request(now):
                    self._request_arm(True)
                    self._arm_attempts += 1
                return

        if self._state.armed and not self._takeoff_sent:
            max_attempts = (
                self.get_parameter("max_takeoff_attempts")
                .get_parameter_value()
                .integer_value
            )
            if self._takeoff_attempts >= max_attempts:
                if not self._warned_takeoff_stop:
                    self.get_logger().warn(
                        "Max takeoff attempts reached; stopping auto-takeoff"
                    )
                    self._warned_takeoff_stop = True
                return
            if self._ready_for_takeoff_request(now):
                self._request_takeoff(
                    altitude_m=float(
                        self._requested_altitude_m
                        or self.get_parameter("takeoff_altitude_m")
                        .get_parameter_value()
                        .double_value
                    ),
                    min_pitch=self._requested_min_pitch,
                    yaw=self._requested_yaw,
                )
                self._takeoff_attempts += 1
                self._takeoff_sent = True
                self._takeoff_requested = False

    def _ready_for_mode_request(self, now) -> bool:
        retry = self.get_parameter("mode_retry_s").get_parameter_value().double_value
        if self._last_mode_request is None:
            return True
        elapsed = (now - self._last_mode_request).nanoseconds / 1e9
        return elapsed >= retry

    def _ready_for_arm_request(self, now) -> bool:
        retry = self.get_parameter("arm_retry_s").get_parameter_value().double_value
        if self._last_arm_request is None:
            return True
        elapsed = (now - self._last_arm_request).nanoseconds / 1e9
        return elapsed >= retry

    def _ready_for_takeoff_request(self, now) -> bool:
        retry = self.get_parameter("takeoff_retry_s").get_parameter_value().double_value
        if self._last_takeoff_request is None:
            return True
        elapsed = (now - self._last_takeoff_request).nanoseconds / 1e9
        return elapsed >= retry

    def _request_mode(self, mode: str) -> None:
        if not self._mode_client.service_is_ready():
            self.get_logger().warn("Set mode service not available")
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        self._mode_client.call_async(req)
        self._last_mode_request = self.get_clock().now()

    def _request_arm(self, arm: bool) -> None:
        if not self._arm_client.service_is_ready():
            self.get_logger().warn("Arming service not available")
            return
        req = CommandBool.Request()
        req.value = arm
        self._arm_client.call_async(req)
        self._last_arm_request = self.get_clock().now()

    def _request_takeoff(self, altitude_m: float, min_pitch: float, yaw: float) -> None:
        if not self._takeoff_client.service_is_ready():
            self.get_logger().warn("Takeoff service not available")
            return
        req = CommandTOL.Request()
        req.min_pitch = float(min_pitch)
        req.yaw = float(yaw)
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = float(altitude_m)
        self._takeoff_client.call_async(req)
        self._last_takeoff_request = self.get_clock().now()
        self.get_logger().info("Takeoff request sent")

    def _on_takeoff_service(self, request: CommandTOL.Request, response: CommandTOL.Response) -> CommandTOL.Response:
        if self._state is None:
            response.success = False
            response.result = 0
            self.get_logger().warn("No MAVROS state yet; cannot takeoff")
            return response

        altitude = (
            request.altitude
            if request.altitude > 0.0
            else self.get_parameter("takeoff_altitude_m")
            .get_parameter_value()
            .double_value
        )

        self._requested_altitude_m = altitude
        self._requested_min_pitch = float(request.min_pitch)
        self._requested_yaw = float(request.yaw)
        self._takeoff_requested = True
        self._takeoff_sent = False
        self._takeoff_attempts = 0
        self._warned_takeoff_stop = False
        self._arm_attempts = 0
        self._warned_arm_stop = False
        self._setpoint_sent = False
        self._target_pose = None

        if self.get_parameter("set_guided_mode").get_parameter_value().bool_value:
            guided = self.get_parameter("guided_mode_name").get_parameter_value().string_value
            if self._state.mode != guided:
                self._request_mode(guided)

        if self.get_parameter("arm_on_start").get_parameter_value().bool_value:
            if not self._state.armed:
                self._request_arm(True)

        response.success = True
        response.result = 0
        self.get_logger().info(
            "Takeoff service accepted: altitude=%.2f, min_pitch=%.2f, yaw=%.2f"
            % (altitude, request.min_pitch, request.yaw)
        )
        return response

    def _use_setpoint_stream(self) -> bool:
        return self.get_parameter("use_setpoint_stream").get_parameter_value().bool_value


def main() -> None:
    rclpy.init()
    node = SimpleTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
