"""Timer-driven mission executor node."""

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from .mission_api import MissionFailurePolicy, MissionStatus
from .mission_context import MissionContext
from .registry import create_mission, import_mission_modules, load_sequence_file


class MissionExecutorNode(Node):
    """Executes a mission sequence one mission at a time."""

    def __init__(self) -> None:
        super().__init__("mission_executor")

        self.declare_parameter("sequence_file", "")
        self.declare_parameter("mission_modules", [])
        self.declare_parameter("loop_rate_hz", 20.0)
        self.declare_parameter("abort_rtl_mode", "RTL")

        self._mavros_state: Optional[State] = None
        self._local_pose: Optional[PoseStamped] = None
        self._global_gps: Optional[NavSatFix] = None

        self._local_setpoint_pub = self.create_publisher(
            PoseStamped,
            "/mavros/setpoint_position/local",
            10,
        )

        self.create_subscription(State, "/mavros/state", self._on_mavros_state, 10)
        self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._on_local_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._on_global_gps,
            qos_profile_sensor_data,
        )

        self._takeoff_client = self.create_client(CommandTOL, "drone_utils/takeoff")
        self._mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self._context = MissionContext(self)
        self._sequence = self._load_sequence()
        self._active_index = 0
        self._active_mission = None
        self._abort_requested = False
        self._abort_mode_future = None
        self._abort_mode_requested = False
        self._done_logged = False

        rate = self.get_parameter("loop_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._control_loop)

        self.get_logger().info(
            f"Mission executor initialized with {len(self._sequence)} mission(s)"
        )

    def _load_sequence(self):
        module_names = (
            self.get_parameter("mission_modules")
            .get_parameter_value()
            .string_array_value
        )
        import_mission_modules(module_names)

        sequence_file = (
            self.get_parameter("sequence_file")
            .get_parameter_value()
            .string_value
        )
        if not sequence_file:
            raise ValueError("mission_executor.sequence_file parameter is required")

        specs = load_sequence_file(sequence_file)
        missions = [create_mission(spec) for spec in specs]
        for mission in missions:
            self.get_logger().info(
                f"Loaded mission '{mission.name}' ({mission.spec.type_name})"
            )
        return missions

    def _on_mavros_state(self, msg: State) -> None:
        self._mavros_state = msg

    def _on_local_pose(self, msg: PoseStamped) -> None:
        self._local_pose = msg

    def _on_global_gps(self, msg: NavSatFix) -> None:
        self._global_gps = msg

    def _control_loop(self) -> None:
        self._context.publish_managed_setpoints()

        if self._abort_requested:
            self._handle_abort_rtl()
            return

        if self._active_mission is None:
            self._start_next_mission()
            return

        try:
            status = self._active_mission.update(self._context)
        except Exception as exc:
            self.get_logger().error(
                f"Mission '{self._active_mission.name}' raised during update: {exc}"
            )
            status = MissionStatus.FAILURE
        if status in (MissionStatus.RUNNING, MissionStatus.WAITING):
            return

        if status == MissionStatus.SUCCESS:
            self.get_logger().info(f"Mission '{self._active_mission.name}' completed")
            self._safe_on_exit(self._active_mission)
            self._active_mission = None
            self._active_index += 1
            return

        if status in (MissionStatus.FAILURE, MissionStatus.CANCELLED):
            self.get_logger().error(
                f"Mission '{self._active_mission.name}' ended with status {status.name}"
            )
            self._safe_on_exit(self._active_mission)
            policy = self._active_mission.spec.failure_policy
            self._active_mission = None
            if policy == MissionFailurePolicy.ABORT_AND_RTL:
                self._context.clear_local_position_setpoint()
                self._abort_requested = True
            return

    def _start_next_mission(self) -> None:
        if self._active_index >= len(self._sequence):
            if not self._done_logged:
                self.get_logger().info("Mission sequence complete")
                self._done_logged = True
            return

        self._active_mission = self._sequence[self._active_index]
        self.get_logger().info(
            f"Starting mission '{self._active_mission.name}' "
            f"({self._active_mission.spec.type_name})"
        )
        try:
            self._active_mission.on_enter(self._context)
        except Exception as exc:
            self.get_logger().error(
                f"Mission '{self._active_mission.name}' failed during on_enter: {exc}"
            )
            self._context.clear_local_position_setpoint()
            self._active_mission = None
            self._abort_requested = True

    def _handle_abort_rtl(self) -> None:
        rtl_mode = (
            self.get_parameter("abort_rtl_mode")
            .get_parameter_value()
            .string_value
        )

        if self._mavros_state is not None and self._mavros_state.mode == rtl_mode:
            if not self._done_logged:
                self.get_logger().warn("Abort RTL confirmed by FCU mode")
                self._done_logged = True
            return

        if self._abort_mode_requested:
            return

        if self._abort_mode_future is not None and not self._abort_mode_future.done():
            return

        if not self._mode_client.service_is_ready():
            self.get_logger().warn("Waiting for set_mode service to send abort RTL")
            return

        self.get_logger().warn(f"Mission failure policy triggered abort via {rtl_mode}")
        self._abort_mode_requested = True
        self._abort_mode_future = self._context.request_mode_change(
            rtl_mode,
            self._on_abort_mode_response,
        )

    def _on_abort_mode_response(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.mode_sent:
                self.get_logger().error("Abort RTL mode request was not accepted")
                self._abort_mode_requested = False
            else:
                self.get_logger().warn("Abort RTL mode request sent")
        except Exception as exc:
            self.get_logger().error(f"Abort RTL request failed: {exc}")
            self._abort_mode_requested = False

    def _safe_on_exit(self, mission) -> None:
        try:
            mission.on_exit(self._context)
        except Exception as exc:
            self.get_logger().error(
                f"Mission '{mission.name}' raised during on_exit: {exc}"
            )


def main() -> None:
    rclpy.init()
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
