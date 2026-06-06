import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [
    str(ROOT / "src" / "drone_mission_demo"),
    str(ROOT / "src" / "drone_mission_core"),
]
for module_name in list(sys.modules):
    if module_name == "drone_mission_core" or module_name.startswith(
        "drone_mission_core."
    ):
        del sys.modules[module_name]


class FakeDuration:
    def __init__(self, nanoseconds: int) -> None:
        self.nanoseconds = nanoseconds


class FakeTime:
    def __init__(self, nanoseconds: int = 0) -> None:
        self.nanoseconds = nanoseconds

    def __sub__(self, other: "FakeTime") -> FakeDuration:
        return FakeDuration(self.nanoseconds - other.nanoseconds)

    @classmethod
    def from_msg(cls, msg) -> "FakeTime":
        return msg

    def to_msg(self) -> "FakeTime":
        return self


class _Msg:
    pass


class _PositionTarget:
    FRAME_LOCAL_NED = 1
    IGNORE_PX = 1
    IGNORE_PY = 2
    IGNORE_PZ = 4
    IGNORE_AFX = 8
    IGNORE_AFY = 16
    IGNORE_AFZ = 32
    IGNORE_YAW_RATE = 64


class _GlobalPositionTarget:
    FRAME_GLOBAL_REL_ALT = 6
    IGNORE_VX = 1
    IGNORE_VY = 2
    IGNORE_VZ = 4
    IGNORE_AFX = 8
    IGNORE_AFY = 16
    IGNORE_AFZ = 32
    IGNORE_YAW_RATE = 64
    IGNORE_YAW = 128


class _AttitudeTarget:
    IGNORE_ROLL_RATE = 1
    IGNORE_PITCH_RATE = 2
    IGNORE_YAW_RATE = 4


class _ExtendedState:
    LANDED_STATE_UNDEFINED = 0
    LANDED_STATE_ON_GROUND = 1


def _install_ros_stubs() -> None:
    sys.modules.setdefault("yaml", types.ModuleType("yaml"))

    rclpy_mod = types.ModuleType("rclpy")
    rclpy_time_mod = types.ModuleType("rclpy.time")
    rclpy_time_mod.Time = FakeTime
    rclpy_node_mod = types.ModuleType("rclpy.node")
    rclpy_node_mod.Node = object
    rclpy_task_mod = types.ModuleType("rclpy.task")
    rclpy_task_mod.Future = object
    rclpy_impl_mod = types.ModuleType("rclpy.impl")
    rclpy_logger_mod = types.ModuleType("rclpy.impl.rcutils_logger")
    rclpy_logger_mod.RcutilsLogger = object
    sys.modules.setdefault("rclpy", rclpy_mod)
    sys.modules.setdefault("rclpy.time", rclpy_time_mod)
    sys.modules.setdefault("rclpy.node", rclpy_node_mod)
    sys.modules.setdefault("rclpy.task", rclpy_task_mod)
    sys.modules.setdefault("rclpy.impl", rclpy_impl_mod)
    sys.modules.setdefault("rclpy.impl.rcutils_logger", rclpy_logger_mod)

    geometry_mod = types.ModuleType("geometry_msgs")
    geometry_msg_mod = types.ModuleType("geometry_msgs.msg")
    geometry_msg_mod.PointStamped = _Msg
    geometry_msg_mod.PoseStamped = _Msg
    sys.modules.setdefault("geometry_msgs", geometry_mod)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msg_mod)

    mavros_mod = types.ModuleType("mavros_msgs")
    mavros_msg_mod = types.ModuleType("mavros_msgs.msg")
    mavros_msg_mod.AttitudeTarget = _AttitudeTarget
    mavros_msg_mod.ExtendedState = _ExtendedState
    mavros_msg_mod.GlobalPositionTarget = _GlobalPositionTarget
    mavros_msg_mod.PositionTarget = _PositionTarget
    mavros_msg_mod.State = _Msg
    mavros_srv_mod = types.ModuleType("mavros_msgs.srv")
    for name in ("CommandLong", "CommandTOL", "SetMode"):
        setattr(mavros_srv_mod, name, _Msg)
    sys.modules.setdefault("mavros_msgs", mavros_mod)
    sys.modules.setdefault("mavros_msgs.msg", mavros_msg_mod)
    sys.modules.setdefault("mavros_msgs.srv", mavros_srv_mod)

    sensor_mod = types.ModuleType("sensor_msgs")
    sensor_msg_mod = types.ModuleType("sensor_msgs.msg")
    sensor_msg_mod.NavSatFix = _Msg
    sys.modules.setdefault("sensor_msgs", sensor_mod)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msg_mod)

    std_mod = types.ModuleType("std_srvs")
    std_srv_mod = types.ModuleType("std_srvs.srv")
    std_srv_mod.SetBool = _Msg
    sys.modules.setdefault("std_srvs", std_mod)
    sys.modules.setdefault("std_srvs.srv", std_srv_mod)


_install_ros_stubs()

from drone_mission_core.mission_api import MissionSpec, MissionStatus
from drone_mission_demo.missions.package_drop_mission import (
    PackageDropMission,
    PackageDropState,
)


class FakeLogger:
    def info(self, _msg: str) -> None:
        pass

    def warn(self, _msg: str) -> None:
        pass

    def error(self, _msg: str) -> None:
        pass


class FakeFuture:
    def __init__(self, success: bool = True) -> None:
        self._success = success

    def result(self):
        return SimpleNamespace(success=self._success)


class FakeContext:
    def __init__(self) -> None:
        self._now_ns = 0
        self.logger = FakeLogger()
        self.global_gps = SimpleNamespace(
            latitude=47.397742,
            longitude=8.545594,
            status=SimpleNamespace(status=0),
        )
        self.local_pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(z=10.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        self.mavros_state = SimpleNamespace(connected=True, armed=True)
        self.target_detection = None
        self.image_size = None
        self.local_velocity_commands = []
        self.global_position_commands = []
        self.sprayer_commands = []

    def now(self) -> FakeTime:
        return FakeTime(self._now_ns)

    def advance(self, seconds: float) -> None:
        self._now_ns += int(seconds * 1e9)

    def seconds_since(self, start_time: FakeTime) -> float:
        return (self.now() - start_time).nanoseconds / 1e9

    def clear_all_setpoints(self) -> None:
        pass

    def clear_target_tracking_state(self) -> None:
        self.target_detection = None
        self.image_size = None

    def target_cv_control_ready(self) -> bool:
        return False

    def set_local_velocity_setpoint(self, east, north, up=0.0, yaw_deg=90.0) -> None:
        self.local_velocity_commands.append((east, north, up, yaw_deg))

    def set_global_position_setpoint(
        self,
        latitude,
        longitude,
        altitude_m,
        yaw_deg=90.0,
        lock_yaw=True,
    ) -> None:
        self.global_position_commands.append(
            (latitude, longitude, altitude_m, yaw_deg, lock_yaw)
        )

    def command_service_ready(self) -> bool:
        return True

    def command_sprayer(self, *, enable: bool, done_callback=None) -> FakeFuture:
        self.sprayer_commands.append(enable)
        future = FakeFuture(success=True)
        if done_callback is not None:
            done_callback(future)
        return future


def make_mission(**config_overrides) -> PackageDropMission:
    config = {
        "target_latitude": 47.397742,
        "target_longitude": 8.545594,
        "target_loss_grace_s": 4.0,
        "recovery_alt_tolerance_m": 0.5,
        "arrival_alt_tolerance_m": 2.0,
        "not_found_ascent_m": 1.0,
        "max_recovery_attempts": 4,
        "drop_altitude_m": 4.0,
        "drop_altitude_tolerance_m": 0.5,
        "drop_column_handoff_altitude_m": 8.0,
        "drop_hover_dwell_s": 0.0,
    }
    config.update(config_overrides)
    spec = MissionSpec(
        type_name="package_drop",
        name="package_drop",
        config=config,
        base_dir=Path("."),
    )
    mission = PackageDropMission(spec)
    context = FakeContext()
    mission.on_enter(context)
    return mission


class PackageDropMissionTests(unittest.TestCase):
    def test_tracking_loss_holds_during_grace_before_recovery(self) -> None:
        mission = make_mission()
        context = FakeContext()
        mission._state = PackageDropState.TRACK_AND_DESCEND

        status = mission.update(context)
        self.assertEqual(status, MissionStatus.RUNNING)
        self.assertEqual(mission._state, PackageDropState.TRACK_AND_DESCEND)
        self.assertEqual(mission._recovery_attempts, 0)
        self.assertEqual(context.local_velocity_commands[-1][:3], (0.0, 0.0, 0.0))

        context.advance(3.9)
        mission.update(context)
        self.assertEqual(mission._state, PackageDropState.TRACK_AND_DESCEND)
        self.assertEqual(mission._recovery_attempts, 0)

        context.advance(0.2)
        mission.update(context)
        self.assertEqual(mission._state, PackageDropState.TARGET_NOT_FOUND)

    def test_recovery_uses_dedicated_altitude_tolerance(self) -> None:
        mission = make_mission()
        context = FakeContext()
        mission._state = PackageDropState.TARGET_NOT_FOUND

        mission.update(context)
        self.assertEqual(mission._state, PackageDropState.TARGET_NOT_FOUND)
        self.assertEqual(mission._recovery_attempts, 1)

        context.local_pose.pose.position.z = 10.4
        mission.update(context)
        self.assertEqual(mission._state, PackageDropState.TARGET_NOT_FOUND)

        context.local_pose.pose.position.z = 10.6
        mission.update(context)
        self.assertEqual(mission._state, PackageDropState.ACQUIRE_TARGET)

    def test_centered_drop_commits_to_fixed_column_below_handoff_altitude(self) -> None:
        mission = make_mission()
        context = FakeContext()
        context.local_pose.pose.position.z = 7.5
        mission._state = PackageDropState.TRACK_AND_DESCEND
        mission._get_centering_descent_command = (
            lambda *_args, **_kwargs: SimpleNamespace(
                tracking_error_m=0.1,
                velocity_east_mps=0.0,
                velocity_north_mps=0.0,
                vertical_velocity_mps=-0.35,
                reached_target_altitude=False,
            )
        )

        mission.update(context)
        self.assertEqual(mission._state, PackageDropState.FINAL_FIXED_DROP_COLUMN)

        context.advance(1.0)
        mission.update(context)
        first_command = context.global_position_commands[-1]
        self.assertEqual(first_command[0], context.global_gps.latitude)
        self.assertEqual(first_command[1], context.global_gps.longitude)

        context.local_pose.pose.position.z = 4.4
        context.advance(1.0)
        mission.update(context)
        self.assertEqual(mission._state, PackageDropState.DROP_PAYLOAD)

    def test_real_drop_commands_sprayer_close_when_fake_drop_is_false(self) -> None:
        mission = make_mission(fake_drop=False)
        context = FakeContext()
        context.local_pose.pose.position.z = 4.0
        mission._state = PackageDropState.DROP_PAYLOAD

        mission.update(context)
        mission.update(context)

        self.assertEqual(context.sprayer_commands, [False])
        self.assertTrue(mission._drop_actuated)


if __name__ == "__main__":
    unittest.main()
