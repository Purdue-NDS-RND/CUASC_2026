import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [
    str(ROOT / "src" / "drone_mission_demo"),
    str(ROOT / "src" / "drone_mission_core"),
]


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
    def safe_load(raw):
        if hasattr(raw, "read"):
            raw = raw.read()
        return json.loads(raw)

    yaml_mod = types.ModuleType("yaml")
    yaml_mod.safe_load = safe_load
    sys.modules.setdefault("yaml", yaml_mod)

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
from drone_mission_core.registry import MISSION_REGISTRY


def _load_gps_waypoint_module():
    MISSION_REGISTRY.pop("gps_waypoint", None)
    module_path = (
        ROOT
        / "src"
        / "drone_mission_demo"
        / "drone_mission_demo"
        / "missions"
        / "gps_waypoint_mission.py"
    )
    spec = importlib.util.spec_from_file_location(
        "gps_waypoint_mission_under_test",
        module_path,
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


gps_waypoint_module = _load_gps_waypoint_module()
GpsWaypointMission = gps_waypoint_module.GpsWaypointMission
GpsWaypointState = gps_waypoint_module.GpsWaypointState


class FakeLogger:
    def __init__(self) -> None:
        self.messages = []

    def info(self, msg: str) -> None:
        self.messages.append(("info", msg))

    def warn(self, msg: str) -> None:
        self.messages.append(("warn", msg))

    def error(self, msg: str) -> None:
        self.messages.append(("error", msg))


class FakeContext:
    def __init__(self) -> None:
        self._now_ns = 0
        self.logger = FakeLogger()
        self.mavros_state = SimpleNamespace(connected=True)
        self.global_gps = SimpleNamespace(
            latitude=40.0,
            longitude=-86.0,
            status=SimpleNamespace(status=0),
        )
        self.local_pose = SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(z=20.0))
        )
        self.global_position_commands = []

    def now(self) -> FakeTime:
        return FakeTime(self._now_ns)

    def advance(self, seconds: float) -> None:
        self._now_ns += int(seconds * 1e9)

    def seconds_since(self, start_time: FakeTime) -> float:
        return (self.now() - start_time).nanoseconds / 1e9

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


def make_mission(**config_overrides) -> GpsWaypointMission:
    config = {
        "waypoints": [
            {"name": "wp1", "latitude": 40.0, "longitude": -86.0},
            {"name": "wp2", "latitude": 40.0001, "longitude": -86.0001},
        ],
        "waypoint_altitude_m": 20.0,
        "arrival_radius_m": 1.0,
        "arrival_alt_tolerance_m": 1.0,
        "hold_time_s": 0.0,
        "mode": "waypoint_navigation",
    }
    config.update(config_overrides)
    spec = MissionSpec(
        type_name="gps_waypoint",
        name="gps_waypoint",
        config=config,
        base_dir=Path("."),
    )
    mission = GpsWaypointMission(spec)
    mission.on_enter(FakeContext())
    return mission


class GpsWaypointMissionTests(unittest.TestCase):
    def test_loads_inline_gps_waypoints(self) -> None:
        mission = make_mission()

        self.assertEqual([wp.name for wp in mission._waypoints], ["wp1", "wp2"])
        self.assertEqual(mission._waypoints[0].latitude, 40.0)
        self.assertEqual(mission._waypoints[1].longitude, -86.0001)
        self.assertEqual(mission._waypoints[0].altitude_m, 20.0)

    def test_loads_waypoint_file(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            waypoint_path = Path(temp_dir) / "waypoints.yaml"
            waypoint_path.write_text(
                json.dumps(
                    {
                        "waypoints": [
                            {
                                "name": "start",
                                "latitude": 40.1,
                                "longitude": -86.1,
                                "altitude_agl_ft": 150,
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            mission = make_mission(
                waypoints=None,
                waypoint_file=str(waypoint_path),
            )

        self.assertEqual(len(mission._waypoints), 1)
        self.assertEqual(mission._waypoints[0].name, "start")
        self.assertAlmostEqual(mission._waypoints[0].altitude_m, 45.72)

    def test_converts_feet_altitude_to_global_setpoint_meters(self) -> None:
        mission = make_mission(
            waypoints=[
                {
                    "name": "wp1",
                    "latitude": 40.0,
                    "longitude": -86.0,
                    "altitude_agl_ft": 50,
                }
            ]
        )
        context = FakeContext()

        mission.update(context)
        mission.update(context)
        mission.update(context)

        self.assertEqual(context.global_position_commands[-1][:2], (40.0, -86.0))
        self.assertAlmostEqual(context.global_position_commands[-1][2], 15.24)

    def test_fails_cleanly_when_no_waypoints_configured(self) -> None:
        context = FakeContext()
        spec = MissionSpec(
            type_name="gps_waypoint",
            name="gps_waypoint",
            config={"waypoints": None},
            base_dir=Path("."),
        )
        mission = GpsWaypointMission(spec)
        mission.on_enter(context)

        self.assertEqual(mission.update(context), MissionStatus.FAILURE)
        self.assertTrue(
            any("No GPS waypoints configured" in msg for _, msg in context.logger.messages)
        )

    def test_waits_for_valid_gps_before_navigating(self) -> None:
        mission = make_mission()
        context = FakeContext()
        context.global_gps.status.status = -1

        mission.update(context)
        mission.update(context)
        status = mission.update(context)

        self.assertEqual(status, MissionStatus.WAITING)
        self.assertEqual(mission._state, GpsWaypointState.WAITING_FOR_GPS)
        self.assertEqual(context.global_position_commands, [])

    def test_publishes_global_setpoint_for_active_waypoint(self) -> None:
        mission = make_mission()
        context = FakeContext()

        mission.update(context)
        mission.update(context)
        mission.update(context)

        self.assertEqual(
            context.global_position_commands[-1],
            (40.0, -86.0, 20.0, 90.0, False),
        )

    def test_advances_after_horizontal_and_altitude_arrival(self) -> None:
        mission = make_mission()
        context = FakeContext()

        mission.update(context)
        mission.update(context)
        mission.update(context)
        mission.update(context)
        mission.update(context)

        self.assertEqual(mission._current_waypoint_index, 1)
        self.assertEqual(mission._state, GpsWaypointState.GO_TO_WAYPOINT)

        context.global_gps.latitude = 40.0001
        context.global_gps.longitude = -86.0001
        mission.update(context)
        status = mission.update(context)

        self.assertEqual(status, MissionStatus.SUCCESS)
        self.assertEqual(mission._state, GpsWaypointState.COMPLETE)

    def test_records_minimum_observed_distances(self) -> None:
        mission = make_mission()
        context = FakeContext()

        mission.update(context)

        self.assertAlmostEqual(mission._min_distances_m[0], 0.0, places=6)
        self.assertGreater(mission._min_distances_m[1], 0.0)

    def test_circuit_time_trial_records_elapsed_time(self) -> None:
        mission = make_mission(
            arrival_radius_m=4.0,
            mode="circuit_time_trial",
        )
        context = FakeContext()

        mission.update(context)
        mission.update(context)
        mission.update(context)
        context.advance(1.0)
        mission.update(context)

        context.global_gps.latitude = 40.0001
        context.global_gps.longitude = -86.0001
        context.advance(9.0)
        mission.update(context)
        mission.update(context)

        self.assertIsNotNone(mission._route_start_time)
        self.assertIsNotNone(mission._route_end_time)
        elapsed_s = (
            mission._route_end_time - mission._route_start_time
        ).nanoseconds / 1e9
        self.assertAlmostEqual(elapsed_s, 9.0)


if __name__ == "__main__":
    unittest.main()
