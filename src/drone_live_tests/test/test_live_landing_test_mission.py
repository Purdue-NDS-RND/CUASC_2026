"""Focused tests for the standalone live landing test mission package."""

from __future__ import annotations

import importlib.util
import os
import sys
import unittest
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus


THIS_FILE = Path(__file__).resolve()
PACKAGE_ROOT = THIS_FILE.parents[1]
SRC_ROOT = THIS_FILE.parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))
sys.path.insert(0, str(SRC_ROOT / "drone_mission_core"))

from drone_mission_core.mission_api import MissionSpec, MissionStatus
from drone_live_tests.missions.live_landing_test_mission import (
    LiveLandingTestMission,
    LiveLandingTestState,
)


class _LoggerStub:
    def info(self, _msg: str) -> None:
        return None

    def warn(self, _msg: str) -> None:
        return None

    def error(self, _msg: str) -> None:
        return None


class _ContextStub:
    def __init__(
        self,
        *,
        altitude_m: float = 10.0,
        latitude: float = 40.4237,
        longitude: float = -86.9212,
        connected: bool = True,
        armed: bool = True,
        landed_state: int = ExtendedState.LANDED_STATE_IN_AIR,
    ) -> None:
        self.logger = _LoggerStub()
        self.current_time_s = 0.0

        self.mavros_state = State()
        self.mavros_state.connected = connected
        self.mavros_state.armed = armed

        self.local_pose = PoseStamped()
        self.local_pose.pose.position.z = altitude_m

        self.global_gps = NavSatFix()
        self.global_gps.latitude = latitude
        self.global_gps.longitude = longitude
        self.global_gps.status = NavSatStatus()
        self.global_gps.status.status = NavSatStatus.STATUS_FIX

        self.extended_state = ExtendedState()
        self.extended_state.landed_state = landed_state

        self.last_global_setpoint = None
        self.last_local_velocity_setpoint = None
        self.last_attitude_climb_rate_setpoint = None
        self.clear_all_setpoints_called = False

    def now(self) -> Time:
        return Time(nanoseconds=int(self.current_time_s * 1e9))

    def seconds_since(self, start_time: Time) -> float:
        return (self.now() - start_time).nanoseconds / 1e9

    def landing_state_available(self) -> bool:
        return (
            self.extended_state.landed_state
            != ExtendedState.LANDED_STATE_UNDEFINED
        )

    def vehicle_is_landed(self) -> bool:
        return self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def clear_all_setpoints(self) -> None:
        self.clear_all_setpoints_called = True

    def set_global_position_setpoint(
        self,
        latitude: float,
        longitude: float,
        altitude_m: float,
        yaw_deg: float = 90.0,
        lock_yaw: bool = True,
    ) -> None:
        self.last_global_setpoint = (
            latitude,
            longitude,
            altitude_m,
            yaw_deg,
            lock_yaw,
        )

    def set_local_velocity_setpoint(
        self,
        east_mps: float,
        north_mps: float,
        up_mps: float = 0.0,
        yaw_deg: float = 90.0,
    ) -> None:
        self.last_local_velocity_setpoint = (
            east_mps,
            north_mps,
            up_mps,
            yaw_deg,
        )

    def set_attitude_climb_rate_setpoint(
        self,
        climb_rate_mps: float,
        yaw_deg: float = 90.0,
        max_climb_rate_mps: float = 2.5,
    ) -> None:
        self.last_attitude_climb_rate_setpoint = (
            climb_rate_mps,
            yaw_deg,
            max_climb_rate_mps,
        )


def _make_mission(**config_overrides) -> LiveLandingTestMission:
    config = {
        "landing_check_threshold_m": 6.0,
        "arrival_alt_tolerance_m": 2.0,
        "relaunch_altitude_m": 12.0,
        "descent_rate_mps": 0.35,
        "final_descent_rate_mps": 0.25,
        "delivery_dwell_s": 5.0,
        "guided_relaunch_rate_mps": 0.6,
        "guided_relaunch_max_climb_rate_mps": 2.5,
        "touchdown_dwell_s": 0.5,
    }
    config.update(config_overrides)
    return LiveLandingTestMission(
        MissionSpec(
            type_name="live_landing_test",
            name="live_landing_test",
            config=config,
            base_dir=Path("."),
        )
    )


def _advance_to_active_descent(
    mission: LiveLandingTestMission,
    context: _ContextStub,
) -> None:
    while mission._state != LiveLandingTestState.DESCEND_IN_COLUMN:
        status = mission.update(context)
        assert status in (MissionStatus.RUNNING, MissionStatus.WAITING)


def _collect_substitution_texts(value) -> list[str]:
    if hasattr(value, "text"):
        return [value.text]
    if isinstance(value, (list, tuple)):
        collected: list[str] = []
        for item in value:
            collected.extend(_collect_substitution_texts(item))
        return collected
    return []


class LiveLandingTestMissionTests(unittest.TestCase):
    def test_descends_without_vision_inputs(self) -> None:
        mission = _make_mission()
        context = _ContextStub(altitude_m=10.0)
        mission.on_enter(context)

        _advance_to_active_descent(mission, context)
        context.current_time_s = 1.0

        status = mission.update(context)

        self.assertEqual(status, MissionStatus.RUNNING)
        self.assertIsNotNone(context.last_global_setpoint)
        self.assertEqual(mission._state, LiveLandingTestState.DESCEND_IN_COLUMN)
        self.assertLess(context.last_global_setpoint[2], 10.0)

    def test_current_column_is_frozen_from_start(self) -> None:
        mission = _make_mission()
        context = _ContextStub(altitude_m=10.0, latitude=35.1234567, longitude=-86.7654321)
        mission.on_enter(context)

        _advance_to_active_descent(mission, context)
        frozen_column = mission._touchdown_column
        self.assertEqual(frozen_column, (35.1234567, -86.7654321))

        context.global_gps.latitude = 36.0000000
        context.global_gps.longitude = -87.0000000
        context.current_time_s = 1.0
        mission.update(context)

        self.assertEqual(context.last_global_setpoint[0], frozen_column[0])
        self.assertEqual(context.last_global_setpoint[1], frozen_column[1])

    def test_handoff_to_final_touchdown_occurs_at_threshold(self) -> None:
        mission = _make_mission()
        context = _ContextStub(altitude_m=6.0)
        mission.on_enter(context)

        while mission._state != LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT:
            status = mission.update(context)
            self.assertEqual(status, MissionStatus.RUNNING)

        self.assertEqual(mission._state, LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT)

    def test_handoff_to_final_touchdown_occurs_within_altitude_tolerance(self) -> None:
        mission = _make_mission(landing_check_threshold_m=6.0, arrival_alt_tolerance_m=0.5)
        context = _ContextStub(altitude_m=6.2)
        mission.on_enter(context)

        while mission._state != LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT:
            status = mission.update(context)
            self.assertEqual(status, MissionStatus.RUNNING)

        self.assertEqual(mission._state, LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT)

    def test_touchdown_requires_debounce(self) -> None:
        mission = _make_mission(touchdown_dwell_s=0.5)
        context = _ContextStub(altitude_m=0.2, landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        mission.on_enter(context)
        mission._touchdown_column = (context.global_gps.latitude, context.global_gps.longitude)
        mission._state = LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT

        first_status = mission.update(context)
        context.current_time_s = 0.2
        second_status = mission.update(context)
        context.current_time_s = 0.6
        third_status = mission.update(context)

        self.assertEqual(first_status, MissionStatus.RUNNING)
        self.assertEqual(second_status, MissionStatus.RUNNING)
        self.assertEqual(third_status, MissionStatus.RUNNING)
        self.assertEqual(mission._state, LiveLandingTestState.GROUND_DWELL)

    def test_disarm_fails_ground_dwell_and_guided_relaunch(self) -> None:
        mission = _make_mission()
        context = _ContextStub(altitude_m=0.0, armed=False, landed_state=ExtendedState.LANDED_STATE_ON_GROUND)
        mission.on_enter(context)

        mission._state = LiveLandingTestState.GROUND_DWELL
        self.assertEqual(mission.update(context), MissionStatus.FAILURE)

        mission._state = LiveLandingTestState.GUIDED_RELAUNCH
        self.assertEqual(mission.update(context), MissionStatus.FAILURE)

    def test_guided_relaunch_succeeds_at_ninety_percent_altitude(self) -> None:
        mission = _make_mission(relaunch_altitude_m=12.0)
        context = _ContextStub(altitude_m=10.8, landed_state=ExtendedState.LANDED_STATE_IN_AIR)
        mission.on_enter(context)
        mission._state = LiveLandingTestState.GUIDED_RELAUNCH

        status = mission.update(context)

        self.assertEqual(status, MissionStatus.SUCCESS)
        self.assertEqual(mission._state, LiveLandingTestState.COMPLETE)
        self.assertTrue(context.clear_all_setpoints_called)


class LiveLandingTestLaunchTests(unittest.TestCase):
    def test_launch_uses_only_mission_executor_and_live_test_module(self) -> None:
        os.environ["ROS_LOG_DIR"] = "/tmp/codex_ros_logs"
        launch_path = PACKAGE_ROOT / "launch" / "live_landing_test.launch.py"
        spec = importlib.util.spec_from_file_location("live_landing_test_launch", launch_path)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)

        launch_description = module.generate_launch_description()
        nodes = [entity for entity in launch_description.entities if hasattr(entity, "node_package")]
        param_map = nodes[0]._Node__parameters[1]
        key_names = {
            "".join(_collect_substitution_texts(key)): value
            for key, value in param_map.items()
        }

        self.assertEqual(len(nodes), 1)
        self.assertEqual(nodes[0].node_package, "drone_mission_core")
        self.assertEqual(nodes[0].node_executable, "mission_executor")
        self.assertIn("mission_modules", key_names)
        self.assertIn("sequence_file", key_names)
        self.assertTrue(
            any(
                "drone_live_tests.missions" in item
                for item in _collect_substitution_texts(key_names["mission_modules"])
            )
        )
        self.assertIn("FindPackageShare(pkg='drone_live_tests')", str(key_names["sequence_file"]))
        self.assertNotIn("simple_takeoff_service", str(launch_description.entities))
        self.assertNotIn("target_cv", str(launch_description.entities))
        self.assertNotIn("gimbal_point_service", str(launch_description.entities))


if __name__ == "__main__":
    unittest.main()
