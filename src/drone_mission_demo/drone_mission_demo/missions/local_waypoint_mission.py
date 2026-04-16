"""Local waypoint mission implementation."""

from __future__ import annotations

import math
from enum import Enum, auto

import yaml

from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission


class LocalWaypointState(Enum):
    INIT = auto()
    GO_TO_WAYPOINT = auto()
    HOLD_AT_WAYPOINT = auto()
    ADVANCE_WAYPOINT = auto()
    COMPLETE = auto()


@register_mission("local_waypoint")
class LocalWaypointMission(BaseMission):
    """Fly a reusable list of local ENU waypoints."""

    def on_enter(self, context: MissionContext) -> None:
        self._state = LocalWaypointState.INIT
        self._current_waypoint_index = 0
        self._hold_start_time = None

        self._waypoint_altitude_m = float(
            self.spec.config.get("waypoint_altitude_m", 20.0)
        )
        self._arrival_radius_m = float(
            self.spec.config.get("arrival_radius_m", 3.0)
        )
        self._arrival_height_tolerance_m = float(
            self.spec.config.get("arrival_height_tolerance_m", 2.0)
        )
        self._hold_time_s = float(self.spec.config.get("hold_time_s", 3.0))
        self._desired_yaw_deg = float(self.spec.config.get("desired_yaw_deg", 90.0))
        self._waypoints = self._load_waypoints()
        self._failed = not bool(self._waypoints)

        if self._failed:
            context.logger.error(f"[{self.name}] No waypoints configured")

    def update(self, context: MissionContext) -> MissionStatus:
        if self._failed:
            return MissionStatus.FAILURE

        handler = {
            LocalWaypointState.INIT: self._handle_init,
            LocalWaypointState.GO_TO_WAYPOINT: self._handle_go_to_waypoint,
            LocalWaypointState.HOLD_AT_WAYPOINT: self._handle_hold_at_waypoint,
            LocalWaypointState.ADVANCE_WAYPOINT: self._handle_advance_waypoint,
            LocalWaypointState.COMPLETE: self._handle_complete,
        }.get(self._state, self._handle_invalid_state)
        return handler(context)

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        self._set_current_waypoint_setpoint(context)
        self._transition_to(LocalWaypointState.GO_TO_WAYPOINT, context)
        return MissionStatus.RUNNING

    def _handle_go_to_waypoint(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING

        self._set_current_waypoint_setpoint(context)
        if self._check_arrival(context):
            self._hold_start_time = context.now()
            context.logger.info(
                f"[{self.name}] Reached waypoint #{self._current_waypoint_index + 1}"
            )
            self._transition_to(LocalWaypointState.HOLD_AT_WAYPOINT, context)
        return MissionStatus.RUNNING

    def _handle_hold_at_waypoint(self, context: MissionContext) -> MissionStatus:
        if self._hold_start_time is None:
            self._hold_start_time = context.now()
            return MissionStatus.RUNNING

        if context.seconds_since(self._hold_start_time) >= self._hold_time_s:
            self._transition_to(LocalWaypointState.ADVANCE_WAYPOINT, context)
        return MissionStatus.RUNNING

    def _handle_advance_waypoint(self, context: MissionContext) -> MissionStatus:
        self._current_waypoint_index += 1
        if self._current_waypoint_index >= len(self._waypoints):
            self._transition_to(LocalWaypointState.COMPLETE, context)
            return MissionStatus.SUCCESS

        self._set_current_waypoint_setpoint(context)
        self._transition_to(LocalWaypointState.GO_TO_WAYPOINT, context)
        return MissionStatus.RUNNING

    def _handle_complete(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    def _load_waypoints(self) -> list[tuple[float, float, float]]:
        inline_waypoints = self.spec.config.get("waypoints")
        if isinstance(inline_waypoints, list):
            return self._normalize_waypoints(inline_waypoints)

        pattern_file = self.spec.config.get("pattern_file")
        if not isinstance(pattern_file, str) or not pattern_file.strip():
            return []

        resolved_path = self.spec.resolve_path(pattern_file.strip())
        with resolved_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}

        waypoint_list = raw.get("waypoints")
        if not isinstance(waypoint_list, list):
            return []
        return self._normalize_waypoints(waypoint_list)

    def _normalize_waypoints(self, raw_waypoints) -> list[tuple[float, float, float]]:
        normalized: list[tuple[float, float, float]] = []
        for index, entry in enumerate(raw_waypoints):
            if not isinstance(entry, (list, tuple)) or len(entry) < 2:
                raise ValueError(
                    f"Malformed waypoint #{index + 1} in mission '{self.name}': {entry}"
                )
            normalized.append(
                (
                    float(entry[0]),
                    float(entry[1]),
                    self._waypoint_altitude_m,
                )
            )
        return normalized

    def _set_current_waypoint_setpoint(self, context: MissionContext) -> None:
        east_m, north_m, up_m = self._waypoints[self._current_waypoint_index]
        context.set_local_position_setpoint(
            east_m,
            north_m,
            up_m,
            yaw_deg=self._desired_yaw_deg,
        )

    def _check_arrival(self, context: MissionContext) -> bool:
        east_m, north_m, up_m = self._waypoints[self._current_waypoint_index]
        pose = context.local_pose.pose.position
        horizontal_dist = math.sqrt(
            (pose.x - east_m) ** 2 + (pose.y - north_m) ** 2
        )
        vertical_dist = abs(pose.z - up_m)
        return (
            horizontal_dist <= self._arrival_radius_m
            and vertical_dist <= self._arrival_height_tolerance_m
        )

    def _transition_to(
        self,
        new_state: LocalWaypointState,
        context: MissionContext | None = None,
    ) -> None:
        if context is not None and new_state != self._state:
            context.logger.info(f"[{self.name}] State: {self._state.name} -> {new_state.name}")
        self._state = new_state
