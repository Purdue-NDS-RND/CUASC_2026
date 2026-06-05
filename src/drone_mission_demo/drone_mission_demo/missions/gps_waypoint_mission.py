"""GPS waypoint mission implementation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any

import yaml
from rclpy.time import Time

from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission


EARTH_RADIUS_M = 6_371_000.0


@dataclass(frozen=True)
class GpsWaypoint:
    name: str
    latitude: float
    longitude: float


class GpsWaypointState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_GPS = auto()
    GO_TO_WAYPOINT = auto()
    HOLD_AT_WAYPOINT = auto()
    ADVANCE_WAYPOINT = auto()
    COMPLETE = auto()


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Horizontal distance in meters between two GPS points."""

    lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2.0) ** 2
    )
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


@register_mission("gps_waypoint")
class GpsWaypointMission(BaseMission):
    """Fly a configured GPS waypoint route in YAML order."""

    MODE_WAYPOINT_NAVIGATION = "waypoint_navigation"
    MODE_CIRCUIT_TIME_TRIAL = "circuit_time_trial"

    def on_enter(self, context: MissionContext) -> None:
        self._state = GpsWaypointState.INIT
        self._current_waypoint_index = 0
        self._hold_start_time: Time | None = None
        self._route_start_time: Time | None = None
        self._route_end_time: Time | None = None
        self._summary_logged = False

        self._mode = str(
            self.spec.config.get("mode", self.MODE_WAYPOINT_NAVIGATION)
        ).strip().lower()
        self._waypoint_altitude_m = float(
            self.spec.config.get("waypoint_altitude_m", 20.0)
        )
        self._arrival_radius_m = float(self.spec.config.get("arrival_radius_m", 3.0))
        self._arrival_alt_tolerance_m = float(
            self.spec.config.get("arrival_alt_tolerance_m", 2.0)
        )
        self._hold_time_s = float(self.spec.config.get("hold_time_s", 0.0))
        self._desired_yaw_deg = float(self.spec.config.get("desired_yaw_deg", 90.0))

        self._waypoints = self._load_waypoints()
        self._min_distances_m = [math.inf for _ in self._waypoints]
        self._failed = not bool(self._waypoints) or self._mode not in {
            self.MODE_WAYPOINT_NAVIGATION,
            self.MODE_CIRCUIT_TIME_TRIAL,
        }

        if not self._waypoints:
            context.logger.error(f"[{self.name}] No GPS waypoints configured")
        if self._mode not in {
            self.MODE_WAYPOINT_NAVIGATION,
            self.MODE_CIRCUIT_TIME_TRIAL,
        }:
            context.logger.error(f"[{self.name}] Unsupported GPS waypoint mode: {self._mode}")

    def on_exit(self, context: MissionContext) -> None:
        self._log_summary(context)

    def update(self, context: MissionContext) -> MissionStatus:
        if self._failed:
            return MissionStatus.FAILURE

        self._update_min_distances(context)
        handler = {
            GpsWaypointState.INIT: self._handle_init,
            GpsWaypointState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            GpsWaypointState.WAITING_FOR_GPS: self._handle_waiting_for_gps,
            GpsWaypointState.GO_TO_WAYPOINT: self._handle_go_to_waypoint,
            GpsWaypointState.HOLD_AT_WAYPOINT: self._handle_hold_at_waypoint,
            GpsWaypointState.ADVANCE_WAYPOINT: self._handle_advance_waypoint,
            GpsWaypointState.COMPLETE: self._handle_complete,
        }.get(self._state, self._handle_invalid_state)
        return handler(context)

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        self._transition_to(GpsWaypointState.WAITING_FOR_CONNECTION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_connection(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING
        self._transition_to(GpsWaypointState.WAITING_FOR_GPS, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_gps(self, context: MissionContext) -> MissionStatus:
        if not self._gps_fix_valid(context):
            return MissionStatus.WAITING
        self._set_current_waypoint_setpoint(context)
        self._transition_to(GpsWaypointState.GO_TO_WAYPOINT, context)
        return MissionStatus.RUNNING

    def _handle_go_to_waypoint(self, context: MissionContext) -> MissionStatus:
        self._set_current_waypoint_setpoint(context)
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        if self._check_arrival(context):
            waypoint = self._waypoints[self._current_waypoint_index]
            context.logger.info(
                f"[{self.name}] Reached GPS waypoint "
                f"#{self._current_waypoint_index + 1} ({waypoint.name})"
            )
            self._record_circuit_timing(context)
            if self._hold_time_s > 0.0 and not self._circuit_final_waypoint_reached():
                self._hold_start_time = context.now()
                self._transition_to(GpsWaypointState.HOLD_AT_WAYPOINT, context)
            else:
                self._transition_to(GpsWaypointState.ADVANCE_WAYPOINT, context)
        return MissionStatus.RUNNING

    def _handle_hold_at_waypoint(self, context: MissionContext) -> MissionStatus:
        self._set_current_waypoint_setpoint(context)
        if self._hold_start_time is None:
            self._hold_start_time = context.now()
            return MissionStatus.RUNNING

        if context.seconds_since(self._hold_start_time) >= self._hold_time_s:
            self._transition_to(GpsWaypointState.ADVANCE_WAYPOINT, context)
        return MissionStatus.RUNNING

    def _handle_advance_waypoint(self, context: MissionContext) -> MissionStatus:
        self._current_waypoint_index += 1
        self._hold_start_time = None
        if self._current_waypoint_index >= len(self._waypoints):
            self._transition_to(GpsWaypointState.COMPLETE, context)
            self._log_summary(context)
            return MissionStatus.SUCCESS

        self._set_current_waypoint_setpoint(context)
        self._transition_to(GpsWaypointState.GO_TO_WAYPOINT, context)
        return MissionStatus.RUNNING

    def _handle_complete(self, context: MissionContext) -> MissionStatus:
        self._log_summary(context)
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    def _load_waypoints(self) -> list[GpsWaypoint]:
        inline_waypoints = self.spec.config.get("waypoints")
        if isinstance(inline_waypoints, list):
            return self._normalize_waypoints(inline_waypoints)

        waypoint_file = self.spec.config.get("waypoint_file")
        if not isinstance(waypoint_file, str) or not waypoint_file.strip():
            return []

        resolved_path = self.spec.resolve_path(waypoint_file.strip())
        with resolved_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}

        waypoint_list = raw.get("waypoints")
        if not isinstance(waypoint_list, list):
            return []
        return self._normalize_waypoints(waypoint_list)

    def _normalize_waypoints(self, raw_waypoints: list[Any]) -> list[GpsWaypoint]:
        normalized: list[GpsWaypoint] = []
        for index, entry in enumerate(raw_waypoints):
            name = f"wp{index + 1}"
            if isinstance(entry, dict):
                raw_name = entry.get("name")
                if isinstance(raw_name, str) and raw_name.strip():
                    name = raw_name.strip()
                latitude = entry.get("latitude")
                longitude = entry.get("longitude")
            elif isinstance(entry, (list, tuple)) and len(entry) >= 2:
                latitude = entry[0]
                longitude = entry[1]
            else:
                raise ValueError(
                    f"Malformed GPS waypoint #{index + 1} in mission "
                    f"'{self.name}': {entry}"
                )

            normalized.append(
                GpsWaypoint(
                    name=name,
                    latitude=float(latitude),
                    longitude=float(longitude),
                )
            )
        return normalized

    def _gps_fix_valid(self, context: MissionContext) -> bool:
        gps = context.global_gps
        if gps is None:
            return False
        status = getattr(getattr(gps, "status", None), "status", 0)
        return status >= 0

    def _set_current_waypoint_setpoint(self, context: MissionContext) -> None:
        waypoint = self._waypoints[self._current_waypoint_index]
        context.set_global_position_setpoint(
            waypoint.latitude,
            waypoint.longitude,
            self._waypoint_altitude_m,
            yaw_deg=self._desired_yaw_deg,
            lock_yaw=False,
        )

    def _check_arrival(self, context: MissionContext) -> bool:
        if context.global_gps is None or context.local_pose is None:
            return False

        waypoint = self._waypoints[self._current_waypoint_index]
        ground_distance = haversine_distance(
            context.global_gps.latitude,
            context.global_gps.longitude,
            waypoint.latitude,
            waypoint.longitude,
        )
        altitude_error = abs(
            context.local_pose.pose.position.z - self._waypoint_altitude_m
        )
        return (
            ground_distance <= self._arrival_radius_m
            and altitude_error <= self._arrival_alt_tolerance_m
        )

    def _update_min_distances(self, context: MissionContext) -> None:
        gps = context.global_gps
        if gps is None:
            return

        for index, waypoint in enumerate(self._waypoints):
            distance_m = haversine_distance(
                gps.latitude,
                gps.longitude,
                waypoint.latitude,
                waypoint.longitude,
            )
            self._min_distances_m[index] = min(
                self._min_distances_m[index],
                distance_m,
            )

    def _record_circuit_timing(self, context: MissionContext) -> None:
        if self._mode != self.MODE_CIRCUIT_TIME_TRIAL:
            return

        if self._current_waypoint_index == 0 and self._route_start_time is None:
            self._route_start_time = context.now()
            context.logger.info(f"[{self.name}] Circuit timer started")

        if self._circuit_final_waypoint_reached() and self._route_end_time is None:
            self._route_end_time = context.now()
            if self._route_start_time is not None:
                elapsed_s = context.seconds_since(self._route_start_time)
                context.logger.info(
                    f"[{self.name}] Circuit timer stopped at {elapsed_s:.2f} s"
                )

    def _circuit_final_waypoint_reached(self) -> bool:
        return (
            self._mode == self.MODE_CIRCUIT_TIME_TRIAL
            and self._current_waypoint_index == len(self._waypoints) - 1
        )

    def _log_summary(self, context: MissionContext) -> None:
        if self._summary_logged or not self._waypoints:
            return

        context.logger.info(f"[{self.name}] GPS waypoint advisory summary:")
        for waypoint, distance_m in zip(self._waypoints, self._min_distances_m):
            if math.isinf(distance_m):
                context.logger.info(f"[{self.name}]   {waypoint.name}: no GPS samples")
            else:
                context.logger.info(
                    f"[{self.name}]   {waypoint.name}: closest {distance_m:.2f} m"
                )

        if (
            self._mode == self.MODE_CIRCUIT_TIME_TRIAL
            and self._route_start_time is not None
            and self._route_end_time is not None
        ):
            elapsed_s = (
                self._route_end_time - self._route_start_time
            ).nanoseconds / 1e9
            context.logger.info(f"[{self.name}]   circuit elapsed: {elapsed_s:.2f} s")

        self._summary_logged = True

    def _transition_to(
        self,
        new_state: GpsWaypointState,
        context: MissionContext | None = None,
    ) -> None:
        if context is not None and new_state != self._state:
            context.logger.info(f"[{self.name}] State: {self._state.name} -> {new_state.name}")
        self._state = new_state
