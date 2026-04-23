"""Autonomous Lawnmower Survey Mission for Object Localization."""

from __future__ import annotations

import math
from enum import Enum, auto

from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission


class ObjectLocalizationState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_GPS = auto()
    CALCULATE_GRID = auto()
    FLYING_WAYPOINTS = auto()
    COMPLETE = auto()


@register_mission("object_localization_survey")
class ObjectLocalizationMission(BaseMission):
    """Generates and flies a lawnmower grid over bounded GPS coordinates."""

    def on_enter(self, context: MissionContext) -> None:
        config = self.spec.config
        self._survey_altitude_m = float(config.get("altitude_m", 25.0))
        self._line_spacing_m = float(config.get("spacing_m", 28.0))
        self._arrival_tolerance_m = float(config.get("arrival_tolerance_m", 2.0))
        self._corners = config.get("corners", [])

        # Competition hard-deck constraint: 25 feet = 7.62 meters
        self._minimum_safe_altitude_m = 7.62

        self._state = ObjectLocalizationState.INIT
        self._waypoints: list[tuple[float, float]] = []
        self._current_wp_idx = 0

        context.clear_all_setpoints()

    def on_exit(self, context: MissionContext) -> None:
        context.clear_all_setpoints()

    def update(self, context: MissionContext) -> MissionStatus:
        # Enforce competition penalty hard-deck safety check
        if context.local_pose is not None:
            if context.local_pose.pose.position.z < self._minimum_safe_altitude_m:
                context.logger.error(
                    f"[{self.name}] HARD DECK VIOLATION: Altitude dropped below "
                    f"{self._minimum_safe_altitude_m}m. Aborting to prevent penalty."
                )
                return MissionStatus.FAILURE

        handler = {
            ObjectLocalizationState.INIT: self._handle_init,
            ObjectLocalizationState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            ObjectLocalizationState.WAITING_FOR_GPS: self._handle_waiting_for_gps,
            ObjectLocalizationState.CALCULATE_GRID: self._handle_calculate_grid,
            ObjectLocalizationState.FLYING_WAYPOINTS: self._handle_flying_waypoints,
            ObjectLocalizationState.COMPLETE: self._handle_complete,
        }.get(self._state, self._handle_invalid_state)

        return handler(context)

    # ------------------------------------------------------------------
    # State Handlers
    # ------------------------------------------------------------------

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        if len(self._corners) < 3:
            context.logger.error(f"[{self.name}] Invalid corners provided in YAML.")
            return MissionStatus.FAILURE

        self._transition_to(ObjectLocalizationState.WAITING_FOR_CONNECTION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_connection(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING
        self._transition_to(ObjectLocalizationState.WAITING_FOR_GPS, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_gps(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.global_gps.status.status < 0:
            return MissionStatus.WAITING
        self._transition_to(ObjectLocalizationState.CALCULATE_GRID, context)
        return MissionStatus.RUNNING

    def _handle_calculate_grid(self, context: MissionContext) -> MissionStatus:
        context.logger.info(f"[{self.name}] Calculating lawnmower grid...")

        lats = [c[0] for c in self._corners]
        lons = [c[1] for c in self._corners]

        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        # Approximate degrees per meter (Roughly 111,320 meters per degree latitude)
        lat_step = self._line_spacing_m / 111320.0

        current_lat = min_lat
        going_east = True

        while current_lat <= max_lat:
            if going_east:
                self._waypoints.append((current_lat, min_lon))
                self._waypoints.append((current_lat, max_lon))
            else:
                self._waypoints.append((current_lat, max_lon))
                self._waypoints.append((current_lat, min_lon))

            going_east = not going_east
            current_lat += lat_step

        context.logger.info(
            f"[{self.name}] Generated {len(self._waypoints)} survey waypoints."
        )
        self._transition_to(ObjectLocalizationState.FLYING_WAYPOINTS, context)
        return MissionStatus.RUNNING

    def _handle_flying_waypoints(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None:
            return MissionStatus.WAITING

        if self._current_wp_idx >= len(self._waypoints):
            context.logger.info(f"[{self.name}] Survey complete!")
            self._transition_to(ObjectLocalizationState.COMPLETE, context)
            return MissionStatus.RUNNING

        target_lat, target_lon = self._waypoints[self._current_wp_idx]

        # Continuously command the executor to fly to the current waypoint
        context.set_global_position_setpoint(
            target_lat, target_lon, self._survey_altitude_m
        )

        # Check if we are within the arrival tolerance
        dist_m = self._calculate_distance_m(
            context.global_gps.latitude,
            context.global_gps.longitude,
            target_lat,
            target_lon,
        )

        if dist_m <= self._arrival_tolerance_m:
            context.logger.info(
                f"[{self.name}] Reached waypoint {self._current_wp_idx + 1}/{len(self._waypoints)}"
            )
            self._current_wp_idx += 1

        return MissionStatus.RUNNING

    def _handle_complete(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    # ------------------------------------------------------------------
    # Math Helpers
    # ------------------------------------------------------------------

    def _transition_to(
        self,
        new_state: ObjectLocalizationState,
        context: MissionContext | None = None,
    ) -> None:
        if new_state == self._state:
            return
        if context is not None:
            context.logger.info(
                f"[{self.name}] State: {self._state.name} -> {new_state.name}"
            )
        self._state = new_state

    def _calculate_distance_m(
        self, lat1: float, lon1: float, lat2: float, lon2: float
    ) -> float:
        """Haversine distance in meters."""
        R = 6378137.0
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        a = (
            math.sin(d_lat / 2) ** 2
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(d_lon / 2) ** 2
        )
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
