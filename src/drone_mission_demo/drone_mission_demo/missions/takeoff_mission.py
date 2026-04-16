"""Takeoff mission implementation."""

from __future__ import annotations

from enum import Enum, auto

from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission


class TakeoffState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_TAKEOFF_SERVICE = auto()
    TAKING_OFF = auto()
    WAITING_FOR_ALTITUDE = auto()
    COMPLETE = auto()


@register_mission("takeoff")
class TakeoffMission(BaseMission):
    """Request takeoff and wait until the altitude gate is met."""

    def on_enter(self, context: MissionContext) -> None:
        self._target_altitude_m = float(
            self.spec.config.get("target_altitude_m", 20.0)
        )
        self._request_sent = False
        self._state = TakeoffState.INIT

    def update(self, context: MissionContext) -> MissionStatus:
        handler = {
            TakeoffState.INIT: self._handle_init,
            TakeoffState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            TakeoffState.WAITING_FOR_TAKEOFF_SERVICE: self._handle_waiting_for_takeoff_service,
            TakeoffState.TAKING_OFF: self._handle_taking_off,
            TakeoffState.WAITING_FOR_ALTITUDE: self._handle_waiting_for_altitude,
            TakeoffState.COMPLETE: self._handle_complete,
        }.get(self._state, self._handle_invalid_state)
        return handler(context)

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        self._transition_to(TakeoffState.WAITING_FOR_CONNECTION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_connection(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING
        self._transition_to(TakeoffState.WAITING_FOR_TAKEOFF_SERVICE, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_takeoff_service(self, context: MissionContext) -> MissionStatus:
        if not context.takeoff_service_ready():
            return MissionStatus.WAITING
        self._transition_to(TakeoffState.TAKING_OFF, context)
        return MissionStatus.RUNNING

    def _handle_taking_off(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is not None:
            current_alt = context.local_pose.pose.position.z
            if current_alt >= self._target_altitude_m * 0.9:
                self._transition_to(TakeoffState.COMPLETE, context)
                return MissionStatus.SUCCESS

        if self._request_sent:
            if context.mavros_state is not None and context.mavros_state.armed:
                self._transition_to(TakeoffState.WAITING_FOR_ALTITUDE, context)
            return MissionStatus.RUNNING

        context.logger.info(
            f"[{self.name}] Requesting takeoff to {self._target_altitude_m:.1f} m"
        )
        context.request_takeoff(
            self._target_altitude_m,
            self._on_takeoff_response,
        )
        self._request_sent = True
        return MissionStatus.RUNNING

    def _handle_waiting_for_altitude(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING

        current_alt = context.local_pose.pose.position.z
        if current_alt >= self._target_altitude_m * 0.9:
            self._transition_to(TakeoffState.COMPLETE, context)
            return MissionStatus.SUCCESS
        return MissionStatus.RUNNING

    def _handle_complete(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    def _on_takeoff_response(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.success:
                self._request_sent = False
        except Exception:
            self._request_sent = False

    def _transition_to(
        self,
        new_state: TakeoffState,
        context: MissionContext | None = None,
    ) -> None:
        if context is not None and new_state != self._state:
            context.logger.info(f"[{self.name}] State: {self._state.name} -> {new_state.name}")
        self._state = new_state
