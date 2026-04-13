"""RTL mission implementation."""

from __future__ import annotations

from enum import Enum, auto

from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission


class RTLState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    REQUESTING_RTL = auto()
    WAITING_FOR_MODE = auto()
    COMPLETE = auto()


@register_mission("rtl")
class RTLMission(BaseMission):
    """Request RTL mode and wait until the FCU reports it."""

    def on_enter(self, context: MissionContext) -> None:
        self._rtl_mode = str(self.spec.config.get("rtl_mode", "RTL"))
        self._request_sent = False
        self._state = RTLState.INIT
        context.clear_local_position_setpoint()

    def update(self, context: MissionContext) -> MissionStatus:
        handler = {
            RTLState.INIT: self._handle_init,
            RTLState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            RTLState.REQUESTING_RTL: self._handle_requesting_rtl,
            RTLState.WAITING_FOR_MODE: self._handle_waiting_for_mode,
            RTLState.COMPLETE: self._handle_complete,
        }.get(self._state, self._handle_invalid_state)
        return handler(context)

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        self._transition_to(RTLState.WAITING_FOR_CONNECTION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_connection(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING
        self._transition_to(RTLState.REQUESTING_RTL, context)
        return MissionStatus.RUNNING

    def _handle_requesting_rtl(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is not None and context.mavros_state.mode == self._rtl_mode:
            self._transition_to(RTLState.COMPLETE, context)
            return MissionStatus.SUCCESS

        if self._request_sent:
            self._transition_to(RTLState.WAITING_FOR_MODE, context)
            return MissionStatus.RUNNING

        if not context.mode_service_ready():
            return MissionStatus.WAITING

        context.logger.info(f"[{self.name}] Requesting mode {self._rtl_mode}")
        context.request_mode_change(self._rtl_mode, self._on_mode_response)
        self._request_sent = True
        return MissionStatus.RUNNING

    def _handle_waiting_for_mode(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING

        if context.mavros_state.mode == self._rtl_mode:
            self._transition_to(RTLState.COMPLETE, context)
            return MissionStatus.SUCCESS
        return MissionStatus.RUNNING

    def _handle_complete(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    def _on_mode_response(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.mode_sent:
                self._request_sent = False
                self._transition_to(RTLState.REQUESTING_RTL)
        except Exception:
            self._request_sent = False
            self._transition_to(RTLState.REQUESTING_RTL)

    def _transition_to(
        self,
        new_state: RTLState,
        context: MissionContext | None = None,
    ) -> None:
        if context is not None and new_state != self._state:
            context.logger.info(f"[{self.name}] State: {self._state.name} -> {new_state.name}")
        self._state = new_state
