"""Standalone live-flight landing and relaunch test mission."""

from __future__ import annotations

from enum import Enum, auto

from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission
from rclpy.time import Time


HOLD_YAW_DEG = 90.0


class LiveLandingTestState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_LOCAL_POSITION = auto()
    WAITING_FOR_GPS = auto()
    WAITING_FOR_LANDING_STATE = auto()
    CAPTURE_START_COLUMN = auto()
    DESCEND_IN_COLUMN = auto()
    FINAL_FIXED_COLUMN_DESCENT = auto()
    GROUND_DWELL = auto()
    GUIDED_RELAUNCH = auto()
    COMPLETE = auto()


@register_mission("live_landing_test")
class LiveLandingTestMission(BaseMission):
    """Descend in the current column, touch down while armed, then relaunch."""

    def on_enter(self, context: MissionContext) -> None:
        config = self.spec.config
        self._landing_check_threshold_m = float(
            config.get("landing_check_threshold_m", 6.0)
        )
        self._relaunch_altitude_m = float(config.get("relaunch_altitude_m", 12.0))
        self._descent_rate_mps = float(config.get("descent_rate_mps", 0.35))
        self._final_descent_rate_mps = float(
            config.get("final_descent_rate_mps", 0.25)
        )
        self._delivery_dwell_s = float(config.get("delivery_dwell_s", 5.0))
        self._guided_relaunch_rate_mps = float(
            config.get("guided_relaunch_rate_mps", 0.6)
        )
        self._guided_relaunch_max_climb_rate_mps = float(
            config.get("guided_relaunch_max_climb_rate_mps", 2.5)
        )
        self._touchdown_dwell_s = float(config.get("touchdown_dwell_s", 0.5))

        self._state = LiveLandingTestState.INIT
        self._touchdown_column: tuple[float, float] | None = None
        self._descent_target_altitude_m: float | None = None
        self._descent_last_update: Time | None = None
        self._touchdown_debounce_start: Time | None = None
        self._ground_dwell_start: Time | None = None
        self._final_descent_hold_position: tuple[float, float] | None = None
        self._final_descent_target_altitude_m: float | None = None
        self._final_descent_last_update: Time | None = None

        context.clear_all_setpoints()

    def on_exit(self, context: MissionContext) -> None:
        context.clear_all_setpoints()

    def update(self, context: MissionContext) -> MissionStatus:
        handler = {
            LiveLandingTestState.INIT: self._handle_init,
            LiveLandingTestState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            LiveLandingTestState.WAITING_FOR_LOCAL_POSITION: self._handle_waiting_for_local_position,
            LiveLandingTestState.WAITING_FOR_GPS: self._handle_waiting_for_gps,
            LiveLandingTestState.WAITING_FOR_LANDING_STATE: self._handle_waiting_for_landing_state,
            LiveLandingTestState.CAPTURE_START_COLUMN: self._handle_capture_start_column,
            LiveLandingTestState.DESCEND_IN_COLUMN: self._handle_descend_in_column,
            LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT: self._handle_final_fixed_column_descent,
            LiveLandingTestState.GROUND_DWELL: self._handle_ground_dwell,
            LiveLandingTestState.GUIDED_RELAUNCH: self._handle_guided_relaunch,
            LiveLandingTestState.COMPLETE: self._handle_complete,
        }.get(self._state, self._handle_invalid_state)
        return handler(context)

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        self._transition_to(LiveLandingTestState.WAITING_FOR_CONNECTION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_connection(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING
        self._transition_to(LiveLandingTestState.WAITING_FOR_LOCAL_POSITION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_local_position(
        self,
        context: MissionContext,
    ) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING
        self._transition_to(LiveLandingTestState.WAITING_FOR_GPS, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_gps(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.global_gps.status.status < 0:
            return MissionStatus.WAITING
        self._transition_to(LiveLandingTestState.WAITING_FOR_LANDING_STATE, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_landing_state(
        self,
        context: MissionContext,
    ) -> MissionStatus:
        if not context.landing_state_available():
            return MissionStatus.WAITING
        self._transition_to(LiveLandingTestState.CAPTURE_START_COLUMN, context)
        return MissionStatus.RUNNING

    def _handle_capture_start_column(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None or context.global_gps is None:
            return MissionStatus.WAITING
        if not context.landing_state_available():
            context.logger.error(
                f"[{self.name}] /mavros/extended_state is required before descent"
            )
            return MissionStatus.FAILURE
        if context.vehicle_is_landed():
            context.logger.error(
                f"[{self.name}] Mission must start while airborne after manual takeoff"
            )
            return MissionStatus.FAILURE

        self._touchdown_column = (
            context.global_gps.latitude,
            context.global_gps.longitude,
        )
        current_altitude = context.local_pose.pose.position.z
        self._descent_target_altitude_m = current_altitude
        self._descent_last_update = context.now()
        context.logger.info(
            f"[{self.name}] Captured landing column at "
            f"({self._touchdown_column[0]:.7f}, {self._touchdown_column[1]:.7f})"
        )

        if current_altitude <= self._landing_check_threshold_m:
            self._transition_to(
                LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT,
                context,
            )
            return MissionStatus.RUNNING

        self._transition_to(LiveLandingTestState.DESCEND_IN_COLUMN, context)
        return MissionStatus.RUNNING

    def _handle_descend_in_column(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None or context.global_gps is None:
            return MissionStatus.WAITING
        if self._touchdown_column is None:
            context.logger.error(f"[{self.name}] No touchdown column captured")
            return MissionStatus.FAILURE
        if not context.landing_state_available():
            context.logger.error(
                f"[{self.name}] Lost /mavros/extended_state during descent"
            )
            return MissionStatus.FAILURE

        current_altitude = context.local_pose.pose.position.z
        if current_altitude <= self._landing_check_threshold_m:
            self._transition_to(
                LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT,
                context,
            )
            return MissionStatus.RUNNING

        if self._descent_target_altitude_m is None:
            self._descent_target_altitude_m = current_altitude

        now = context.now()
        elapsed_s = 0.0
        if self._descent_last_update is not None:
            elapsed_s = max(
                0.0,
                min((now - self._descent_last_update).nanoseconds / 1e9, 0.5),
            )
        self._descent_last_update = now

        max_descent_step = abs(self._descent_rate_mps) * elapsed_s
        self._descent_target_altitude_m = min(
            self._descent_target_altitude_m,
            current_altitude,
        )
        self._descent_target_altitude_m = max(
            self._landing_check_threshold_m,
            self._descent_target_altitude_m - max_descent_step,
        )

        context.set_global_position_setpoint(
            self._touchdown_column[0],
            self._touchdown_column[1],
            self._descent_target_altitude_m,
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )
        return MissionStatus.RUNNING

    def _handle_final_fixed_column_descent(
        self,
        context: MissionContext,
    ) -> MissionStatus:
        if context.local_pose is None or context.global_gps is None:
            return MissionStatus.WAITING
        if not context.landing_state_available():
            context.logger.error(
                f"[{self.name}] Lost /mavros/extended_state during touchdown descent"
            )
            return MissionStatus.FAILURE

        if context.vehicle_is_landed():
            context.set_local_velocity_setpoint(
                0.0,
                0.0,
                0.0,
                yaw_deg=HOLD_YAW_DEG,
            )
            if self._touchdown_debounce_start is None:
                self._touchdown_debounce_start = context.now()
                context.logger.info(
                    f"[{self.name}] Ground contact detected, debouncing for "
                    f"{self._touchdown_dwell_s:.1f} s"
                )
                return MissionStatus.RUNNING

            if (
                context.seconds_since(self._touchdown_debounce_start)
                < self._touchdown_dwell_s
            ):
                return MissionStatus.RUNNING

            context.logger.info(f"[{self.name}] Touchdown confirmed by FCU landed state")
            self._transition_to(LiveLandingTestState.GROUND_DWELL, context)
            return MissionStatus.RUNNING

        self._touchdown_debounce_start = None
        if self._final_descent_hold_position is None:
            self._final_descent_hold_position = self._touchdown_column or (
                context.global_gps.latitude,
                context.global_gps.longitude,
            )
            self._final_descent_target_altitude_m = context.local_pose.pose.position.z
            self._final_descent_last_update = context.now()
            context.logger.info(
                f"[{self.name}] Freezing touchdown column at "
                f"({self._final_descent_hold_position[0]:.7f}, "
                f"{self._final_descent_hold_position[1]:.7f})"
            )

        if self._final_descent_target_altitude_m is None:
            self._final_descent_target_altitude_m = context.local_pose.pose.position.z

        now = context.now()
        elapsed_s = 0.0
        if self._final_descent_last_update is not None:
            elapsed_s = max(
                0.0,
                min((now - self._final_descent_last_update).nanoseconds / 1e9, 0.5),
            )
        self._final_descent_last_update = now

        current_altitude = context.local_pose.pose.position.z
        max_descent_step = abs(self._final_descent_rate_mps) * elapsed_s
        self._final_descent_target_altitude_m = min(
            self._final_descent_target_altitude_m,
            current_altitude,
        )
        self._final_descent_target_altitude_m = max(
            0.0,
            self._final_descent_target_altitude_m - max_descent_step,
        )

        context.set_global_position_setpoint(
            self._final_descent_hold_position[0],
            self._final_descent_hold_position[1],
            self._final_descent_target_altitude_m,
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )
        return MissionStatus.RUNNING

    def _handle_ground_dwell(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is not None and not context.mavros_state.armed:
            context.logger.error(
                f"[{self.name}] Vehicle disarmed on the ground before relaunch. "
                "Increase FCU DISARM_DELAY or reduce delivery_dwell_s."
            )
            return MissionStatus.FAILURE
        if not context.landing_state_available():
            context.logger.error(
                f"[{self.name}] Lost /mavros/extended_state during ground dwell"
            )
            return MissionStatus.FAILURE
        if not context.vehicle_is_landed():
            self._transition_to(
                LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT,
                context,
            )
            return MissionStatus.RUNNING

        context.set_local_velocity_setpoint(
            0.0,
            0.0,
            0.0,
            yaw_deg=HOLD_YAW_DEG,
        )

        if self._ground_dwell_start is None:
            self._ground_dwell_start = context.now()
            context.logger.info(
                f"[{self.name}] Holding on the ground for "
                f"{self._delivery_dwell_s:.1f} s before relaunch"
            )

        if context.seconds_since(self._ground_dwell_start) < self._delivery_dwell_s:
            return MissionStatus.RUNNING

        self._transition_to(LiveLandingTestState.GUIDED_RELAUNCH, context)
        return MissionStatus.RUNNING

    def _handle_guided_relaunch(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING
        if context.mavros_state is not None and not context.mavros_state.armed:
            context.logger.error(
                f"[{self.name}] Vehicle disarmed before guided relaunch. "
                "Touch-and-go requires staying armed on the ground."
            )
            return MissionStatus.FAILURE

        current_altitude = context.local_pose.pose.position.z
        if current_altitude >= self._relaunch_altitude_m * 0.9:
            context.clear_all_setpoints()
            self._transition_to(LiveLandingTestState.COMPLETE, context)
            return MissionStatus.SUCCESS

        context.set_attitude_climb_rate_setpoint(
            abs(self._guided_relaunch_rate_mps),
            yaw_deg=HOLD_YAW_DEG,
            max_climb_rate_mps=self._guided_relaunch_max_climb_rate_mps,
        )
        return MissionStatus.RUNNING

    def _handle_complete(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    def _transition_to(
        self,
        new_state: LiveLandingTestState,
        context: MissionContext | None = None,
    ) -> None:
        if new_state == self._state:
            return

        if new_state != LiveLandingTestState.DESCEND_IN_COLUMN:
            self._descent_last_update = None
        if new_state != LiveLandingTestState.FINAL_FIXED_COLUMN_DESCENT:
            self._touchdown_debounce_start = None
            self._final_descent_hold_position = None
            self._final_descent_target_altitude_m = None
            self._final_descent_last_update = None
        if new_state != LiveLandingTestState.GROUND_DWELL:
            self._ground_dwell_start = None

        if context is not None:
            context.logger.info(
                f"[{self.name}] State: {self._state.name} -> {new_state.name}"
            )
        self._state = new_state
