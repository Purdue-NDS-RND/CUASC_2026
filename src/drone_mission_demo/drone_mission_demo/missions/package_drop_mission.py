"""Package drop mission implementation."""

from __future__ import annotations

from enum import Enum, auto

from drone_mission_core.mission_api import MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission
from rclpy.time import Time
from .red_bullseye_mission_base import (
    HOLD_YAW_DEG,
    TRANSIT_YAW_DEG,
    RedBullseyeMissionBase,
    haversine_distance,
)


class PackageDropState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_GPS = auto()
    TRANSIT_TO_TARGET = auto()
    ACQUIRE_TARGET = auto()
    TARGET_NOT_FOUND = auto()
    TRACK_AND_DESCEND = auto()
    FINAL_FIXED_DROP_COLUMN = auto()
    DROP_PAYLOAD = auto()
    COMPLETE = auto()


@register_mission("package_drop")
class PackageDropMission(RedBullseyeMissionBase):
    """Fly to a GPS target, visually track it, descend, and release payload."""

    def on_enter(self, context: MissionContext) -> None:
        config = self.spec.config
        self._load_common_vision_config(config)
        self._drop_altitude_m = float(config.get("drop_altitude_m", 5.0))
        self._drop_altitude_tolerance_m = float(
            config.get("drop_altitude_tolerance_m", 0.5)
        )
        self._drop_column_handoff_altitude_m = float(
            config.get("drop_column_handoff_altitude_m", 8.0)
        )
        self._descent_rate_mps = float(config.get("descent_rate_mps", 0.5))
        self._drop_hover_dwell_s = float(config.get("drop_hover_dwell_s", 2.0))

        self._state = PackageDropState.INIT
        self._initialize_common_vision_state()
        self._drop_actuated = False
        self._drop_hover_start: Time | None = None
        self._drop_column_hold_position: tuple[float, float] | None = None
        self._drop_column_target_altitude_m: float | None = None
        self._drop_column_last_update: Time | None = None

        self._enter_common_mission(context)

    def on_exit(self, context: MissionContext) -> None:
        self._exit_common_mission(context)

    def update(self, context: MissionContext) -> MissionStatus:
        if self._failed:
            return MissionStatus.FAILURE

        handler = {
            PackageDropState.INIT: self._handle_init,
            PackageDropState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            PackageDropState.WAITING_FOR_GPS: self._handle_waiting_for_gps,
            PackageDropState.TRANSIT_TO_TARGET: self._handle_transit_to_target,
            PackageDropState.ACQUIRE_TARGET: self._handle_acquire_target,
            PackageDropState.TARGET_NOT_FOUND: self._handle_target_not_found,
            PackageDropState.TRACK_AND_DESCEND: self._handle_track_and_descend,
            PackageDropState.FINAL_FIXED_DROP_COLUMN: self._handle_final_fixed_drop_column,
            PackageDropState.DROP_PAYLOAD: self._handle_drop_payload,
            PackageDropState.COMPLETE: self._handle_complete,
        }.get(self._state, self._handle_invalid_state)
        return handler(context)

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        if not self._target_cv_enabled:
            self._request_target_cv_enable(context)
            return MissionStatus.WAITING
        self._transition_to(PackageDropState.WAITING_FOR_CONNECTION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_connection(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING
        self._transition_to(PackageDropState.WAITING_FOR_GPS, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_gps(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.global_gps.status.status < 0:
            return MissionStatus.WAITING
        self._transition_to(PackageDropState.TRANSIT_TO_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_transit_to_target(self, context: MissionContext) -> MissionStatus:
        context.set_global_position_setpoint(
            self._target_latitude,
            self._target_longitude,
            self._transit_altitude_m,
            yaw_deg=TRANSIT_YAW_DEG,
            lock_yaw=False,
        )

        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        ground_distance = haversine_distance(
            context.global_gps.latitude,
            context.global_gps.longitude,
            self._target_latitude,
            self._target_longitude,
        )
        altitude_error = abs(
            context.local_pose.pose.position.z - self._transit_altitude_m
        )
        if (
            ground_distance <= self._arrival_radius_m
            and altitude_error <= self._arrival_alt_tolerance_m
        ):
            context.logger.info(f"[{self.name}] Arrived at GPS drop zone")
            self._transition_to(PackageDropState.ACQUIRE_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_acquire_target(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        self._hold_current_position(context)

        if not self._has_recent_target_detection(context):
            if self._target_loss_grace_expired(context):
                self._transition_to(PackageDropState.TARGET_NOT_FOUND, context)
            return MissionStatus.RUNNING

        self._clear_target_loss_grace()
        if self._centering_dwell_start is None:
            self._centering_dwell_start = context.now()
            context.logger.info(
                f"[{self.name}] Target acquired, holding lock for "
                f"{self._centering_dwell_s:.1f} s"
            )
            return MissionStatus.RUNNING

        if context.seconds_since(self._centering_dwell_start) < self._centering_dwell_s:
            return MissionStatus.RUNNING

        self._transition_to(PackageDropState.TRACK_AND_DESCEND, context)
        return MissionStatus.RUNNING

    def _handle_target_not_found(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        recovery = self._update_target_recovery(context)
        if recovery.failed:
            return MissionStatus.FAILURE
        if recovery.reached_altitude:
            self._transition_to(PackageDropState.ACQUIRE_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_track_and_descend(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING

        command = self._get_centering_descent_command(
            context,
            target_altitude_m=self._drop_altitude_m,
            descent_rate_mps=self._descent_rate_mps,
            target_altitude_tolerance_m=self._drop_altitude_tolerance_m,
        )
        if command is None:
            if self._hold_tracking_loss_grace(context):
                self._transition_to(PackageDropState.TARGET_NOT_FOUND, context)
            return MissionStatus.RUNNING

        self._clear_target_loss_grace()
        if command.tracking_error_m <= self._centering_tolerance_m:
            if (
                context.local_pose.pose.position.z
                <= self._drop_column_handoff_altitude_m
            ):
                context.logger.info(
                    f"[{self.name}] Entering fixed drop column at "
                    f"{context.local_pose.pose.position.z:.2f} m "
                    f"(handoff {self._drop_column_handoff_altitude_m:.2f} m)"
                )
                self._transition_to(PackageDropState.FINAL_FIXED_DROP_COLUMN, context)
                return MissionStatus.RUNNING
            if command.reached_target_altitude:
                if self._centering_dwell_start is None:
                    self._centering_dwell_start = context.now()
                    context.logger.info(
                        f"[{self.name}] At drop altitude and centered, holding for "
                        f"{self._centering_dwell_s:.1f} s"
                    )
                elif context.seconds_since(self._centering_dwell_start) >= self._centering_dwell_s:
                    self._transition_to(PackageDropState.DROP_PAYLOAD, context)
                    return MissionStatus.RUNNING
            else:
                self._centering_dwell_start = None
        else:
            self._centering_dwell_start = None

        context.set_local_velocity_setpoint(
            command.velocity_east_mps,
            command.velocity_north_mps,
            command.vertical_velocity_mps,
            yaw_deg=HOLD_YAW_DEG,
        )
        self._log_velocity_command(
            context,
            east_mps=command.velocity_east_mps,
            north_mps=command.velocity_north_mps,
            up_mps=command.vertical_velocity_mps,
        )
        return MissionStatus.RUNNING

    def _handle_final_fixed_drop_column(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        if self._drop_column_hold_position is None:
            self._drop_column_hold_position = (
                context.global_gps.latitude,
                context.global_gps.longitude,
            )
            self._drop_column_target_altitude_m = context.local_pose.pose.position.z
            self._drop_column_last_update = context.now()
            context.logger.info(
                f"[{self.name}] Freezing drop column at "
                f"({self._drop_column_hold_position[0]:.7f}, "
                f"{self._drop_column_hold_position[1]:.7f})"
            )

        if self._drop_column_target_altitude_m is None:
            self._drop_column_target_altitude_m = context.local_pose.pose.position.z

        now = context.now()
        elapsed_s = 0.0
        if self._drop_column_last_update is not None:
            elapsed_s = max(
                0.0,
                min((now - self._drop_column_last_update).nanoseconds / 1e9, 0.5),
            )
        self._drop_column_last_update = now

        current_altitude = context.local_pose.pose.position.z
        max_descent_step = abs(self._descent_rate_mps) * elapsed_s
        self._drop_column_target_altitude_m = min(
            self._drop_column_target_altitude_m,
            current_altitude,
        )
        self._drop_column_target_altitude_m = max(
            self._drop_altitude_m,
            self._drop_column_target_altitude_m - max_descent_step,
        )

        self._hold_drop_column_position(context, self._drop_column_target_altitude_m)
        if abs(current_altitude - self._drop_altitude_m) <= self._drop_altitude_tolerance_m:
            self._transition_to(PackageDropState.DROP_PAYLOAD, context)
        return MissionStatus.RUNNING

    def _handle_drop_payload(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is not None and context.local_pose is not None:
            self._hold_drop_column_position(context, self._drop_altitude_m)
        else:
            context.set_local_velocity_setpoint(
                0.0,
                0.0,
                0.0,
                yaw_deg=HOLD_YAW_DEG,
            )

        if self._drop_actuated:
            self._transition_to(PackageDropState.COMPLETE, context)
            return MissionStatus.SUCCESS

        if self._drop_hover_start is None:
            self._drop_hover_start = context.now()
            context.logger.info(
                f"[{self.name}] Hovering at drop altitude for "
                f"{self._drop_hover_dwell_s:.1f} s"
            )
            return MissionStatus.RUNNING

        if context.seconds_since(self._drop_hover_start) < self._drop_hover_dwell_s:
            return MissionStatus.RUNNING

        if self._fake_drop:
            context.logger.info(
                f"[{self.name}] Fake drop complete"
            )
            self._transition_to(PackageDropState.COMPLETE, context)
            return MissionStatus.SUCCESS

        if self._actuator_requested:
            return MissionStatus.RUNNING

        if not context.command_service_ready():
            return MissionStatus.WAITING

        context.logger.info(f"[{self.name}] Opening sprayer for payload drop")
        context.command_sprayer(
            enable=True,
            done_callback=self._on_sprayer_response,
        )
        self._actuator_requested = True
        return MissionStatus.RUNNING

    def _handle_complete(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    def _on_sprayer_response(self, future) -> None:
        self._actuator_requested = False
        try:
            result = future.result()
            if result is not None and result.success:
                self._drop_actuated = True
        except Exception:
            self._drop_actuated = False

    def _hold_drop_column_position(
        self,
        context: MissionContext,
        altitude_m: float,
    ) -> None:
        if self._drop_column_hold_position is None:
            self._hold_current_position(context, altitude_m=altitude_m)
            return
        context.set_global_position_setpoint(
            self._drop_column_hold_position[0],
            self._drop_column_hold_position[1],
            altitude_m,
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )

    def _transition_to(
        self,
        new_state: PackageDropState,
        context: MissionContext | None = None,
    ) -> None:
        if new_state == self._state:
            return

        self._centering_dwell_start = None
        self._target_loss_start = None
        self._last_velocity_log_time = None
        self._reset_tracking_filter()
        if new_state != PackageDropState.TARGET_NOT_FOUND:
            self._recovery_target_altitude = None
            self._recovery_hold_position = None
        if new_state not in (
            PackageDropState.FINAL_FIXED_DROP_COLUMN,
            PackageDropState.DROP_PAYLOAD,
        ):
            self._drop_column_hold_position = None
            self._drop_column_target_altitude_m = None
            self._drop_column_last_update = None
        if new_state != PackageDropState.DROP_PAYLOAD:
            self._drop_hover_start = None

        if context is not None:
            context.logger.info(
                f"[{self.name}] State: {self._state.name} -> {new_state.name}"
            )
        self._state = new_state
