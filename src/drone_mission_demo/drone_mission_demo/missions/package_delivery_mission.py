"""Package delivery mission implementation."""

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

FINAL_DESCENT_BUFFER_M = 1.0


class PackageDeliveryState(Enum):
    INIT = auto()
    WAITING_FOR_CONNECTION = auto()
    WAITING_FOR_GPS = auto()
    TRANSIT_TO_TARGET = auto()
    ACQUIRE_TARGET = auto()
    TARGET_NOT_FOUND = auto()
    TRACK_AND_DESCEND = auto()
    FINAL_FIXED_COLUMN_DESCENT = auto()
    GROUND_DWELL = auto()
    GUIDED_RELAUNCH = auto()
    COMPLETE = auto()


@register_mission("package_delivery")
class PackageDeliveryMission(RedBullseyeMissionBase):
    """Fly to a GPS target, land on it in GUIDED, deliver, and relaunch."""

    def on_enter(self, context: MissionContext) -> None:
        config = self.spec.config
        self._load_common_vision_config(config)
        self._landing_check_threshold_m = float(
            config.get(
                "landing_check_threshold_m",
                config.get("touchdown_altitude_m", 0.35),
            )
        )
        self._touchdown_handoff_tolerance_m = float(
            config.get("touchdown_handoff_tolerance_m", 0.2)
        )
        self._relaunch_altitude_m = float(
            config.get("relaunch_altitude_m", self._transit_altitude_m)
        )
        self._descent_rate_mps = float(config.get("descent_rate_mps", 0.5))
        self._final_descent_rate_mps = float(
            config.get("final_descent_rate_mps", 0.2)
        )
        self._delivery_dwell_s = float(config.get("delivery_dwell_s", 5.0))
        self._guided_relaunch_rate_mps = float(
            config.get("guided_relaunch_rate_mps", 0.6)
        )
        self._guided_relaunch_max_climb_rate_mps = float(
            config.get("guided_relaunch_max_climb_rate_mps", 2.5)
        )
        self._touchdown_dwell_s = float(config.get("touchdown_dwell_s", 0.5))

        self._state = PackageDeliveryState.INIT
        self._initialize_common_vision_state()
        self._delivery_complete = False
        self._touchdown_debounce_start: Time | None = None
        self._ground_dwell_start: Time | None = None
        self._final_descent_hold_position: tuple[float, float] | None = None
        self._final_descent_target_altitude_m: float | None = None
        self._final_descent_last_update: Time | None = None

        self._enter_common_mission(context)

    def on_exit(self, context: MissionContext) -> None:
        self._exit_common_mission(context)

    def update(self, context: MissionContext) -> MissionStatus:
        if self._failed:
            return MissionStatus.FAILURE

        handler = {
            PackageDeliveryState.INIT: self._handle_init,
            PackageDeliveryState.WAITING_FOR_CONNECTION: self._handle_waiting_for_connection,
            PackageDeliveryState.WAITING_FOR_GPS: self._handle_waiting_for_gps,
            PackageDeliveryState.TRANSIT_TO_TARGET: self._handle_transit_to_target,
            PackageDeliveryState.ACQUIRE_TARGET: self._handle_acquire_target,
            PackageDeliveryState.TARGET_NOT_FOUND: self._handle_target_not_found,
            PackageDeliveryState.TRACK_AND_DESCEND: self._handle_track_and_descend,
            PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT: self._handle_final_fixed_column_descent,
            PackageDeliveryState.GROUND_DWELL: self._handle_ground_dwell,
            PackageDeliveryState.GUIDED_RELAUNCH: self._handle_guided_relaunch,
            PackageDeliveryState.COMPLETE: self._handle_complete,
        }.get(self._state, self._handle_invalid_state)
        return handler(context)

    def _handle_init(self, context: MissionContext) -> MissionStatus:
        if not self._target_cv_enabled:
            self._request_target_cv_enable(context)
            return MissionStatus.WAITING
        self._transition_to(PackageDeliveryState.WAITING_FOR_CONNECTION, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_connection(self, context: MissionContext) -> MissionStatus:
        if context.mavros_state is None or not context.mavros_state.connected:
            return MissionStatus.WAITING
        self._transition_to(PackageDeliveryState.WAITING_FOR_GPS, context)
        return MissionStatus.RUNNING

    def _handle_waiting_for_gps(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.global_gps.status.status < 0:
            return MissionStatus.WAITING
        self._transition_to(PackageDeliveryState.TRANSIT_TO_TARGET, context)
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
            context.logger.info(f"[{self.name}] Arrived at GPS delivery zone")
            self._transition_to(PackageDeliveryState.ACQUIRE_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_acquire_target(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        self._hold_current_position(context)

        if not self._has_recent_target_detection(context):
            if self._target_loss_grace_expired(context):
                self._transition_to(PackageDeliveryState.TARGET_NOT_FOUND, context)
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

        self._transition_to(PackageDeliveryState.TRACK_AND_DESCEND, context)
        return MissionStatus.RUNNING

    def _handle_target_not_found(self, context: MissionContext) -> MissionStatus:
        if context.global_gps is None or context.local_pose is None:
            return MissionStatus.WAITING

        recovery = self._update_target_recovery(context)
        if recovery.failed:
            return MissionStatus.FAILURE
        if recovery.reached_altitude:
            self._transition_to(PackageDeliveryState.ACQUIRE_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_track_and_descend(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING

        descent_rate = self._descent_rate_mps
        if (
            context.local_pose.pose.position.z
            <= self._landing_check_threshold_m + FINAL_DESCENT_BUFFER_M
        ):
            descent_rate = self._final_descent_rate_mps

        command = self._get_centering_descent_command(
            context,
            target_altitude_m=self._landing_check_threshold_m,
            descent_rate_mps=descent_rate,
            target_altitude_tolerance_m=self._touchdown_handoff_tolerance_m,
        )
        if command is None:
            if self._hold_tracking_loss_grace(context):
                self._transition_to(PackageDeliveryState.TARGET_NOT_FOUND, context)
            return MissionStatus.RUNNING

        self._clear_target_loss_grace()
        if command.tracking_error_m <= self._centering_tolerance_m:
            if command.reached_target_altitude:
                if not context.landing_state_available():
                    context.logger.error(
                        f"[{self.name}] /mavros/extended_state is required before "
                        "final touchdown descent"
                    )
                    return MissionStatus.FAILURE
                context.logger.info(
                    f"[{self.name}] Entering final touchdown column at "
                    f"{context.local_pose.pose.position.z:.2f} m "
                    f"(threshold {self._landing_check_threshold_m:.2f} m, "
                    f"handoff tolerance {self._touchdown_handoff_tolerance_m:.2f} m)"
                )
                self._transition_to(
                    PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT,
                    context,
                )
                return MissionStatus.RUNNING

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

            if context.seconds_since(self._touchdown_debounce_start) < self._touchdown_dwell_s:
                return MissionStatus.RUNNING

            context.logger.info(f"[{self.name}] Touchdown confirmed by FCU landed state")
            self._transition_to(PackageDeliveryState.GROUND_DWELL, context)
            return MissionStatus.RUNNING

        self._touchdown_debounce_start = None
        if self._final_descent_hold_position is None:
            self._final_descent_hold_position = (
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
                PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT,
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
                f"[{self.name}] Holding on target for "
                f"{self._delivery_dwell_s:.1f} s before relaunch"
            )
            if self._fake_drop:
                context.logger.info(f"[{self.name}] Fake delivery complete")
                self._delivery_complete = True

        if not self._delivery_complete and not self._fake_drop:
            if self._actuator_requested:
                pass
            elif not context.command_service_ready():
                return MissionStatus.WAITING
            else:
                context.logger.info(f"[{self.name}] Releasing gripper payload")
                context.command_gripper(
                    release=True,
                    done_callback=self._on_gripper_response,
                )
                self._actuator_requested = True

        if context.seconds_since(self._ground_dwell_start) < self._delivery_dwell_s:
            return MissionStatus.RUNNING
        if not self._delivery_complete:
            return MissionStatus.RUNNING

        self._transition_to(PackageDeliveryState.GUIDED_RELAUNCH, context)
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

        current_alt = context.local_pose.pose.position.z
        if current_alt >= self._relaunch_altitude_m * 0.9:
            context.clear_all_setpoints()
            self._transition_to(PackageDeliveryState.COMPLETE, context)
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

    def _on_gripper_response(self, future) -> None:
        self._actuator_requested = False
        try:
            result = future.result()
            if result is not None and result.success:
                self._delivery_complete = True
        except Exception:
            self._delivery_complete = False

    def _transition_to(
        self,
        new_state: PackageDeliveryState,
        context: MissionContext | None = None,
    ) -> None:
        if new_state == self._state:
            return

        self._centering_dwell_start = None
        self._target_loss_start = None
        self._last_velocity_log_time = None
        self._reset_tracking_filter()
        if new_state != PackageDeliveryState.TARGET_NOT_FOUND:
            self._recovery_target_altitude = None
            self._recovery_hold_position = None
        if new_state != PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT:
            self._touchdown_debounce_start = None
            self._final_descent_hold_position = None
            self._final_descent_target_altitude_m = None
            self._final_descent_last_update = None
        if new_state != PackageDeliveryState.GROUND_DWELL:
            self._ground_dwell_start = None

        if context is not None:
            context.logger.info(
                f"[{self.name}] State: {self._state.name} -> {new_state.name}"
            )
        self._state = new_state
