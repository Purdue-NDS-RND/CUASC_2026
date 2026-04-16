"""Package delivery mission implementation."""

from __future__ import annotations

import math
from enum import Enum, auto

from drone_mission_core.mission_api import BaseMission, MissionStatus
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.registry import register_mission
from rclpy.time import Time


EARTH_RADIUS_M = 6_371_000.0
TRANSIT_YAW_DEG = 90.0
HOLD_YAW_DEG = 90.0
FINAL_DESCENT_BUFFER_M = 1.0


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
class PackageDeliveryMission(BaseMission):
    """Fly to a GPS target, land on it in GUIDED, deliver, and relaunch."""

    def on_enter(self, context: MissionContext) -> None:
        config = self.spec.config
        self._target_latitude = float(config.get("target_latitude", 0.0))
        self._target_longitude = float(config.get("target_longitude", 0.0))
        self._transit_altitude_m = float(config.get("transit_altitude_m", 20.0))
        self._landing_check_threshold_m = float(
            config.get(
                "landing_check_threshold_m",
                config.get("touchdown_altitude_m", 0.35),
            )
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
        self._fake_drop = bool(config.get("fake_drop", False))
        self._centering_tolerance_px = float(
            config.get("centering_tolerance_px", 30.0)
        )
        self._arrival_radius_m = float(config.get("arrival_radius_m", 3.0))
        self._arrival_alt_tolerance_m = float(
            config.get("arrival_alt_tolerance_m", 2.0)
        )
        self._not_found_ascent_m = float(config.get("not_found_ascent_m", 5.0))
        self._max_recovery_altitude_m = float(
            config.get("max_recovery_altitude_m", self._transit_altitude_m + 10.0)
        )
        self._max_recovery_attempts = int(config.get("max_recovery_attempts", 3))
        self._centering_dwell_s = float(config.get("centering_dwell_s", 1.0))
        self._touchdown_dwell_s = float(config.get("touchdown_dwell_s", 0.5))
        self._servo_channel = int(config.get("servo_channel", 9))
        self._servo_open_pwm = int(config.get("servo_open_pwm", 1900))
        self._gimbal_pitch_deg = float(config.get("gimbal_pitch_deg", -90.0))
        self._gimbal_yaw_deg = float(config.get("gimbal_yaw_deg", 0.0))
        self._centering_gain_mps_per_px = float(
            config.get("centering_gain_mps_per_px", 0.02)
        )
        self._max_centering_speed_mps = float(
            config.get("max_centering_speed_mps", 2.0)
        )
        self._target_timeout_s = float(config.get("target_timeout_s", 2.0))

        self._state = PackageDeliveryState.INIT
        self._gimbal_requested = False
        self._servo_requested = False
        self._delivery_complete = False
        self._recovery_attempts = 0
        self._centering_dwell_start: Time | None = None
        self._touchdown_debounce_start: Time | None = None
        self._ground_dwell_start: Time | None = None
        self._final_descent_hold_position: tuple[float, float] | None = None
        self._final_descent_target_altitude_m: float | None = None
        self._final_descent_last_update: Time | None = None
        self._target_loss_start: Time | None = None
        self._recovery_target_altitude: float | None = None
        self._recovery_hold_position: tuple[float, float] | None = None
        self._target_cv_enabled = False
        self._target_cv_enable_requested = False
        self._failed = (
            self._target_latitude == 0.0 and self._target_longitude == 0.0
        )

        context.clear_all_setpoints()
        context.clear_target_tracking_state()
        self._request_target_cv_enable(context)
        if self._failed:
            context.logger.error(
                f"[{self.name}] target_latitude and target_longitude must be set"
            )

    def on_exit(self, context: MissionContext) -> None:
        context.clear_all_setpoints()
        context.clear_target_tracking_state()
        if context.target_cv_control_ready():
            context.set_target_cv_enabled(False)

    def update(self, context: MissionContext) -> MissionStatus:
        if self._failed:
            return MissionStatus.FAILURE

        self._request_gimbal_if_ready(context)

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
            self._transition_to(PackageDeliveryState.TARGET_NOT_FOUND, context)
            return MissionStatus.RUNNING

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

        if (
            self._recovery_target_altitude is None
            or self._recovery_hold_position is None
        ):
            if self._recovery_attempts >= self._max_recovery_attempts:
                context.logger.error(
                    f"[{self.name}] Target recovery exceeded "
                    f"{self._max_recovery_attempts} attempts"
                )
                return MissionStatus.FAILURE

            current_altitude = context.local_pose.pose.position.z
            next_altitude = current_altitude + self._not_found_ascent_m
            if (
                current_altitude >= self._max_recovery_altitude_m
                or next_altitude > self._max_recovery_altitude_m
            ):
                context.logger.error(
                    f"[{self.name}] Recovery climb would exceed "
                    f"{self._max_recovery_altitude_m:.1f} m"
                )
                return MissionStatus.FAILURE

            self._recovery_target_altitude = min(
                next_altitude,
                self._max_recovery_altitude_m,
            )
            self._recovery_hold_position = (
                context.global_gps.latitude,
                context.global_gps.longitude,
            )
            self._recovery_attempts += 1
            context.logger.info(
                f"[{self.name}] Target lost, climbing to "
                f"{self._recovery_target_altitude:.1f} m "
                f"(attempt {self._recovery_attempts}/{self._max_recovery_attempts})"
            )

        context.set_global_position_setpoint(
            self._recovery_hold_position[0],
            self._recovery_hold_position[1],
            self._recovery_target_altitude,
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )
        altitude_error = abs(
            context.local_pose.pose.position.z - self._recovery_target_altitude
        )
        if altitude_error <= self._arrival_alt_tolerance_m:
            self._transition_to(PackageDeliveryState.ACQUIRE_TARGET, context)
        return MissionStatus.RUNNING

    def _handle_track_and_descend(self, context: MissionContext) -> MissionStatus:
        if context.local_pose is None:
            return MissionStatus.WAITING

        tracking = self._get_tracking_solution(context)
        if tracking is None:
            context.set_local_velocity_setpoint(
                0.0,
                0.0,
                0.0,
                yaw_deg=HOLD_YAW_DEG,
            )
            if context.local_pose.pose.position.z <= self._landing_check_threshold_m:
                context.logger.warn(
                    f"[{self.name}] Lost target inside touchdown band; retrying"
                )
                self._transition_to(PackageDeliveryState.TARGET_NOT_FOUND, context)
                return MissionStatus.RUNNING
            if self._target_loss_start is None:
                self._target_loss_start = context.now()
                return MissionStatus.RUNNING

            if context.seconds_since(self._target_loss_start) >= self._target_timeout_s:
                self._transition_to(PackageDeliveryState.TARGET_NOT_FOUND, context)
            return MissionStatus.RUNNING

        self._target_loss_start = None
        pixel_error, velocity_east, velocity_north = tracking
        current_altitude = context.local_pose.pose.position.z
        final_descent_start_m = self._landing_check_threshold_m + FINAL_DESCENT_BUFFER_M

        vertical_velocity = 0.0
        if pixel_error <= self._centering_tolerance_px:
            if current_altitude <= self._landing_check_threshold_m:
                if not context.landing_state_available():
                    context.logger.error(
                        f"[{self.name}] /mavros/extended_state is required before "
                        "final touchdown descent"
                    )
                    return MissionStatus.FAILURE
                self._transition_to(
                    PackageDeliveryState.FINAL_FIXED_COLUMN_DESCENT,
                    context,
                )
                return MissionStatus.RUNNING
            descent_rate = self._descent_rate_mps
            if current_altitude <= final_descent_start_m:
                descent_rate = self._final_descent_rate_mps
            vertical_velocity = -abs(descent_rate)

        context.set_local_velocity_setpoint(
            velocity_east,
            velocity_north,
            vertical_velocity,
            yaw_deg=HOLD_YAW_DEG,
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
            if self._servo_requested:
                pass
            elif not context.command_service_ready():
                return MissionStatus.WAITING
            else:
                context.logger.info(f"[{self.name}] Delivering payload")
                context.actuate_servo(
                    self._servo_channel,
                    self._servo_open_pwm,
                    self._on_servo_response,
                )
                self._servo_requested = True

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
            context.set_local_velocity_setpoint(
                0.0,
                0.0,
                0.0,
                yaw_deg=HOLD_YAW_DEG,
            )
            self._transition_to(PackageDeliveryState.COMPLETE, context)
            return MissionStatus.SUCCESS

        context.set_local_velocity_setpoint(
            0.0,
            0.0,
            abs(self._guided_relaunch_rate_mps),
            yaw_deg=HOLD_YAW_DEG,
        )
        return MissionStatus.RUNNING

    def _handle_complete(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.SUCCESS

    def _handle_invalid_state(self, _context: MissionContext) -> MissionStatus:
        return MissionStatus.FAILURE

    def _get_tracking_solution(
        self,
        context: MissionContext,
        max_centering_speed_mps: float | None = None,
    ) -> tuple[float, float, float] | None:
        if not self._has_recent_target_detection(context):
            return None

        detection = context.target_detection
        if detection is None:
            return None

        image_width, image_height = context.image_size or (640, 480)
        image_cx = image_width / 2.0
        image_cy = image_height / 2.0
        error_x = detection.point.x - image_cx
        error_y = detection.point.y - image_cy
        pixel_error = math.hypot(error_x, error_y)

        velocity_east = error_x * self._centering_gain_mps_per_px
        velocity_north = -error_y * self._centering_gain_mps_per_px
        speed = math.hypot(velocity_east, velocity_north)
        max_speed = (
            self._max_centering_speed_mps
            if max_centering_speed_mps is None
            else max(0.0, float(max_centering_speed_mps))
        )
        if speed > max_speed and speed > 0.0:
            scale = max_speed / speed
            velocity_east *= scale
            velocity_north *= scale

        return pixel_error, velocity_east, velocity_north

    def _has_recent_target_detection(self, context: MissionContext) -> bool:
        detection = context.target_detection
        if detection is None or detection.point.x < 0.0 or detection.point.y < 0.0:
            return False

        detection_time = Time.from_msg(detection.header.stamp)
        return context.seconds_since(detection_time) < self._target_timeout_s

    def _hold_current_position(
        self,
        context: MissionContext,
        altitude_m: float | None = None,
    ) -> None:
        if context.global_gps is None or context.local_pose is None:
            return

        hold_altitude = (
            context.local_pose.pose.position.z
            if altitude_m is None
            else float(altitude_m)
        )
        context.set_global_position_setpoint(
            context.global_gps.latitude,
            context.global_gps.longitude,
            hold_altitude,
            yaw_deg=HOLD_YAW_DEG,
            lock_yaw=True,
        )

    def _request_gimbal_if_ready(self, context: MissionContext) -> None:
        if self._gimbal_requested or not context.gimbal_service_ready():
            return
        context.point_gimbal(
            self._gimbal_pitch_deg,
            self._gimbal_yaw_deg,
            self._on_gimbal_response,
        )
        self._gimbal_requested = True
        context.logger.info(
            f"[{self.name}] Pointing gimbal to "
            f"pitch={self._gimbal_pitch_deg:.1f}, yaw={self._gimbal_yaw_deg:.1f}"
        )

    def _on_gimbal_response(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.success:
                self._gimbal_requested = False
        except Exception:
            self._gimbal_requested = False

    def _on_servo_response(self, future) -> None:
        self._servo_requested = False
        try:
            result = future.result()
            if result is not None and result.success:
                self._delivery_complete = True
        except Exception:
            self._delivery_complete = False

    def _request_target_cv_enable(self, context: MissionContext) -> None:
        if self._target_cv_enabled or self._target_cv_enable_requested:
            return
        if not context.target_cv_control_ready():
            return

        context.logger.info(f"[{self.name}] Enabling target detection")
        context.set_target_cv_enabled(True, self._on_target_cv_enable_response)
        self._target_cv_enable_requested = True

    def _on_target_cv_enable_response(self, future) -> None:
        self._target_cv_enable_requested = False
        try:
            result = future.result()
            self._target_cv_enabled = bool(result is not None and result.success)
        except Exception:
            self._target_cv_enabled = False

    def _transition_to(
        self,
        new_state: PackageDeliveryState,
        context: MissionContext | None = None,
    ) -> None:
        if new_state == self._state:
            return

        self._centering_dwell_start = None
        self._target_loss_start = None
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
