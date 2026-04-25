"""Shared helpers for vision-guided payload missions."""

from __future__ import annotations

import math
from dataclasses import dataclass

from drone_mission_core.mission_api import BaseMission
from drone_mission_core.mission_context import MissionContext
from drone_mission_core.tracking_projection import (
    DEFAULT_CAMERA_CALIBRATION_HEIGHT_PX,
    DEFAULT_CAMERA_CALIBRATION_WIDTH_PX,
    DEFAULT_CAMERA_FX_PX,
    DEFAULT_CAMERA_FY_PX,
    CameraIntrinsics,
    ground_offset_to_velocity,
    project_normalized_detection_to_ground_offset,
)
from rclpy.time import Time


EARTH_RADIUS_M = 6_371_000.0
TRANSIT_YAW_DEG = 90.0
HOLD_YAW_DEG = 90.0


@dataclass(frozen=True)
class TrackingVelocityCommand:
    tracking_error_m: float
    velocity_east_mps: float
    velocity_north_mps: float
    vertical_velocity_mps: float
    reached_target_altitude: bool


@dataclass(frozen=True)
class RecoveryCommandResult:
    failed: bool
    reached_altitude: bool


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


class RedBullseyeMissionBase(BaseMission):
    """Common targeting and centering helpers for payload missions."""

    def _load_common_vision_config(self, config: dict) -> None:
        self._target_latitude = float(config.get("target_latitude", 0.0))
        self._target_longitude = float(config.get("target_longitude", 0.0))
        self._transit_altitude_m = float(config.get("transit_altitude_m", 20.0))
        self._fake_drop = bool(config.get("fake_drop", False))
        self._centering_tolerance_m = float(config.get("centering_tolerance_m", 0.35))
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
        self._centering_deadband_m = float(config.get("centering_deadband_m", 0.08))
        self._tracking_low_pass_alpha = float(
            config.get("tracking_low_pass_alpha", 0.25)
        )
        self._centering_gain_mps_per_m = float(
            config.get("centering_gain_mps_per_m", 0.75)
        )
        self._max_centering_speed_mps = float(
            config.get("max_centering_speed_mps", 2.0)
        )
        self._camera_intrinsics = CameraIntrinsics(
            fx_px=float(config.get("camera_fx_px", DEFAULT_CAMERA_FX_PX)),
            fy_px=float(config.get("camera_fy_px", DEFAULT_CAMERA_FY_PX)),
            calibration_width_px=float(
                config.get(
                    "camera_calibration_width_px",
                    DEFAULT_CAMERA_CALIBRATION_WIDTH_PX,
                )
            ),
            calibration_height_px=float(
                config.get(
                    "camera_calibration_height_px",
                    DEFAULT_CAMERA_CALIBRATION_HEIGHT_PX,
                )
            ),
        )
        self._camera_yaw_offset_deg = float(config.get("camera_yaw_offset_deg", 0.0))
        self._min_projection_altitude_m = float(
            config.get("min_projection_altitude_m", 0.75)
        )
        self._max_projection_distance_m = float(
            config.get("max_projection_distance_m", 25.0)
        )
        self._target_timeout_s = float(config.get("target_timeout_s", 2.0))
        self._target_loss_grace_s = float(
            config.get("target_loss_grace_s", self._target_timeout_s)
        )
        self._recovery_alt_tolerance_m = float(
            config.get("recovery_alt_tolerance_m", self._arrival_alt_tolerance_m)
        )
        self._failed = self._target_latitude == 0.0 and self._target_longitude == 0.0

    def _initialize_common_vision_state(self) -> None:
        self._actuator_requested = False
        self._recovery_attempts = 0
        self._centering_dwell_start: Time | None = None
        self._target_loss_start: Time | None = None
        self._recovery_target_altitude: float | None = None
        self._recovery_hold_position: tuple[float, float] | None = None
        self._target_cv_enabled = False
        self._target_cv_enable_requested = False
        self._filtered_tracking_east_m: float | None = None
        self._filtered_tracking_north_m: float | None = None
        self._last_velocity_log_time: Time | None = None

    def _enter_common_mission(self, context: MissionContext) -> None:
        context.clear_all_setpoints()
        context.clear_target_tracking_state()
        self._request_target_cv_enable(context)
        if self._failed:
            context.logger.error(
                f"[{self.name}] target_latitude and target_longitude must be set"
            )

    def _exit_common_mission(self, context: MissionContext) -> None:
        self._reset_tracking_filter()
        context.clear_all_setpoints()
        context.clear_target_tracking_state()
        if context.target_cv_control_ready():
            context.set_target_cv_enabled(False)

    def _get_centering_descent_command(
        self,
        context: MissionContext,
        *,
        target_altitude_m: float,
        descent_rate_mps: float,
        max_centering_speed_mps: float | None = None,
        target_altitude_tolerance_m: float | None = None,
    ) -> TrackingVelocityCommand | None:
        if context.local_pose is None:
            return None

        tracking = self._get_tracking_solution(
            context,
            max_centering_speed_mps=max_centering_speed_mps,
        )
        if tracking is None:
            return None

        tracking_error_m, velocity_east, velocity_north = tracking
        altitude_error = abs(context.local_pose.pose.position.z - target_altitude_m)
        altitude_tolerance_m = self._arrival_alt_tolerance_m
        if target_altitude_tolerance_m is not None:
            altitude_tolerance_m = max(0.0, float(target_altitude_tolerance_m))
        reached_target_altitude = altitude_error <= altitude_tolerance_m

        vertical_velocity = 0.0
        if (
            tracking_error_m <= self._centering_tolerance_m
            and not reached_target_altitude
        ):
            vertical_velocity = -abs(descent_rate_mps)

        return TrackingVelocityCommand(
            tracking_error_m=tracking_error_m,
            velocity_east_mps=velocity_east,
            velocity_north_mps=velocity_north,
            vertical_velocity_mps=vertical_velocity,
            reached_target_altitude=reached_target_altitude,
        )

    def _has_recent_target_detection(self, context: MissionContext) -> bool:
        detection = context.target_detection
        if detection is None:
            return False
        if detection.point.x == -1.0 and detection.point.y == -1.0:
            return False

        detection_time = Time.from_msg(detection.header.stamp)
        return context.seconds_since(detection_time) < self._target_timeout_s

    def _get_tracking_solution(
        self,
        context: MissionContext,
        max_centering_speed_mps: float | None = None,
    ) -> tuple[float, float, float] | None:
        if not self._has_recent_target_detection(context):
            return None

        detection = context.target_detection
        if detection is None or context.local_pose is None:
            return None

        image_size = context.image_size
        if image_size is None:
            return None

        orientation = context.local_pose.pose.orientation
        projection = project_normalized_detection_to_ground_offset(
            x_norm=float(detection.point.x),
            y_norm=float(detection.point.y),
            image_width=int(image_size[0]),
            image_height=int(image_size[1]),
            intrinsics=self._camera_intrinsics,
            qx=float(orientation.x),
            qy=float(orientation.y),
            qz=float(orientation.z),
            qw=float(orientation.w),
            altitude_m=float(context.local_pose.pose.position.z),
            camera_yaw_offset_deg=self._camera_yaw_offset_deg,
            min_projection_altitude_m=self._min_projection_altitude_m,
            max_projection_distance_m=self._max_projection_distance_m,
        )
        if projection is None:
            return None

        east_error_m = projection.east_m
        north_error_m = projection.north_m
        if abs(east_error_m) <= self._centering_deadband_m:
            east_error_m = 0.0
        if abs(north_error_m) <= self._centering_deadband_m:
            north_error_m = 0.0

        alpha = max(0.0, min(self._tracking_low_pass_alpha, 1.0))
        if (
            self._filtered_tracking_east_m is None
            or self._filtered_tracking_north_m is None
        ):
            self._filtered_tracking_east_m = east_error_m
            self._filtered_tracking_north_m = north_error_m
        else:
            self._filtered_tracking_east_m = self._filtered_tracking_east_m + (
                alpha * (east_error_m - self._filtered_tracking_east_m)
            )
            self._filtered_tracking_north_m = self._filtered_tracking_north_m + (
                alpha * (north_error_m - self._filtered_tracking_north_m)
            )

        filtered_error_m = math.hypot(
            self._filtered_tracking_east_m,
            self._filtered_tracking_north_m,
        )
        max_speed = self._max_centering_speed_mps
        if max_centering_speed_mps is not None:
            max_speed = max(0.0, float(max_centering_speed_mps))

        velocity_east, velocity_north = ground_offset_to_velocity(
            east_error_m=self._filtered_tracking_east_m,
            north_error_m=self._filtered_tracking_north_m,
            gain_mps_per_m=self._centering_gain_mps_per_m,
            max_speed_mps=max_speed,
        )
        return filtered_error_m, velocity_east, velocity_north

    def _target_loss_grace_expired(self, context: MissionContext) -> bool:
        if self._target_loss_start is None:
            self._target_loss_start = context.now()
            context.logger.warn(
                f"[{self.name}] Target tracking lost; holding for "
                f"{self._target_loss_grace_s:.1f} s before recovery"
            )
            return False
        return context.seconds_since(self._target_loss_start) >= self._target_loss_grace_s

    def _clear_target_loss_grace(self) -> None:
        self._target_loss_start = None

    def _hold_tracking_loss_grace(self, context: MissionContext) -> bool:
        self._reset_tracking_filter()
        context.set_local_velocity_setpoint(
            0.0,
            0.0,
            0.0,
            yaw_deg=HOLD_YAW_DEG,
        )
        self._log_velocity_command(
            context,
            east_mps=0.0,
            north_mps=0.0,
            up_mps=0.0,
        )
        return self._target_loss_grace_expired(context)

    def _update_target_recovery(self, context: MissionContext) -> RecoveryCommandResult:
        if context.global_gps is None or context.local_pose is None:
            return RecoveryCommandResult(failed=False, reached_altitude=False)

        if self._recovery_target_altitude is None or self._recovery_hold_position is None:
            if self._recovery_attempts >= self._max_recovery_attempts:
                context.logger.error(
                    f"[{self.name}] Target recovery exceeded "
                    f"{self._max_recovery_attempts} attempts"
                )
                return RecoveryCommandResult(failed=True, reached_altitude=False)

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
                return RecoveryCommandResult(failed=True, reached_altitude=False)

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
        return RecoveryCommandResult(
            failed=False,
            reached_altitude=altitude_error <= self._recovery_alt_tolerance_m,
        )

    def _reset_tracking_filter(self) -> None:
        self._filtered_tracking_east_m = None
        self._filtered_tracking_north_m = None

    def _log_velocity_command(
        self,
        context: MissionContext,
        *,
        east_mps: float,
        north_mps: float,
        up_mps: float,
    ) -> None:
        now = context.now()
        if self._last_velocity_log_time is not None:
            elapsed_s = (now - self._last_velocity_log_time).nanoseconds / 1e9
            if elapsed_s < 1.0:
                return

        context.logger.info(
            f"[{self.name}] Velocity command: "
            f"east={east_mps:.3f} m/s, "
            f"north={north_mps:.3f} m/s, "
            f"up={up_mps:.3f} m/s"
        )
        self._last_velocity_log_time = now

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
