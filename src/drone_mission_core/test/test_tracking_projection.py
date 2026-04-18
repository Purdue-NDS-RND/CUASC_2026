import math
import unittest

from drone_mission_core.tracking_projection import (
    CameraIntrinsics,
    DEFAULT_CAMERA_CALIBRATION_HEIGHT_PX,
    DEFAULT_CAMERA_CALIBRATION_WIDTH_PX,
    DEFAULT_CAMERA_FX_PX,
    DEFAULT_CAMERA_FY_PX,
    ground_offset_to_velocity,
    project_normalized_detection_to_ground_offset,
)


TEST_INTRINSICS = CameraIntrinsics(
    fx_px=DEFAULT_CAMERA_FX_PX,
    fy_px=DEFAULT_CAMERA_FY_PX,
    calibration_width_px=DEFAULT_CAMERA_CALIBRATION_WIDTH_PX,
    calibration_height_px=DEFAULT_CAMERA_CALIBRATION_HEIGHT_PX,
)


def quaternion_from_euler(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
) -> tuple[float, float, float, float]:
    half_roll = 0.5 * roll_rad
    half_pitch = 0.5 * pitch_rad
    half_yaw = 0.5 * yaw_rad

    sin_roll = math.sin(half_roll)
    cos_roll = math.cos(half_roll)
    sin_pitch = math.sin(half_pitch)
    cos_pitch = math.cos(half_pitch)
    sin_yaw = math.sin(half_yaw)
    cos_yaw = math.cos(half_yaw)

    qx = (
        (sin_roll * cos_pitch * cos_yaw)
        - (cos_roll * sin_pitch * sin_yaw)
    )
    qy = (
        (cos_roll * sin_pitch * cos_yaw)
        + (sin_roll * cos_pitch * sin_yaw)
    )
    qz = (
        (cos_roll * cos_pitch * sin_yaw)
        - (sin_roll * sin_pitch * cos_yaw)
    )
    qw = (
        (cos_roll * cos_pitch * cos_yaw)
        + (sin_roll * sin_pitch * sin_yaw)
    )
    return qx, qy, qz, qw

class TrackingProjectionTests(unittest.TestCase):
    def test_centered_detection_projects_to_zero_ground_error(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.0,
            y_norm=0.0,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=10.0,
        )

        self.assertIsNotNone(projection)
        self.assertAlmostEqual(projection.east_m, 0.0, delta=0.05)
        self.assertAlmostEqual(projection.north_m, 0.0, delta=0.05)
        self.assertAlmostEqual(projection.horizontal_error_m, 0.0, delta=0.05)

    def test_right_side_detection_commands_east_positive(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.1,
            y_norm=0.0,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=10.0,
        )

        self.assertIsNotNone(projection)
        self.assertGreater(projection.east_m, 0.0)
        self.assertAlmostEqual(projection.north_m, 0.0, delta=0.15)

    def test_top_side_detection_commands_north_positive(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.0,
            y_norm=-0.1,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=10.0,
        )

        self.assertIsNotNone(projection)
        self.assertGreater(projection.north_m, 0.0)
        self.assertAlmostEqual(projection.east_m, 0.0, delta=0.15)

    def test_positive_roll_moves_center_projection_west(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(
            math.radians(10.0),
            0.0,
            math.pi / 2.0,
        )
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.0,
            y_norm=0.0,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=10.0,
        )

        self.assertIsNotNone(projection)
        self.assertLess(projection.east_m, 0.0)

    def test_negative_pitch_moves_center_projection_north(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(
            0.0,
            math.radians(-10.0),
            math.pi / 2.0,
        )
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.0,
            y_norm=0.0,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=10.0,
        )

        self.assertIsNotNone(projection)
        self.assertGreater(projection.north_m, 0.0)

    def test_yaw_rotation_changes_world_direction(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.1,
            y_norm=0.0,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=10.0,
        )

        self.assertIsNotNone(projection)
        self.assertLess(projection.north_m, 0.0)
        self.assertAlmostEqual(projection.east_m, 0.0, delta=0.15)

    def test_projection_is_resolution_invariant_after_intrinsic_scaling(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        full_resolution_projection = project_normalized_detection_to_ground_offset(
            x_norm=0.08,
            y_norm=-0.06,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=12.0,
        )
        half_resolution_projection = project_normalized_detection_to_ground_offset(
            x_norm=0.08,
            y_norm=-0.06,
            image_width=1920,
            image_height=1080,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=12.0,
        )

        self.assertIsNotNone(full_resolution_projection)
        self.assertIsNotNone(half_resolution_projection)
        self.assertAlmostEqual(
            half_resolution_projection.east_m,
            full_resolution_projection.east_m,
            delta=0.05,
        )
        self.assertAlmostEqual(
            half_resolution_projection.north_m,
            full_resolution_projection.north_m,
            delta=0.05,
        )

    def test_projection_returns_none_below_minimum_altitude(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.1,
            y_norm=0.0,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=0.2,
            min_projection_altitude_m=0.5,
        )

        self.assertIsNone(projection)

    def test_projection_returns_none_when_camera_ray_is_near_horizon(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(
            0.0,
            math.radians(-90.0),
            math.pi / 2.0,
        )
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.0,
            y_norm=0.0,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=10.0,
        )

        self.assertIsNone(projection)

    def test_projection_returns_none_when_horizontal_distance_is_too_large(self) -> None:
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, math.pi / 2.0)
        projection = project_normalized_detection_to_ground_offset(
            x_norm=0.8,
            y_norm=0.0,
            image_width=3840,
            image_height=2160,
            intrinsics=TEST_INTRINSICS,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
            altitude_m=20.0,
            max_projection_distance_m=1.0,
        )

        self.assertIsNone(projection)

    def test_ground_offset_velocity_clamps_to_max_speed(self) -> None:
        velocity_east_mps, velocity_north_mps = ground_offset_to_velocity(
            east_error_m=10.0,
            north_error_m=0.0,
            gain_mps_per_m=1.0,
            max_speed_mps=2.0,
        )

        self.assertAlmostEqual(velocity_east_mps, 2.0, delta=1.0e-6)
        self.assertAlmostEqual(velocity_north_mps, 0.0, delta=1.0e-6)


if __name__ == "__main__":
    unittest.main()
