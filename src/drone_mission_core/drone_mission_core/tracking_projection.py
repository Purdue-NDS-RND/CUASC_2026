"""Helpers for projecting downward-camera image detections onto the ground."""

from __future__ import annotations

import math
from dataclasses import dataclass


DEFAULT_CAMERA_FX_PX = 2334.99647
DEFAULT_CAMERA_FY_PX = 2338.06625
DEFAULT_CAMERA_CALIBRATION_WIDTH_PX = 3840.0
DEFAULT_CAMERA_CALIBRATION_HEIGHT_PX = 2160.0


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics for a calibrated camera image size."""

    fx_px: float
    fy_px: float
    calibration_width_px: float
    calibration_height_px: float


@dataclass(frozen=True)
class GroundProjection:
    """Projected target location on the horizontal ground plane."""

    east_m: float
    north_m: float
    horizontal_error_m: float


def normalized_detection_to_pixel(
    x_norm: float,
    y_norm: float,
    image_width: int,
    image_height: int,
) -> tuple[float, float]:
    """Convert centered normalized image coordinates back to pixel coordinates."""

    if image_width <= 0:
        raise ValueError("image_width must be positive")
    if image_height <= 0:
        raise ValueError("image_height must be positive")

    pixel_u = 0.5 * (float(x_norm) + 1.0) * float(image_width)
    pixel_v = 0.5 * (float(y_norm) + 1.0) * float(image_height)
    return pixel_u, pixel_v


def normalized_detection_to_pixel_error(
    x_norm: float,
    y_norm: float,
    image_width: int,
    image_height: int,
) -> tuple[float, float]:
    """Convert centered normalized image coordinates into centered pixel error."""

    if image_width <= 0:
        raise ValueError("image_width must be positive")
    if image_height <= 0:
        raise ValueError("image_height must be positive")

    pixel_error_u = 0.5 * float(x_norm) * float(image_width)
    pixel_error_v = 0.5 * float(y_norm) * float(image_height)
    return pixel_error_u, pixel_error_v


def scale_intrinsics(
    intrinsics: CameraIntrinsics,
    image_width: int,
    image_height: int,
) -> CameraIntrinsics:
    """Scale calibrated intrinsics to the current image resolution."""

    if intrinsics.calibration_width_px <= 0.0:
        raise ValueError("calibration_width_px must be positive")
    if intrinsics.calibration_height_px <= 0.0:
        raise ValueError("calibration_height_px must be positive")
    if image_width <= 0:
        raise ValueError("image_width must be positive")
    if image_height <= 0:
        raise ValueError("image_height must be positive")

    scale_x = float(image_width) / float(intrinsics.calibration_width_px)
    scale_y = float(image_height) / float(intrinsics.calibration_height_px)

    return CameraIntrinsics(
        fx_px=float(intrinsics.fx_px) * scale_x,
        fy_px=float(intrinsics.fy_px) * scale_y,
        calibration_width_px=float(image_width),
        calibration_height_px=float(image_height),
    )


def pixel_to_camera_ray(
    pixel_error_u: float,
    pixel_error_v: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float, float]:
    """Create a pinhole camera ray from center-relative pixel error."""

    if intrinsics.fx_px == 0.0:
        raise ValueError("fx_px must be non-zero")
    if intrinsics.fy_px == 0.0:
        raise ValueError("fy_px must be non-zero")

    x_cam = float(pixel_error_u) / float(intrinsics.fx_px)
    y_cam = float(pixel_error_v) / float(intrinsics.fy_px)
    z_cam = 1.0
    return x_cam, y_cam, z_cam


def rotate_camera_ray_for_yaw_offset(
    ray_cam: tuple[float, float, float],
    camera_yaw_offset_deg: float,
) -> tuple[float, float, float]:
    """Undo fixed camera rotation about the optical axis.

    Positive yaw offset rotates the image clockwise.
    """

    yaw_offset_rad = math.radians(float(camera_yaw_offset_deg))
    cos_yaw = math.cos(yaw_offset_rad)
    sin_yaw = math.sin(yaw_offset_rad)

    x_cam = ray_cam[0]
    y_cam = ray_cam[1]
    z_cam = ray_cam[2]

    rotated_x_cam = (cos_yaw * x_cam) - (sin_yaw * y_cam)
    rotated_y_cam = (sin_yaw * x_cam) + (cos_yaw * y_cam)
    rotated_z_cam = z_cam
    return rotated_x_cam, rotated_y_cam, rotated_z_cam


def camera_ray_to_body_flu(
    ray_cam: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Map a downward camera ray into ROS body FLU coordinates.

    Zero camera yaw offset assumes:
    - image top points forward
    - image right points right
    - optical axis points down
    """

    body_x = -float(ray_cam[1])
    body_y = -float(ray_cam[0])
    body_z = -float(ray_cam[2])
    return body_x, body_y, body_z


def quaternion_to_rotation_matrix(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Build a body-to-world rotation matrix from a quaternion."""

    norm = math.sqrt(
        (float(qx) * float(qx))
        + (float(qy) * float(qy))
        + (float(qz) * float(qz))
        + (float(qw) * float(qw))
    )
    if norm <= 1.0e-9:
        raise ValueError("quaternion norm must be positive")

    qx_n = float(qx) / norm
    qy_n = float(qy) / norm
    qz_n = float(qz) / norm
    qw_n = float(qw) / norm

    xx = qx_n * qx_n
    yy = qy_n * qy_n
    zz = qz_n * qz_n
    xy = qx_n * qy_n
    xz = qx_n * qz_n
    yz = qy_n * qz_n
    wx = qw_n * qx_n
    wy = qw_n * qy_n
    wz = qw_n * qz_n

    row_0 = (
        1.0 - (2.0 * (yy + zz)),
        2.0 * (xy - wz),
        2.0 * (xz + wy),
    )
    row_1 = (
        2.0 * (xy + wz),
        1.0 - (2.0 * (xx + zz)),
        2.0 * (yz - wx),
    )
    row_2 = (
        2.0 * (xz - wy),
        2.0 * (yz + wx),
        1.0 - (2.0 * (xx + yy)),
    )
    return row_0, row_1, row_2


def rotate_vector(
    rotation_matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a vector using a 3x3 matrix."""

    rotated_x = (
        (rotation_matrix[0][0] * vector[0])
        + (rotation_matrix[0][1] * vector[1])
        + (rotation_matrix[0][2] * vector[2])
    )
    rotated_y = (
        (rotation_matrix[1][0] * vector[0])
        + (rotation_matrix[1][1] * vector[1])
        + (rotation_matrix[1][2] * vector[2])
    )
    rotated_z = (
        (rotation_matrix[2][0] * vector[0])
        + (rotation_matrix[2][1] * vector[1])
        + (rotation_matrix[2][2] * vector[2])
    )
    return rotated_x, rotated_y, rotated_z


def project_normalized_detection_to_ground_offset(
    x_norm: float,
    y_norm: float,
    image_width: int,
    image_height: int,
    intrinsics: CameraIntrinsics,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    altitude_m: float,
    camera_yaw_offset_deg: float = 0.0,
    min_projection_altitude_m: float = 0.5,
    max_projection_distance_m: float = 50.0,
) -> GroundProjection | None:
    """Project a normalized image detection onto a flat ground plane."""

    if float(altitude_m) < float(min_projection_altitude_m):
        return None

    try:
        pixel_error_u, pixel_error_v = normalized_detection_to_pixel_error(
            x_norm,
            y_norm,
            image_width,
            image_height,
        )
        scaled_intrinsics = scale_intrinsics(intrinsics, image_width, image_height)
        ray_cam = pixel_to_camera_ray(
            pixel_error_u,
            pixel_error_v,
            scaled_intrinsics,
        )
        ray_cam = rotate_camera_ray_for_yaw_offset(ray_cam, camera_yaw_offset_deg)
        ray_body = camera_ray_to_body_flu(ray_cam)
        rotation_matrix = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    except ValueError:
        return None

    ray_world = rotate_vector(rotation_matrix, ray_body)
    downward_component = float(ray_world[2])
    if downward_component >= -1.0e-6:
        return None

    scale_to_ground = float(altitude_m) / (-downward_component)
    east_m = float(ray_world[0]) * scale_to_ground
    north_m = float(ray_world[1]) * scale_to_ground
    horizontal_error_m = math.hypot(east_m, north_m)

    if horizontal_error_m > float(max_projection_distance_m):
        return None

    return GroundProjection(
        east_m=east_m,
        north_m=north_m,
        horizontal_error_m=horizontal_error_m,
    )


def ground_offset_to_velocity(
    east_error_m: float,
    north_error_m: float,
    gain_mps_per_m: float,
    max_speed_mps: float,
) -> tuple[float, float]:
    """Convert ground offset into a bounded world-frame XY velocity command."""

    velocity_east_mps = float(east_error_m) * float(gain_mps_per_m)
    velocity_north_mps = float(north_error_m) * float(gain_mps_per_m)

    if float(max_speed_mps) <= 0.0:
        return 0.0, 0.0

    speed_mps = math.hypot(velocity_east_mps, velocity_north_mps)
    if speed_mps > float(max_speed_mps):
        if speed_mps > 0.0:
            scale = float(max_speed_mps) / speed_mps
            velocity_east_mps = velocity_east_mps * scale
            velocity_north_mps = velocity_north_mps * scale

    return velocity_east_mps, velocity_north_mps
