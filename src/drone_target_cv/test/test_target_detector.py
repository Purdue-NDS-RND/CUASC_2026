"""Synthetic image tests for the pure OpenCV target detector."""

from __future__ import annotations

from pathlib import Path
import sys

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from drone_target_cv.target_detector import (  # noqa: E402
    RedTargetDetector,
    RedTargetDetectorConfig,
)


ASPHALT = (82, 82, 82)
RED = (28, 30, 210)
SHADOW_RED = (50, 55, 145)
WHITE = (238, 238, 238)


def _blank_frame() -> np.ndarray:
    return np.full((480, 640, 3), ASPHALT, dtype=np.uint8)


def _draw_solid_circle(
    frame: np.ndarray,
    center: tuple[int, int],
    radius: int,
    color: tuple[int, int, int] = RED,
) -> None:
    cv2.circle(frame, center, radius, color, -1, lineType=cv2.LINE_AA)


def _draw_bullseye(
    frame: np.ndarray,
    center: tuple[int, int],
    outer_radius: int,
) -> None:
    cv2.circle(
        frame,
        center,
        outer_radius + 18,
        WHITE,
        -1,
        lineType=cv2.LINE_AA,
    )
    rings = [
        (outer_radius, RED),
        (int(outer_radius * 0.84), WHITE),
        (int(outer_radius * 0.70), RED),
        (int(outer_radius * 0.56), WHITE),
        (int(outer_radius * 0.42), RED),
        (int(outer_radius * 0.28), WHITE),
        (int(outer_radius * 0.16), RED),
        (int(outer_radius * 0.07), WHITE),
        (max(5, int(outer_radius * 0.035)), RED),
    ]
    for radius, color in rings:
        cv2.circle(frame, center, radius, color, -1, lineType=cv2.LINE_AA)


def _assert_center_near(
    actual: tuple[int, int] | None,
    expected: tuple[float, float],
    tolerance_px: float,
) -> None:
    assert actual is not None
    dx = float(actual[0]) - float(expected[0])
    dy = float(actual[1]) - float(expected[1])
    assert np.hypot(dx, dy) <= tolerance_px


def test_detects_centered_solid_practice_circle() -> None:
    frame = _blank_frame()
    expected_center = (380, 210)
    _draw_solid_circle(frame, expected_center, 74)

    result = RedTargetDetector().detect(frame)

    assert result.accepted
    _assert_center_near(result.center, expected_center, 2.0)
    assert result.solid_score > result.bullseye_score


def test_detects_shadowed_washed_red_circle() -> None:
    frame = _blank_frame()
    expected_center = (230, 310)
    _draw_solid_circle(frame, expected_center, 64, SHADOW_RED)

    result = RedTargetDetector().detect(frame)

    assert result.accepted
    _assert_center_near(result.center, expected_center, 3.0)
    assert result.confidence >= 0.25


def test_detects_off_center_official_style_bullseye() -> None:
    frame = _blank_frame()
    expected_center = (250, 190)
    _draw_bullseye(frame, expected_center, 130)

    result = RedTargetDetector().detect(frame)

    assert result.accepted
    _assert_center_near(result.center, expected_center, 3.0)
    assert result.bullseye_score > 0.2
    assert result.radial_transitions >= 3.0


def test_detects_perspective_warped_bullseye() -> None:
    frame = _blank_frame()
    original_center = np.array([[[320.0, 240.0]]], dtype=np.float32)
    _draw_bullseye(frame, (320, 240), 130)
    source = np.array(
        [[0.0, 0.0], [639.0, 0.0], [639.0, 479.0], [0.0, 479.0]],
        dtype=np.float32,
    )
    destination = np.array(
        [[80.0, 60.0], [570.0, 45.0], [615.0, 430.0], [35.0, 455.0]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(
        frame,
        matrix,
        (640, 480),
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=ASPHALT,
    )
    expected_center = cv2.perspectiveTransform(original_center, matrix)[0][0]

    result = RedTargetDetector().detect(warped)

    assert result.accepted
    _assert_center_near(result.center, expected_center, 8.0)


def test_ignores_small_red_distractor_when_target_is_present() -> None:
    frame = _blank_frame()
    expected_center = (410, 250)
    _draw_solid_circle(frame, expected_center, 82)
    cv2.rectangle(frame, (30, 30), (95, 80), RED, -1)

    result = RedTargetDetector().detect(frame)

    assert result.accepted
    _assert_center_near(result.center, expected_center, 3.0)


def test_rejects_low_circularity_red_blob() -> None:
    frame = _blank_frame()
    cv2.rectangle(frame, (80, 228), (560, 252), RED, -1)
    detector = RedTargetDetector(RedTargetDetectorConfig(min_circularity=0.4))

    result = detector.detect(frame)

    assert not result.accepted
    assert result.circularity < 0.4
    assert "circularity" in result.reject_reason
