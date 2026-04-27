"""Pure OpenCV red target detector for solid circles and bullseyes."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class RedTargetDetectorConfig:
    """Configuration for the red target detector."""

    min_target_area_px: float = 25.0
    min_cluster_area_px: float = 0.0
    min_detection_confidence: float = 0.25
    hsv_blur_kernel_px: int = 5
    morph_kernel_px: int = 3
    mask_blur_kernel_px: int = 0
    use_light_normalization: bool = True
    clahe_clip_limit: float = 2.0
    clahe_tile_grid_size_px: int = 8
    hsv_red1_h_min: int = 0
    hsv_red1_h_max: int = 12
    hsv_red2_h_min: int = 168
    hsv_red2_h_max: int = 180
    hsv_s_min: int = 70
    hsv_s_max: int = 255
    hsv_v_min: int = 45
    hsv_v_max: int = 255
    red_dominance_ratio: float = 1.15
    red_difference_min: int = 15
    red_min_channel: int = 45
    cluster_kernel_px: int = 31
    cluster_dilate_iterations: int = 1
    radial_ray_count: int = 32
    radial_sample_count: int = 80
    bullseye_min_transitions: float = 3.0


@dataclass(frozen=True)
class TargetDetectionCandidate:
    """Scored target candidate derived from one red cluster."""

    center: tuple[int, int]
    red_area_px: float
    cluster_area_px: float
    confidence: float
    solid_score: float
    bullseye_score: float
    circularity: float
    radial_transitions: float
    center_consistency: float
    selected_red_mask: np.ndarray
    selected_cluster_mask: np.ndarray
    cluster_contour: np.ndarray | None


@dataclass(frozen=True)
class TargetDetectionResult:
    """Detector output and debug fields for one frame."""

    center: tuple[int, int] | None
    red_area_px: float
    cluster_area_px: float
    confidence: float
    solid_score: float
    bullseye_score: float
    circularity: float
    radial_transitions: float
    center_consistency: float
    raw_red_area_px: int
    clean_red_area_px: int
    cluster_count: int
    reject_reason: str
    clean_red_mask: np.ndarray
    selected_red_mask: np.ndarray
    cluster_mask: np.ndarray
    selected_cluster_mask: np.ndarray
    cluster_contour: np.ndarray | None

    @property
    def accepted(self) -> bool:
        """Return whether this frame has an accepted target detection."""

        return self.center is not None


class RedTargetDetector:
    """Detect red solid-circle and red/white bullseye targets."""

    def __init__(self, config: RedTargetDetectorConfig | None = None) -> None:
        self.config = config or RedTargetDetectorConfig()

    def detect(self, image: np.ndarray) -> TargetDetectionResult:
        """Detect the best target candidate in a BGR image."""

        bgr = self._ensure_bgr(image)
        raw_red_mask = self._build_red_mask(bgr)
        clean_red_mask = self._clean_red_mask(raw_red_mask)
        cluster_mask = self._build_cluster_mask(clean_red_mask)

        raw_red_area_px = int(cv2.countNonZero(raw_red_mask))
        clean_red_area_px = int(cv2.countNonZero(clean_red_mask))
        blank = np.zeros(clean_red_mask.shape, dtype=np.uint8)

        min_cluster_area_px = self.config.min_cluster_area_px
        if min_cluster_area_px <= 0.0:
            min_cluster_area_px = self.config.min_target_area_px

        label_count, labels, stats, _ = cv2.connectedComponentsWithStats(
            cluster_mask,
            8,
        )
        cluster_count = max(0, int(label_count) - 1)
        best_rejected: TargetDetectionCandidate | None = None
        best_candidate: TargetDetectionCandidate | None = None

        for label_index in range(1, label_count):
            cluster_area_px = float(
                stats[label_index, cv2.CC_STAT_AREA]
            )
            if cluster_area_px < min_cluster_area_px:
                continue

            cluster_component = np.zeros(clean_red_mask.shape, dtype=np.uint8)
            cluster_component[labels == label_index] = 255
            selected_red = cv2.bitwise_and(
                clean_red_mask,
                clean_red_mask,
                mask=cluster_component,
            )
            red_area_px = float(cv2.countNonZero(selected_red))
            if red_area_px < self.config.min_target_area_px:
                continue

            candidate = self._score_candidate(
                selected_red,
                cluster_component,
                red_area_px,
                cluster_area_px,
            )
            if candidate is None:
                continue

            if (
                best_rejected is None
                or candidate.confidence > best_rejected.confidence
            ):
                best_rejected = candidate

            if candidate.confidence < self.config.min_detection_confidence:
                continue

            if best_candidate is None:
                best_candidate = candidate
                continue

            best_score = self._selection_score(best_candidate)
            candidate_score = self._selection_score(candidate)
            if candidate_score > best_score:
                best_candidate = candidate

        if best_candidate is not None:
            return self._result_from_candidate(
                best_candidate,
                raw_red_area_px,
                clean_red_area_px,
                cluster_count,
                "accepted",
                clean_red_mask,
                cluster_mask,
            )

        if best_rejected is not None:
            return self._result_from_candidate(
                best_rejected,
                raw_red_area_px,
                clean_red_area_px,
                cluster_count,
                "best candidate below confidence threshold",
                clean_red_mask,
                cluster_mask,
                accepted=False,
            )

        if raw_red_area_px == 0:
            reject_reason = "no red pixels after thresholding"
        elif clean_red_area_px == 0:
            reject_reason = "red pixels removed by mask cleanup"
        elif cluster_count == 0:
            reject_reason = "no red clusters after grouping"
        else:
            reject_reason = "red clusters below minimum target area"

        return TargetDetectionResult(
            center=None,
            red_area_px=0.0,
            cluster_area_px=0.0,
            confidence=0.0,
            solid_score=0.0,
            bullseye_score=0.0,
            circularity=0.0,
            radial_transitions=0.0,
            center_consistency=0.0,
            raw_red_area_px=raw_red_area_px,
            clean_red_area_px=clean_red_area_px,
            cluster_count=cluster_count,
            reject_reason=reject_reason,
            clean_red_mask=clean_red_mask,
            selected_red_mask=blank,
            cluster_mask=cluster_mask,
            selected_cluster_mask=blank,
            cluster_contour=None,
        )

    @staticmethod
    def _ensure_bgr(image: np.ndarray) -> np.ndarray:
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.ndim == 3 and image.shape[2] == 3:
            return image
        if image.ndim == 3 and image.shape[2] == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        raise ValueError(f"Unsupported image shape: {image.shape}")

    def _build_red_mask(self, bgr: np.ndarray) -> np.ndarray:
        normalized = self._normalize_light(bgr)
        hsv = cv2.cvtColor(normalized, cv2.COLOR_BGR2HSV)
        hsv_blur_kernel_px = _odd_kernel_size(self.config.hsv_blur_kernel_px)
        if hsv_blur_kernel_px > 1:
            hsv = cv2.GaussianBlur(
                hsv,
                (hsv_blur_kernel_px, hsv_blur_kernel_px),
                0,
            )

        lower_red1 = np.array(
            [
                _clamp_int(self.config.hsv_red1_h_min, 0, 180),
                _clamp_int(self.config.hsv_s_min, 0, 255),
                _clamp_int(self.config.hsv_v_min, 0, 255),
            ],
            dtype=np.uint8,
        )
        upper_red1 = np.array(
            [
                _clamp_int(self.config.hsv_red1_h_max, 0, 180),
                _clamp_int(self.config.hsv_s_max, 0, 255),
                _clamp_int(self.config.hsv_v_max, 0, 255),
            ],
            dtype=np.uint8,
        )
        lower_red2 = np.array(
            [
                _clamp_int(self.config.hsv_red2_h_min, 0, 180),
                _clamp_int(self.config.hsv_s_min, 0, 255),
                _clamp_int(self.config.hsv_v_min, 0, 255),
            ],
            dtype=np.uint8,
        )
        upper_red2 = np.array(
            [
                _clamp_int(self.config.hsv_red2_h_max, 0, 180),
                _clamp_int(self.config.hsv_s_max, 0, 255),
                _clamp_int(self.config.hsv_v_max, 0, 255),
            ],
            dtype=np.uint8,
        )
        hsv_mask = cv2.bitwise_or(
            cv2.inRange(hsv, lower_red1, upper_red1),
            cv2.inRange(hsv, lower_red2, upper_red2),
        )

        b_channel, g_channel, r_channel = cv2.split(bgr)
        r_float = r_channel.astype(np.float32)
        g_float = g_channel.astype(np.float32)
        b_float = b_channel.astype(np.float32)
        red_floor = max(0.0, float(self.config.red_min_channel))
        red_margin = max(0.0, float(self.config.red_difference_min))
        red_ratio = max(1.0, float(self.config.red_dominance_ratio))
        dominance = (
            (r_float >= red_floor)
            & (r_float >= (g_float * red_ratio))
            & (r_float >= (b_float * red_ratio))
            & ((r_float - np.maximum(g_float, b_float)) >= red_margin)
        )
        dominance_mask = np.zeros(hsv_mask.shape, dtype=np.uint8)
        dominance_mask[dominance] = 255
        return cv2.bitwise_or(hsv_mask, dominance_mask)

    def _normalize_light(self, bgr: np.ndarray) -> np.ndarray:
        if not self.config.use_light_normalization:
            return bgr

        tile_size = max(2, int(self.config.clahe_tile_grid_size_px))
        clahe = cv2.createCLAHE(
            clipLimit=max(0.1, float(self.config.clahe_clip_limit)),
            tileGridSize=(tile_size, tile_size),
        )
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)
        l_channel = clahe.apply(l_channel)
        normalized_lab = cv2.merge((l_channel, a_channel, b_channel))
        return cv2.cvtColor(normalized_lab, cv2.COLOR_LAB2BGR)

    def _clean_red_mask(self, red_mask: np.ndarray) -> np.ndarray:
        clean = red_mask
        morph_kernel_px = _odd_kernel_size(self.config.morph_kernel_px)
        if morph_kernel_px > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (morph_kernel_px, morph_kernel_px),
            )
            clean = cv2.morphologyEx(clean, cv2.MORPH_OPEN, kernel)
            clean = cv2.morphologyEx(clean, cv2.MORPH_CLOSE, kernel)

        mask_blur_kernel_px = _odd_kernel_size(
            self.config.mask_blur_kernel_px,
            allow_disabled=True,
        )
        if mask_blur_kernel_px:
            clean = cv2.GaussianBlur(
                clean,
                (mask_blur_kernel_px, mask_blur_kernel_px),
                0,
            )
            _, clean = cv2.threshold(clean, 127, 255, cv2.THRESH_BINARY)
        return clean

    def _build_cluster_mask(self, clean_red_mask: np.ndarray) -> np.ndarray:
        cluster_kernel_px = _odd_kernel_size(self.config.cluster_kernel_px)
        if cluster_kernel_px <= 1:
            return clean_red_mask.copy()

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (cluster_kernel_px, cluster_kernel_px),
        )
        cluster = cv2.morphologyEx(clean_red_mask, cv2.MORPH_CLOSE, kernel)
        iterations = max(0, int(self.config.cluster_dilate_iterations))
        if iterations:
            cluster = cv2.dilate(cluster, kernel, iterations=iterations)
        return cluster

    def _score_candidate(
        self,
        selected_red: np.ndarray,
        cluster_component: np.ndarray,
        red_area_px: float,
        cluster_area_px: float,
    ) -> TargetDetectionCandidate | None:
        red_moments = cv2.moments(selected_red, binaryImage=True)
        if red_moments["m00"] <= 0.0:
            return None

        center_x = int(round(red_moments["m10"] / red_moments["m00"]))
        center_y = int(round(red_moments["m01"] / red_moments["m00"]))
        center = (center_x, center_y)

        cluster_contours, _ = cv2.findContours(
            cluster_component,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not cluster_contours:
            return None

        cluster_contour = max(cluster_contours, key=cv2.contourArea)
        contour_area = max(0.0, float(cv2.contourArea(cluster_contour)))
        perimeter = max(0.0, float(cv2.arcLength(cluster_contour, True)))
        circularity = 0.0
        if perimeter > 0.0:
            circularity = (
                (4.0 * math.pi * contour_area) / (perimeter * perimeter)
            )
        circularity = _clamp_float(circularity, 0.0, 1.0)

        (circle_x, circle_y), circle_radius = cv2.minEnclosingCircle(
            cluster_contour,
        )
        circle_area = math.pi * max(1.0, float(circle_radius)) ** 2
        red_fill_ratio = red_area_px / max(circle_area, 1.0)
        cluster_fill_ratio = red_area_px / max(cluster_area_px, 1.0)

        solid_fill_score = _score_range(red_fill_ratio, 0.35, 0.75)
        solid_cluster_score = _score_range(cluster_fill_ratio, 0.35, 0.70)
        solid_score = max(solid_fill_score, solid_cluster_score) * circularity

        radial_transitions = self._radial_transition_score(
            selected_red,
            center,
            max(1.0, float(circle_radius)),
        )
        bullseye_transition_score = _score_range(
            radial_transitions,
            self.config.bullseye_min_transitions,
            self.config.bullseye_min_transitions + 4.0,
        )
        bullseye_score = bullseye_transition_score * max(0.35, circularity)

        center_distance = math.hypot(center_x - circle_x, center_y - circle_y)
        center_consistency = 1.0 - min(
            1.0,
            center_distance / max(1.0, float(circle_radius)),
        )

        area_score = min(
            1.0,
            red_area_px / max(1.0, self.config.min_target_area_px * 8.0),
        )
        shape_score = max(solid_score, bullseye_score)
        confidence = (
            (0.48 * shape_score)
            + (0.22 * area_score)
            + (0.18 * circularity)
            + (0.12 * center_consistency)
        )
        confidence = _clamp_float(confidence, 0.0, 1.0)

        return TargetDetectionCandidate(
            center=center,
            red_area_px=red_area_px,
            cluster_area_px=cluster_area_px,
            confidence=confidence,
            solid_score=_clamp_float(solid_score, 0.0, 1.0),
            bullseye_score=_clamp_float(bullseye_score, 0.0, 1.0),
            circularity=circularity,
            radial_transitions=radial_transitions,
            center_consistency=center_consistency,
            selected_red_mask=selected_red,
            selected_cluster_mask=cluster_component,
            cluster_contour=cluster_contour,
        )

    def _radial_transition_score(
        self,
        mask: np.ndarray,
        center: tuple[int, int],
        radius: float,
    ) -> float:
        ray_count = max(8, int(self.config.radial_ray_count))
        sample_count = max(12, int(self.config.radial_sample_count))
        height, width = mask.shape[:2]
        center_x, center_y = center
        transitions: list[int] = []

        for ray_index in range(ray_count):
            angle = (2.0 * math.pi * ray_index) / float(ray_count)
            cos_angle = math.cos(angle)
            sin_angle = math.sin(angle)
            values: list[bool] = []
            for sample_index in range(sample_count):
                sample_radius = (
                    float(radius) * float(sample_index)
                ) / float(sample_count - 1)
                x_pixel = int(round(center_x + (cos_angle * sample_radius)))
                y_pixel = int(round(center_y + (sin_angle * sample_radius)))
                if x_pixel < 0 or x_pixel >= width:
                    break
                if y_pixel < 0 or y_pixel >= height:
                    break
                values.append(mask[y_pixel, x_pixel] > 0)

            if len(values) < 8:
                continue
            transitions.append(_count_bool_transitions(values))

        if not transitions:
            return 0.0
        return float(np.median(np.array(transitions, dtype=np.float32)))

    @staticmethod
    def _selection_score(candidate: TargetDetectionCandidate) -> float:
        area_bonus = 0.04 * math.log1p(candidate.red_area_px)
        return candidate.confidence + area_bonus

    @staticmethod
    def _result_from_candidate(
        candidate: TargetDetectionCandidate,
        raw_red_area_px: int,
        clean_red_area_px: int,
        cluster_count: int,
        reject_reason: str,
        clean_red_mask: np.ndarray,
        cluster_mask: np.ndarray,
        *,
        accepted: bool = True,
    ) -> TargetDetectionResult:
        center = candidate.center if accepted else None
        return TargetDetectionResult(
            center=center,
            red_area_px=candidate.red_area_px,
            cluster_area_px=candidate.cluster_area_px,
            confidence=candidate.confidence,
            solid_score=candidate.solid_score,
            bullseye_score=candidate.bullseye_score,
            circularity=candidate.circularity,
            radial_transitions=candidate.radial_transitions,
            center_consistency=candidate.center_consistency,
            raw_red_area_px=raw_red_area_px,
            clean_red_area_px=clean_red_area_px,
            cluster_count=cluster_count,
            reject_reason=reject_reason,
            clean_red_mask=clean_red_mask,
            selected_red_mask=candidate.selected_red_mask,
            cluster_mask=cluster_mask,
            selected_cluster_mask=candidate.selected_cluster_mask,
            cluster_contour=candidate.cluster_contour,
        )


def draw_debug_overlay(
    image: np.ndarray,
    result: TargetDetectionResult,
) -> np.ndarray:
    """Draw detection debug information on a BGR image."""

    annotated = RedTargetDetector._ensure_bgr(image).copy()
    if result.cluster_contour is not None:
        contour_color = (0, 255, 255) if result.accepted else (0, 165, 255)
        cv2.drawContours(
            annotated,
            [result.cluster_contour],
            -1,
            contour_color,
            2,
        )

    if result.accepted and result.center is not None:
        cv2.drawMarker(
            annotated,
            result.center,
            (0, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=24,
            thickness=2,
        )
        cv2.circle(annotated, result.center, 5, (0, 0, 255), -1)

    if result.accepted:
        status = "TARGET"
    else:
        status = f"NO TARGET: {result.reject_reason}"
    lines = [
        status,
        (
            f"conf={result.confidence:.2f} solid={result.solid_score:.2f} "
            f"bull={result.bullseye_score:.2f}"
        ),
        (
            f"red={result.red_area_px:.0f} "
            f"cluster={result.cluster_area_px:.0f} "
            f"circ={result.circularity:.2f} "
            f"trans={result.radial_transitions:.1f}"
        ),
    ]
    for line_index, line in enumerate(lines):
        y_pixel = 24 + (line_index * 24)
        cv2.putText(
            annotated,
            line,
            (12, y_pixel),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            line,
            (12, y_pixel),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return annotated


def _odd_kernel_size(value: int, *, allow_disabled: bool = False) -> int:
    size = int(value)
    if allow_disabled and size <= 1:
        return 0
    size = max(size, 1)
    if size % 2 == 0:
        size += 1
    return size


def _clamp_int(value: int, lower: int, upper: int) -> int:
    return max(lower, min(int(value), upper))


def _clamp_float(value: float, lower: float, upper: float) -> float:
    return max(lower, min(float(value), upper))


def _score_range(value: float, low: float, high: float) -> float:
    if high <= low:
        return 1.0 if value >= high else 0.0
    return _clamp_float((float(value) - low) / (high - low), 0.0, 1.0)


def _count_bool_transitions(values: list[bool]) -> int:
    if not values:
        return 0

    filtered_values = values
    if len(values) >= 5:
        filtered_values = []
        for index in range(len(values)):
            start = max(0, index - 1)
            stop = min(len(values), index + 2)
            true_count = sum(1 for value in values[start:stop] if value)
            filtered_values.append(true_count >= 2)

    transitions = 0
    previous = filtered_values[0]
    for value in filtered_values[1:]:
        if value != previous:
            transitions += 1
            previous = value
    return transitions
