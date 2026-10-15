#!/usr/bin/env python3
"""Astronaut torso depth estimation and short-term tracking."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from zed_depth import DepthFrame


LEFT_SHOULDER = 11
RIGHT_SHOULDER = 12
LEFT_HIP = 23
RIGHT_HIP = 24
TORSO_LANDMARKS = (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)


@dataclass(frozen=True)
class DepthEstimate:
    position_m: tuple[float, float, float]
    distance_m: float
    direction: str
    horizontal_angle_deg: float
    sample_count: int


@dataclass(frozen=True)
class TrackingResult:
    astronaut_detected: bool = False
    position_m: tuple[float, float, float] | None = None
    distance_m: float | None = None
    direction: str = "UNKNOWN"
    horizontal_angle_deg: float | None = None
    depth_status: str = "NO_POSE"
    sample_count: int = 0
    stale_age_s: float = 0.0
    simulated: bool = False


def map_video_point_to_depth(
    x: float,
    y: float,
    video_size: tuple[int, int],
    depth_size: tuple[int, int],
) -> tuple[int, int]:
    """Map a video pixel into an aligned depth image of a possibly different size."""
    video_width, video_height = video_size
    depth_width, depth_height = depth_size
    if min(video_width, video_height, depth_width, depth_height) <= 0:
        raise ValueError("video and depth dimensions must be positive")
    x = max(0.0, min(float(video_width - 1), float(x)))
    y = max(0.0, min(float(video_height - 1), float(y)))
    depth_x = int(round(x * (depth_width - 1) / max(video_width - 1, 1)))
    depth_y = int(round(y * (depth_height - 1) / max(video_height - 1, 1)))
    return depth_x, depth_y


def horizontal_direction(
    x_m: float,
    z_m: float,
    center_tolerance_deg: float = 10.0,
) -> tuple[str, float]:
    """Return camera-relative LEFT/CENTER/RIGHT and horizontal bearing."""
    angle = math.degrees(math.atan2(float(x_m), float(z_m)))
    if angle < -center_tolerance_deg:
        return "LEFT", angle
    if angle > center_tolerance_deg:
        return "RIGHT", angle
    return "CENTER", angle


def _visible_torso(landmarks: Sequence, minimum_visibility: float) -> bool:
    if landmarks is None or len(landmarks) <= max(TORSO_LANDMARKS):
        return False
    return all(
        float(getattr(landmarks[index], "visibility", 0.0)) >= minimum_visibility
        and math.isfinite(float(landmarks[index].x))
        and math.isfinite(float(landmarks[index].y))
        for index in TORSO_LANDMARKS
    )


def torso_sample_pixels(
    landmarks: Sequence,
    video_size: tuple[int, int],
    depth_size: tuple[int, int],
    rows: int = 5,
    columns: int = 7,
) -> list[tuple[int, int]]:
    """Sample an inset bilinear grid inside the shoulder/hip quadrilateral."""
    if rows < 2 or columns < 2:
        raise ValueError("torso sample grid must be at least 2 by 2")
    video_width, video_height = video_size
    ls = np.array((landmarks[LEFT_SHOULDER].x, landmarks[LEFT_SHOULDER].y))
    rs = np.array((landmarks[RIGHT_SHOULDER].x, landmarks[RIGHT_SHOULDER].y))
    lh = np.array((landmarks[LEFT_HIP].x, landmarks[LEFT_HIP].y))
    rh = np.array((landmarks[RIGHT_HIP].x, landmarks[RIGHT_HIP].y))

    pixels = []
    for vertical in np.linspace(0.18, 0.82, rows):
        left_edge = (1.0 - vertical) * ls + vertical * lh
        right_edge = (1.0 - vertical) * rs + vertical * rh
        for horizontal in np.linspace(0.15, 0.85, columns):
            normalized = (1.0 - horizontal) * left_edge + horizontal * right_edge
            video_x = float(normalized[0]) * (video_width - 1)
            video_y = float(normalized[1]) * (video_height - 1)
            pixels.append(
                map_video_point_to_depth(
                    video_x,
                    video_y,
                    video_size,
                    depth_size,
                )
            )
    return list(dict.fromkeys(pixels))


def estimate_torso_depth(
    landmarks: Sequence,
    depth_frame: DepthFrame,
    video_size: tuple[int, int],
    minimum_visibility: float = 0.45,
    minimum_distance_m: float = 0.20,
    maximum_distance_m: float = 20.0,
    maximum_confidence: float = 80.0,
    minimum_samples: int = 8,
) -> DepthEstimate | None:
    """Robust median 3D torso position from aligned ZED XYZ samples."""
    xyz = depth_frame.xyz_m
    if xyz is None or xyz.ndim != 3 or xyz.shape[2] < 3:
        return None
    if not _visible_torso(landmarks, minimum_visibility):
        return None

    depth_height, depth_width = xyz.shape[:2]
    confidence = depth_frame.confidence
    use_confidence = (
        confidence is not None and confidence.shape[:2] == (depth_height, depth_width)
    )
    samples = []
    for x, y in torso_sample_pixels(
        landmarks,
        video_size,
        (depth_width, depth_height),
    ):
        point = np.asarray(xyz[y, x, :3], dtype=np.float64)
        if not np.isfinite(point).all():
            continue
        if not minimum_distance_m <= point[2] <= maximum_distance_m:
            continue
        euclidean = float(np.linalg.norm(point))
        if not minimum_distance_m <= euclidean <= maximum_distance_m * 1.10:
            continue
        if use_confidence:
            value = float(confidence[y, x])
            if not math.isfinite(value) or value > maximum_confidence:
                continue
        samples.append(point)

    if len(samples) < minimum_samples:
        return None
    values = np.asarray(samples)
    median_z = float(np.median(values[:, 2]))
    deviations = np.abs(values[:, 2] - median_z)
    mad = float(np.median(deviations))
    tolerance = max(0.20, 3.0 * 1.4826 * mad)
    filtered = values[deviations <= tolerance]
    if len(filtered) < minimum_samples:
        return None

    median_position = np.median(filtered, axis=0)
    x_m, y_m, z_m = (float(value) for value in median_position)
    distance_m = math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)
    direction, angle = horizontal_direction(x_m, z_m)
    return DepthEstimate(
        position_m=(x_m, y_m, z_m),
        distance_m=distance_m,
        direction=direction,
        horizontal_angle_deg=angle,
        sample_count=len(filtered),
    )


class AstronautTracker:
    """Smooth valid depth and bridge brief pose/depth dropouts as stale data."""

    def __init__(
        self,
        minimum_distance_m: float = 0.20,
        maximum_distance_m: float = 20.0,
        maximum_confidence: float = 80.0,
        smoothing_alpha: float = 0.35,
        stale_after_s: float = 1.0,
        pose_grace_s: float = 0.6,
    ):
        if not 0.0 < smoothing_alpha <= 1.0:
            raise ValueError("smoothing alpha must be in (0, 1]")
        if minimum_distance_m <= 0.0 or maximum_distance_m <= minimum_distance_m:
            raise ValueError("depth range is invalid")
        self.minimum_distance_m = minimum_distance_m
        self.maximum_distance_m = maximum_distance_m
        self.maximum_confidence = maximum_confidence
        self.smoothing_alpha = smoothing_alpha
        self.stale_after_s = stale_after_s
        self.pose_grace_s = pose_grace_s
        self._position = None
        self._last_valid_time = None
        self._last_pose_time = None
        self._last_simulated = False

    def update(
        self,
        pose_landmarks,
        depth_frame: DepthFrame | None,
        video_size: tuple[int, int],
        now: float | None = None,
    ) -> TrackingResult:
        now = time.monotonic() if now is None else float(now)
        landmarks = None if pose_landmarks is None else pose_landmarks.landmark
        pose_visible = _visible_torso(landmarks, 0.45)
        if pose_visible:
            self._last_pose_time = now
        else:
            return self._missing_pose(now)

        if depth_frame is None or depth_frame.xyz_m is None:
            return self._unavailable_or_stale(now, astronaut_detected=True)

        estimate = estimate_torso_depth(
            landmarks,
            depth_frame,
            video_size,
            minimum_distance_m=self.minimum_distance_m,
            maximum_distance_m=self.maximum_distance_m,
            maximum_confidence=self.maximum_confidence,
        )
        if estimate is None:
            return self._unavailable_or_stale(now, astronaut_detected=True)

        measured = np.asarray(estimate.position_m, dtype=np.float64)
        if self._position is None:
            self._position = measured
        else:
            alpha = self.smoothing_alpha
            self._position = alpha * measured + (1.0 - alpha) * self._position
        self._last_valid_time = now
        self._last_simulated = depth_frame.simulated
        return self._result(
            astronaut_detected=True,
            status="SIMULATED" if depth_frame.simulated else "VALID",
            sample_count=estimate.sample_count,
            stale_age_s=0.0,
        )

    def _missing_pose(self, now: float) -> TrackingResult:
        if (
            self._last_pose_time is not None
            and now - self._last_pose_time <= self.pose_grace_s
            and self._position is not None
        ):
            return self._result(
                astronaut_detected=True,
                status="STALE",
                stale_age_s=self._valid_age(now),
            )
        return TrackingResult(depth_status="NO_POSE")

    def _unavailable_or_stale(
        self,
        now: float,
        astronaut_detected: bool,
    ) -> TrackingResult:
        age = self._valid_age(now)
        if self._position is not None and age <= self.stale_after_s:
            return self._result(
                astronaut_detected=astronaut_detected,
                status="STALE",
                stale_age_s=age,
            )
        return TrackingResult(
            astronaut_detected=astronaut_detected,
            depth_status="UNAVAILABLE",
        )

    def _valid_age(self, now: float) -> float:
        if self._last_valid_time is None:
            return math.inf
        return max(0.0, now - self._last_valid_time)

    def _result(
        self,
        astronaut_detected: bool,
        status: str,
        sample_count: int = 0,
        stale_age_s: float = 0.0,
    ) -> TrackingResult:
        x_m, y_m, z_m = (float(value) for value in self._position)
        distance_m = math.sqrt(x_m * x_m + y_m * y_m + z_m * z_m)
        direction, angle = horizontal_direction(x_m, z_m)
        return TrackingResult(
            astronaut_detected=astronaut_detected,
            position_m=(x_m, y_m, z_m),
            distance_m=distance_m,
            direction=direction,
            horizontal_angle_deg=angle,
            depth_status=status,
            sample_count=sample_count,
            stale_age_s=stale_age_s,
            simulated=self._last_simulated,
        )
