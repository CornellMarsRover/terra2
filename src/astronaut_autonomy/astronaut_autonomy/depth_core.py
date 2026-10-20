"""Pure torso-depth estimation for organized ZED point clouds."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_HIP = 11
RIGHT_HIP = 12
TORSO_KEYPOINTS = (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)
DISTANCE_COMMANDS = frozenset(("FOLLOW", "FETCH"))


@dataclass(frozen=True)
class TargetEstimate:
    """Astronaut location in the rover ZED frame (forward, left, up)."""

    position_m: tuple[float, float, float]
    distance_m: float
    direction: str
    horizontal_angle_deg: float
    sample_count: int


@dataclass(frozen=True)
class TrackedTarget:
    astronaut_detected: bool
    estimate: TargetEstimate | None
    depth_status: str
    stale_age_s: float = 0.0


def command_requires_distance(command: str) -> bool:
    return command.upper() in DISTANCE_COMMANDS


def safe_command_for(gesture: str, distance_valid: bool) -> str:
    """Withhold motion-relevant commands until current metric depth is valid."""
    normalized = gesture.upper()
    if normalized == "NONE":
        return "NONE"
    if command_requires_distance(normalized) and not distance_valid:
        return "NONE"
    return normalized


def organized_xyz_from_buffer(
    data: bytes | bytearray | memoryview,
    height: int,
    width: int,
    point_step: int,
    row_step: int,
    x_offset: int,
    y_offset: int,
    z_offset: int,
    is_bigendian: bool = False,
) -> np.ndarray:
    """Decode XYZ from an organized PointCloud2-compatible byte buffer.

    The explicit strides preserve row padding and fields beyond XYZ. A copy is
    returned so the result is safe after the ROS message leaves its callback.
    """
    if min(height, width, point_step, row_step) <= 0:
        raise ValueError("point-cloud dimensions and strides must be positive")
    maximum_offset = max(x_offset, y_offset, z_offset)
    if min(x_offset, y_offset, z_offset) < 0:
        raise ValueError("field offsets cannot be negative")
    if maximum_offset + 4 > point_step:
        raise ValueError("XYZ fields do not fit within point_step")
    if row_step < width * point_step:
        raise ValueError("row_step is too small for the declared row")
    if len(data) < height * row_step:
        raise ValueError("point-cloud data buffer is truncated")

    byte_order = ">" if is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": ("x", "y", "z"),
            "formats": (
                byte_order + "f4",
                byte_order + "f4",
                byte_order + "f4",
            ),
            "offsets": (x_offset, y_offset, z_offset),
            "itemsize": point_step,
        }
    )
    structured = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=data,
        strides=(row_step, point_step),
    )
    return np.stack(
        (structured["x"], structured["y"], structured["z"]),
        axis=-1,
    ).astype(np.float32, copy=True)


def horizontal_direction(
    forward_m: float,
    left_m: float,
    center_tolerance_deg: float = 10.0,
) -> tuple[str, float]:
    """Return target bearing in the ZED right-handed Z-up camera frame."""
    angle = math.degrees(math.atan2(float(left_m), float(forward_m)))
    if angle > center_tolerance_deg:
        return "LEFT", angle
    if angle < -center_tolerance_deg:
        return "RIGHT", angle
    return "CENTER", angle


def map_image_point_to_depth(
    x_px: float,
    y_px: float,
    image_size: tuple[int, int],
    depth_size: tuple[int, int],
) -> tuple[int, int]:
    image_width, image_height = image_size
    depth_width, depth_height = depth_size
    if min(image_width, image_height, depth_width, depth_height) <= 0:
        raise ValueError("image and depth dimensions must be positive")
    x_px = max(0.0, min(float(image_width - 1), float(x_px)))
    y_px = max(0.0, min(float(image_height - 1), float(y_px)))
    depth_x = int(round(x_px * (depth_width - 1) / max(image_width - 1, 1)))
    depth_y = int(round(y_px * (depth_height - 1) / max(image_height - 1, 1)))
    return depth_x, depth_y


def _visible_torso(keypoints: Sequence, minimum_confidence: float) -> bool:
    if keypoints is None or len(keypoints) <= max(TORSO_KEYPOINTS):
        return False
    for index in TORSO_KEYPOINTS:
        point = keypoints[index]
        if len(point) < 3:
            return False
        if float(point[2]) < minimum_confidence:
            return False
        if not math.isfinite(float(point[0])) or not math.isfinite(float(point[1])):
            return False
    return True


def torso_sample_pixels(
    keypoints: Sequence,
    image_size: tuple[int, int],
    depth_size: tuple[int, int],
    rows: int = 5,
    columns: int = 7,
) -> list[tuple[int, int]]:
    """Return an inset sampling grid in the shoulder/hip quadrilateral."""
    if rows < 2 or columns < 2:
        raise ValueError("torso sample grid must be at least 2 by 2")
    left_shoulder = np.asarray(keypoints[LEFT_SHOULDER][:2], dtype=np.float64)
    right_shoulder = np.asarray(keypoints[RIGHT_SHOULDER][:2], dtype=np.float64)
    left_hip = np.asarray(keypoints[LEFT_HIP][:2], dtype=np.float64)
    right_hip = np.asarray(keypoints[RIGHT_HIP][:2], dtype=np.float64)
    samples = []
    for vertical in np.linspace(0.18, 0.82, rows):
        left_edge = (1.0 - vertical) * left_shoulder + vertical * left_hip
        right_edge = (1.0 - vertical) * right_shoulder + vertical * right_hip
        for horizontal in np.linspace(0.15, 0.85, columns):
            point = (1.0 - horizontal) * left_edge + horizontal * right_edge
            samples.append(
                map_image_point_to_depth(
                    float(point[0]),
                    float(point[1]),
                    image_size,
                    depth_size,
                )
            )
    return list(dict.fromkeys(samples))


def estimate_target_from_xyz(
    keypoints: Sequence,
    xyz_forward_left_up_m: np.ndarray,
    image_size: tuple[int, int],
    minimum_keypoint_confidence: float = 0.35,
    minimum_distance_m: float = 0.20,
    maximum_distance_m: float = 20.0,
    minimum_samples: int = 8,
) -> TargetEstimate | None:
    """Estimate astronaut position from robust median torso XYZ samples."""
    xyz = np.asarray(xyz_forward_left_up_m)
    if xyz.ndim != 3 or xyz.shape[2] < 3:
        return None
    if not _visible_torso(keypoints, minimum_keypoint_confidence):
        return None

    depth_height, depth_width = xyz.shape[:2]
    samples = []
    for x, y in torso_sample_pixels(
        keypoints,
        image_size,
        (depth_width, depth_height),
    ):
        point = np.asarray(xyz[y, x, :3], dtype=np.float64)
        if not np.isfinite(point).all():
            continue
        forward = float(point[0])
        distance = float(np.linalg.norm(point))
        if not minimum_distance_m <= forward <= maximum_distance_m:
            continue
        if not minimum_distance_m <= distance <= maximum_distance_m * 1.10:
            continue
        samples.append(point)

    if len(samples) < minimum_samples:
        return None
    values = np.asarray(samples)
    median_forward = float(np.median(values[:, 0]))
    deviations = np.abs(values[:, 0] - median_forward)
    mad = float(np.median(deviations))
    tolerance = max(0.20, 3.0 * 1.4826 * mad)
    filtered = values[deviations <= tolerance]
    if len(filtered) < minimum_samples:
        return None

    forward_m, left_m, up_m = (
        float(value) for value in np.median(filtered, axis=0)
    )
    distance_m = math.sqrt(
        forward_m * forward_m + left_m * left_m + up_m * up_m
    )
    direction, angle = horizontal_direction(forward_m, left_m)
    return TargetEstimate(
        position_m=(forward_m, left_m, up_m),
        distance_m=distance_m,
        direction=direction,
        horizontal_angle_deg=angle,
        sample_count=len(filtered),
    )


class TargetTracker:
    """Smooth current depth and retain it briefly as explicitly stale data."""

    def __init__(self, smoothing_alpha: float = 0.35, stale_after_s: float = 0.6):
        if not 0.0 < smoothing_alpha <= 1.0:
            raise ValueError("smoothing alpha must be in (0, 1]")
        if stale_after_s < 0.0:
            raise ValueError("stale duration cannot be negative")
        self.smoothing_alpha = float(smoothing_alpha)
        self.stale_after_s = float(stale_after_s)
        self._position = None
        self._last_valid_time = None

    def update(
        self,
        astronaut_detected: bool,
        estimate: TargetEstimate | None,
        now: float,
    ) -> TrackedTarget:
        if estimate is not None:
            measured = np.asarray(estimate.position_m, dtype=np.float64)
            if self._position is None:
                self._position = measured
            else:
                alpha = self.smoothing_alpha
                self._position = alpha * measured + (1.0 - alpha) * self._position
            self._last_valid_time = float(now)
            return TrackedTarget(
                astronaut_detected=True,
                estimate=self._estimate_from_position(estimate.sample_count),
                depth_status="VALID",
            )

        if self._position is not None and self._last_valid_time is not None:
            age = max(0.0, float(now) - self._last_valid_time)
            if age <= self.stale_after_s:
                return TrackedTarget(
                    astronaut_detected=astronaut_detected,
                    estimate=self._estimate_from_position(0),
                    depth_status="STALE",
                    stale_age_s=age,
                )
        return TrackedTarget(
            astronaut_detected=astronaut_detected,
            estimate=None,
            depth_status="UNAVAILABLE" if astronaut_detected else "NO_POSE",
        )

    def _estimate_from_position(self, sample_count: int) -> TargetEstimate:
        forward_m, left_m, up_m = (float(value) for value in self._position)
        distance_m = math.sqrt(
            forward_m * forward_m + left_m * left_m + up_m * up_m
        )
        direction, angle = horizontal_direction(forward_m, left_m)
        return TargetEstimate(
            position_m=(forward_m, left_m, up_m),
            distance_m=distance_m,
            direction=direction,
            horizontal_angle_deg=angle,
            sample_count=sample_count,
        )
