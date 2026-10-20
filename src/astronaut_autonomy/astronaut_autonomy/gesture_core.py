"""Pure astronaut-pose gesture classification and temporal stabilization."""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, degrees, hypot
from typing import Sequence


# Ultralytics pose models use the 17-keypoint COCO ordering. Left and right are
# the person's anatomical sides and must not be swapped for a mirrored display.
NOSE = 0
LEFT_EYE = 1
RIGHT_EYE = 2
LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_ELBOW = 7
RIGHT_ELBOW = 8
LEFT_WRIST = 9
RIGHT_WRIST = 10
LEFT_HIP = 11
RIGHT_HIP = 12


@dataclass(frozen=True)
class GestureResult:
    label: str
    confidence: float
    detail: str = ""


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _point(
    keypoints: Sequence[Sequence[float]],
    index: int,
    minimum_confidence: float,
) -> tuple[float, float, float] | None:
    if index >= len(keypoints) or len(keypoints[index]) < 3:
        return None
    x, y, confidence = (float(value) for value in keypoints[index][:3])
    if confidence < minimum_confidence:
        return None
    return x, y, confidence


def _distance(first, second) -> float:
    return hypot(first[0] - second[0], first[1] - second[1])


def _joint_angle(first, middle, last) -> float:
    first_vector = (first[0] - middle[0], first[1] - middle[1])
    last_vector = (last[0] - middle[0], last[1] - middle[1])
    first_length = hypot(*first_vector)
    last_length = hypot(*last_vector)
    if first_length < 1e-6 or last_length < 1e-6:
        return 0.0
    cosine = (
        first_vector[0] * last_vector[0]
        + first_vector[1] * last_vector[1]
    ) / (first_length * last_length)
    return degrees(acos(_clamp(cosine, -1.0, 1.0)))


def _orientation(first, second, third) -> float:
    return (
        (second[0] - first[0]) * (third[1] - first[1])
        - (second[1] - first[1]) * (third[0] - first[0])
    )


def _segments_cross(first_start, first_end, second_start, second_end) -> bool:
    return (
        _orientation(first_start, first_end, second_start)
        * _orientation(first_start, first_end, second_end)
        <= 0.0
        and _orientation(second_start, second_end, first_start)
        * _orientation(second_start, second_end, first_end)
        <= 0.0
    )


def classify_pose(
    keypoints: Sequence[Sequence[float]],
    min_keypoint_confidence: float = 0.35,
) -> GestureResult:
    """Classify the rover's five static astronaut commands.

    Rules are scale-independent, using shoulder width and torso height. GIVE is
    deliberately evaluated before FETCH, and COME before the remaining poses,
    so the commands are mutually exclusive.
    """

    indices = (
        NOSE,
        LEFT_EYE,
        RIGHT_EYE,
        LEFT_SHOULDER,
        RIGHT_SHOULDER,
        LEFT_ELBOW,
        RIGHT_ELBOW,
        LEFT_WRIST,
        RIGHT_WRIST,
        LEFT_HIP,
        RIGHT_HIP,
    )
    points = {
        index: _point(keypoints, index, min_keypoint_confidence)
        for index in indices
    }
    required = (
        LEFT_SHOULDER,
        RIGHT_SHOULDER,
        LEFT_ELBOW,
        RIGHT_ELBOW,
        LEFT_WRIST,
        RIGHT_WRIST,
        LEFT_HIP,
        RIGHT_HIP,
    )
    if any(points[index] is None for index in required):
        return GestureResult("NONE", 0.0)

    left_shoulder = points[LEFT_SHOULDER]
    right_shoulder = points[RIGHT_SHOULDER]
    left_elbow = points[LEFT_ELBOW]
    right_elbow = points[RIGHT_ELBOW]
    left_wrist = points[LEFT_WRIST]
    right_wrist = points[RIGHT_WRIST]
    left_hip = points[LEFT_HIP]
    right_hip = points[RIGHT_HIP]
    shoulder_width = _distance(left_shoulder, right_shoulder)
    shoulder_center = (
        (left_shoulder[0] + right_shoulder[0]) / 2.0,
        (left_shoulder[1] + right_shoulder[1]) / 2.0,
    )
    hip_center = (
        (left_hip[0] + right_hip[0]) / 2.0,
        (left_hip[1] + right_hip[1]) / 2.0,
    )
    torso_height = _distance(shoulder_center, hip_center)
    if shoulder_width < 1.0 or torso_height < 1.0:
        return GestureResult("NONE", 0.0)

    confidence = min(points[index][2] for index in required)
    torso_center_x = (shoulder_center[0] + hip_center[0]) / 2.0

    # GIVE: forearms form an X, with each wrist near the opposite shoulder.
    left_to_opposite = _distance(left_wrist, right_shoulder) / shoulder_width
    right_to_opposite = _distance(right_wrist, left_shoulder) / shoulder_width
    wrists_at_chest = all(
        shoulder_center[1] - 0.35 * torso_height
        <= wrist[1]
        <= hip_center[1] + 0.15 * torso_height
        for wrist in (left_wrist, right_wrist)
    )
    give = (
        left_to_opposite <= 0.80
        and right_to_opposite <= 0.80
        and wrists_at_chest
        and _segments_cross(left_elbow, left_wrist, right_elbow, right_wrist)
    )
    if give:
        geometry_score = 1.0 - 0.5 * (
            left_to_opposite + right_to_opposite
        ) / 0.80
        return GestureResult(
            "GIVE",
            min(confidence, _clamp(0.65 + 0.35 * geometry_score, 0.0, 1.0)),
        )

    # FETCH: anatomical left wrist at the neck while the right arm points in
    # any horizontal/diagonal direction. Image mirroring does not alter the
    # model's anatomical keypoint labels.
    neck_center = (
        shoulder_center[0],
        shoulder_center[1] - 0.12 * torso_height,
    )
    left_at_neck = _distance(left_wrist, neck_center) <= 0.72 * shoulder_width
    right_angle = _joint_angle(right_shoulder, right_elbow, right_wrist)
    right_reach = _distance(right_shoulder, right_wrist) / shoulder_width
    right_displacement = (
        right_wrist[0] - right_shoulder[0],
        right_wrist[1] - right_shoulder[1],
    )
    clear_direction = hypot(*right_displacement) >= 1.00 * shoulder_width
    fetch = (
        left_at_neck
        and right_angle >= 142.0
        and right_reach >= 1.25
        and clear_direction
    )
    if fetch:
        direction = "LEFT" if right_displacement[0] < 0.0 else "RIGHT"
        angle_score = _clamp((right_angle - 142.0) / 38.0, 0.0, 1.0)
        return GestureResult(
            "FETCH",
            min(confidence, 0.68 + 0.25 * angle_score),
            detail=direction,
        )

    # COME: relaxed rocket/roof pose. Hands dominate the test; elbows may be
    # bent and only need to be approximately shoulder height or higher.
    face_points = [points[index] for index in (NOSE, LEFT_EYE, RIGHT_EYE)]
    face_points = [point for point in face_points if point is not None]
    face_y = (
        sum(point[1] for point in face_points) / len(face_points)
        if face_points
        else shoulder_center[1] - 0.65 * torso_height
    )
    wrist_gap = _distance(left_wrist, right_wrist) / shoulder_width
    hands_above_face = max(left_wrist[1], right_wrist[1]) < face_y
    elbows_high_enough = (
        left_elbow[1] <= left_shoulder[1] + 0.25 * torso_height
        and right_elbow[1] <= right_shoulder[1] + 0.25 * torso_height
    )
    come = hands_above_face and wrist_gap <= 0.40 and elbows_high_enough
    if come:
        gap_score = 1.0 - wrist_gap / 0.40
        return GestureResult(
            "COME",
            min(confidence, 0.70 + 0.25 * gap_score),
        )

    # FOLLOW: both wrists on their corresponding hips, outward elbows, and an
    # upright torso. The x-side checks reject crossed hands and FETCH-like poses.
    left_hip_distance = _distance(left_wrist, left_hip) / shoulder_width
    right_hip_distance = _distance(right_wrist, right_hip) / shoulder_width
    left_elbow_out = abs(left_elbow[0] - torso_center_x) >= 0.55 * shoulder_width
    right_elbow_out = abs(right_elbow[0] - torso_center_x) >= 0.55 * shoulder_width
    elbows_opposite_sides = (
        (left_elbow[0] - torso_center_x)
        * (right_elbow[0] - torso_center_x)
        < 0.0
    )
    wrists_on_corresponding_sides = (
        (left_wrist[0] - torso_center_x)
        * (left_hip[0] - torso_center_x)
        >= 0.0
        and (right_wrist[0] - torso_center_x)
        * (right_hip[0] - torso_center_x)
        >= 0.0
    )
    torso_tilt = abs(shoulder_center[0] - hip_center[0]) / torso_height
    follow = (
        left_hip_distance <= 0.78
        and right_hip_distance <= 0.78
        and left_elbow_out
        and right_elbow_out
        and elbows_opposite_sides
        and wrists_on_corresponding_sides
        and torso_tilt <= 0.35
    )
    if follow:
        distance_score = 1.0 - 0.5 * (
            left_hip_distance + right_hip_distance
        ) / 0.78
        return GestureResult(
            "FOLLOW",
            min(confidence, _clamp(0.68 + 0.27 * distance_score, 0.0, 1.0)),
        )

    # STAY: both arms form a horizontal T-pose.
    def arm_is_horizontal(shoulder, elbow, wrist) -> bool:
        return (
            abs(elbow[1] - shoulder[1]) <= 0.38 * shoulder_width
            and abs(wrist[1] - shoulder[1]) <= 0.45 * shoulder_width
            and _distance(shoulder, wrist) >= 1.15 * shoulder_width
            and _joint_angle(shoulder, elbow, wrist) >= 145.0
        )

    stay = arm_is_horizontal(
        left_shoulder, left_elbow, left_wrist
    ) and arm_is_horizontal(right_shoulder, right_elbow, right_wrist)
    if stay:
        return GestureResult("STAY", confidence)

    return GestureResult("NONE", 0.0)


class GestureStabilizer:
    """Require repeated observations before changing the published gesture."""

    def __init__(self, confirmation_frames: int = 5, clear_frames: int = 5):
        if confirmation_frames < 1 or clear_frames < 1:
            raise ValueError("frame thresholds must be positive")
        self.confirmation_frames = int(confirmation_frames)
        self.clear_frames = int(clear_frames)
        self.stable = GestureResult("NONE", 0.0)
        self._candidate = "NONE"
        self._candidate_confidence = 0.0
        self._candidate_detail = ""
        self._candidate_count = 0

    def update(self, observation: GestureResult) -> GestureResult:
        if observation.label == self.stable.label:
            self._candidate = observation.label
            self._candidate_confidence = observation.confidence
            self._candidate_detail = observation.detail
            self._candidate_count = 0
            if observation.label != "NONE":
                self.stable = observation
            return self.stable

        if observation.label == self._candidate:
            self._candidate_count += 1
            self._candidate_confidence = min(
                self._candidate_confidence,
                observation.confidence,
            )
            self._candidate_detail = observation.detail
        else:
            self._candidate = observation.label
            self._candidate_confidence = observation.confidence
            self._candidate_detail = observation.detail
            self._candidate_count = 1

        required = (
            self.clear_frames
            if observation.label == "NONE"
            else self.confirmation_frames
        )
        if self._candidate_count >= required:
            self.stable = GestureResult(
                self._candidate,
                self._candidate_confidence,
                self._candidate_detail,
            )
            self._candidate_count = 0
        return self.stable
