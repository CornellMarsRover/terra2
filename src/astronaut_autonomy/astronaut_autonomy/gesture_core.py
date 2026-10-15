"""Pure astronaut-pose gesture classification and temporal stabilization."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Sequence


# Ultralytics pose models use the 17-keypoint COCO ordering.
LEFT_SHOULDER = 5
RIGHT_SHOULDER = 6
LEFT_ELBOW = 7
RIGHT_ELBOW = 8
LEFT_WRIST = 9
RIGHT_WRIST = 10


@dataclass(frozen=True)
class GestureResult:
    label: str
    confidence: float


def _point(
    keypoints: Sequence[Sequence[float]],
    index: int,
    min_confidence: float,
) -> tuple[float, float, float] | None:
    if index >= len(keypoints) or len(keypoints[index]) < 3:
        return None
    x, y, confidence = (float(value) for value in keypoints[index][:3])
    if confidence < min_confidence:
        return None
    return x, y, confidence


def classify_pose(
    keypoints: Sequence[Sequence[float]],
    min_keypoint_confidence: float = 0.35,
) -> GestureResult:
    """Classify a small set of static gestures from one COCO-format pose.

    Labels describe the person's anatomical side, not the left/right side of
    the camera image. The state machine can map these labels to rover actions
    after the behavior is reviewed and tested.
    """

    points = {
        index: _point(keypoints, index, min_keypoint_confidence)
        for index in (
            LEFT_SHOULDER,
            RIGHT_SHOULDER,
            LEFT_ELBOW,
            RIGHT_ELBOW,
            LEFT_WRIST,
            RIGHT_WRIST,
        )
    }
    left_shoulder = points[LEFT_SHOULDER]
    right_shoulder = points[RIGHT_SHOULDER]
    if left_shoulder is None or right_shoulder is None:
        return GestureResult("none", 0.0)

    shoulder_width = hypot(
        left_shoulder[0] - right_shoulder[0],
        left_shoulder[1] - right_shoulder[1],
    )
    if shoulder_width < 1.0:
        return GestureResult("none", 0.0)

    left_elbow = points[LEFT_ELBOW]
    right_elbow = points[RIGHT_ELBOW]
    left_wrist = points[LEFT_WRIST]
    right_wrist = points[RIGHT_WRIST]

    left_up = (
        left_wrist is not None
        and left_wrist[1] < left_shoulder[1] - 0.25 * shoulder_width
    )
    right_up = (
        right_wrist is not None
        and right_wrist[1] < right_shoulder[1] - 0.25 * shoulder_width
    )
    if left_up and right_up:
        return GestureResult(
            "hands_up",
            min(left_wrist[2], right_wrist[2]),
        )

    def arm_is_out(shoulder, elbow, wrist) -> bool:
        if elbow is None or wrist is None:
            return False
        vertical_tolerance = 0.45 * shoulder_width
        wrist_extension = abs(wrist[0] - shoulder[0])
        elbow_extension = abs(elbow[0] - shoulder[0])
        return (
            abs(wrist[1] - shoulder[1]) <= vertical_tolerance
            and abs(elbow[1] - shoulder[1]) <= vertical_tolerance
            and wrist_extension >= 1.2 * shoulder_width
            and elbow_extension >= 0.4 * shoulder_width
        )

    left_out = arm_is_out(left_shoulder, left_elbow, left_wrist)
    right_out = arm_is_out(right_shoulder, right_elbow, right_wrist)

    if left_out and right_out:
        return GestureResult(
            "both_arms_out",
            min(left_elbow[2], left_wrist[2], right_elbow[2], right_wrist[2]),
        )
    if left_out:
        return GestureResult(
            "left_arm_out",
            min(left_elbow[2], left_wrist[2]),
        )
    if right_out:
        return GestureResult(
            "right_arm_out",
            min(right_elbow[2], right_wrist[2]),
        )
    return GestureResult("none", 0.0)


class GestureStabilizer:
    """Require repeated observations before changing the published gesture."""

    def __init__(self, confirmation_frames: int = 5, clear_frames: int = 5):
        if confirmation_frames < 1 or clear_frames < 1:
            raise ValueError("frame thresholds must be positive")
        self.confirmation_frames = int(confirmation_frames)
        self.clear_frames = int(clear_frames)
        self.stable = GestureResult("none", 0.0)
        self._candidate = "none"
        self._candidate_confidence = 0.0
        self._candidate_count = 0

    def update(self, observation: GestureResult) -> GestureResult:
        if observation.label == self.stable.label:
            self._candidate = observation.label
            self._candidate_confidence = observation.confidence
            self._candidate_count = 0
            if observation.label != "none":
                self.stable = observation
            return self.stable

        if observation.label == self._candidate:
            self._candidate_count += 1
            self._candidate_confidence = min(
                self._candidate_confidence,
                observation.confidence,
            )
        else:
            self._candidate = observation.label
            self._candidate_confidence = observation.confidence
            self._candidate_count = 1

        required = (
            self.clear_frames
            if observation.label == "none"
            else self.confirmation_frames
        )
        if self._candidate_count >= required:
            self.stable = GestureResult(
                self._candidate,
                self._candidate_confidence,
            )
            self._candidate_count = 0
        return self.stable
