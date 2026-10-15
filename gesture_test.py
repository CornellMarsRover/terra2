#!/usr/bin/env python3
"""Recognize five astronaut gestures with a Mac webcam and MediaPipe Pose.

Recommended setup on macOS (MediaPipe 0.10.21 supports Python 3.11):

    brew install python@3.11
    python3.11 -m venv .venv-gesture
    source .venv-gesture/bin/activate
    python -m pip install --upgrade pip
    python -m pip install -r requirements-gesture-test.txt
    python gesture_test.py --zed

Allow Terminal (or the terminal application being used) under System Settings >
Privacy & Security > Camera when macOS asks. Stand far enough away that the
camera can see your shoulders, arms, and hips, and press Q in the camera window
to quit.
With --zed (the default), the program discovers the ZED by name and verifies it
by capturing one of its advertised stereo modes instead of assuming an index.
Use --no-zed --camera INDEX to explicitly select a regular webcam.

This is intentionally a small rule-based demo. Gestures use normalized 2-D
joint geometry. FOLLOW also uses MediaPipe world landmarks, when available, to
reject a torso that is noticeably tilted away from upright.
"""

from __future__ import annotations

import argparse
import math
import platform
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

try:
    import cv2
    import mediapipe as mp
except ImportError as exc:
    missing = exc.name or "a required package"
    raise SystemExit(
        f"Missing {missing}. Create the Python 3.11 environment shown at the "
        "top of gesture_test.py, then run the script again."
    ) from exc

try:
    import AVFoundation
    import CoreMedia
except ImportError as exc:
    raise SystemExit(
        "Missing macOS AVFoundation bindings. Run: "
        "python -m pip install -r requirements-gesture-test.txt"
    ) from exc


POSE = mp.solutions.pose
DRAWING = mp.solutions.drawing_utils
CONNECTIONS = POSE.POSE_CONNECTIONS

NOSE = POSE.PoseLandmark.NOSE.value
LEFT_EYE = POSE.PoseLandmark.LEFT_EYE.value
RIGHT_EYE = POSE.PoseLandmark.RIGHT_EYE.value
LEFT_EAR = POSE.PoseLandmark.LEFT_EAR.value
RIGHT_EAR = POSE.PoseLandmark.RIGHT_EAR.value
LS = POSE.PoseLandmark.LEFT_SHOULDER.value
RS = POSE.PoseLandmark.RIGHT_SHOULDER.value
LE = POSE.PoseLandmark.LEFT_ELBOW.value
RE = POSE.PoseLandmark.RIGHT_ELBOW.value
LW = POSE.PoseLandmark.LEFT_WRIST.value
RW = POSE.PoseLandmark.RIGHT_WRIST.value
LH = POSE.PoseLandmark.LEFT_HIP.value
RH = POSE.PoseLandmark.RIGHT_HIP.value

LANDMARKS_NEEDED = (LS, RS, LE, RE, LW, RW, LH, RH)


@dataclass(frozen=True)
class Detection:
    label: str = "NONE"
    confidence: float = 0.0
    detail: str = ""


@dataclass(frozen=True)
class CaptureFormat:
    width: int
    height: int
    pixel_format: str
    frame_rates: tuple[float, ...]


@dataclass(frozen=True)
class CameraDeviceInfo:
    avfoundation_index: int
    name: str
    unique_id: str
    formats: tuple[CaptureFormat, ...]


@dataclass
class CameraSession:
    capture: object
    device_name: str
    opencv_index: int
    requested_width: int
    requested_height: int
    requested_fps: float
    actual_width: int
    actual_height: int
    actual_fps: float
    source_pixel_format: str
    stereo_side_by_side: bool
    first_frame: object


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def distance(a, b, dimensions: int = 2) -> float:
    values_a = (a.x, a.y, a.z)
    values_b = (b.x, b.y, b.z)
    return math.sqrt(
        sum((values_a[index] - values_b[index]) ** 2 for index in range(dimensions))
    )


def joint_angle(a, b, c, dimensions: int = 2) -> float:
    """Return the angle ABC in degrees."""
    va = (a.x - b.x, a.y - b.y, a.z - b.z)
    vc = (c.x - b.x, c.y - b.y, c.z - b.z)
    dot = sum(va[index] * vc[index] for index in range(dimensions))
    mag_a = math.sqrt(sum(va[index] ** 2 for index in range(dimensions)))
    mag_c = math.sqrt(sum(vc[index] ** 2 for index in range(dimensions)))
    if mag_a < 1e-6 or mag_c < 1e-6:
        return 0.0
    return math.degrees(math.acos(clamp(dot / (mag_a * mag_c), -1.0, 1.0)))


def minimum_visibility(landmarks: Sequence, indexes: Iterable[int]) -> float:
    return min(float(landmarks[index].visibility) for index in indexes)


def visibility_quality(
    landmarks: Sequence,
    indexes: Iterable[int],
    minimum: float = 0.30,
) -> float:
    """Average visibility, rejecting only landmarks that are effectively lost."""
    values = [float(landmarks[index].visibility) for index in indexes]
    if not values or min(values) < minimum:
        return 0.0
    return sum(values) / len(values)


def outward(shoulder, wrist, body_center_x: float) -> bool:
    """True when a wrist is farther from the body's center than its shoulder."""
    return (wrist.x - shoulder.x) * (shoulder.x - body_center_x) > 0.0


def segments_cross(a, b, c, d) -> bool:
    """Return whether 2-D segments AB and CD cross each other."""

    def signed_area(p, q, r) -> float:
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x)

    def on_segment(p, q, r) -> bool:
        return (
            min(p.x, r.x) <= q.x <= max(p.x, r.x)
            and min(p.y, r.y) <= q.y <= max(p.y, r.y)
        )

    ab_c = signed_area(a, b, c)
    ab_d = signed_area(a, b, d)
    cd_a = signed_area(c, d, a)
    cd_b = signed_area(c, d, b)
    if ab_c * ab_d < 0.0 and cd_a * cd_b < 0.0:
        return True

    epsilon = 1e-6
    return (
        (abs(ab_c) <= epsilon and on_segment(a, c, b))
        or (abs(ab_d) <= epsilon and on_segment(a, d, b))
        or (abs(cd_a) <= epsilon and on_segment(c, a, d))
        or (abs(cd_b) <= epsilon and on_segment(c, b, d))
    )


class GestureRecognizer:
    """Rule-based pose classifier using normalized image and world landmarks."""

    def __init__(
        self,
        confidence_threshold: float = 0.65,
        fetch_min_arm_angle: float = 105.0,
        fetch_min_extension: float = 0.75,
        fetch_neck_radius: float = 0.55,
        come_hand_gap: float = 0.35,
        come_elbow_tolerance: float = 0.25,
    ):
        self.confidence_threshold = confidence_threshold
        self.fetch_min_arm_angle = fetch_min_arm_angle
        self.fetch_min_extension = fetch_min_extension
        self.fetch_neck_radius = fetch_neck_radius
        self.come_hand_gap = come_hand_gap
        self.come_elbow_tolerance = come_elbow_tolerance

    def update(self, pose_landmarks, world_landmarks) -> Detection:
        if pose_landmarks is None:
            return Detection()

        landmarks = pose_landmarks.landmark
        if len(landmarks) <= max(LANDMARKS_NEEDED):
            return Detection()

        shoulder_width = distance(landmarks[LS], landmarks[RS])
        shoulder_y = (landmarks[LS].y + landmarks[RS].y) / 2.0
        hip_y = (landmarks[LH].y + landmarks[RH].y) / 2.0
        torso_height = abs(hip_y - shoulder_y)
        if shoulder_width < 0.03 or torso_height < 0.05:
            return Detection()

        # These three poses intentionally use precedence rather than a confidence
        # contest. GIVE is checked before FETCH, and each successful result returns
        # immediately, so the three commands are mutually exclusive.
        for candidate in (
            self._give(landmarks, shoulder_width, torso_height),
            self._fetch(
                landmarks,
                world_landmarks,
                shoulder_width,
                torso_height,
            ),
            self._come(landmarks, shoulder_width, torso_height),
        ):
            if candidate.confidence >= self.confidence_threshold:
                return candidate

        best = max(
            self._follow(
                landmarks,
                world_landmarks,
                shoulder_width,
                torso_height,
            ),
            self._stay(landmarks, shoulder_width),
            key=lambda result: result.confidence,
        )
        return best if best.confidence >= self.confidence_threshold else Detection()

    def _follow(
        self,
        lm: Sequence,
        world_landmarks,
        shoulder_width: float,
        torso_height: float,
    ) -> Detection:
        """Upright pose with each hand on its matching hip and elbows out."""
        visibility = visibility_quality(lm, (LS, RS, LE, RE, LW, RW, LH, RH))
        if visibility < self.confidence_threshold:
            return Detection()

        # Scale x and y independently so the thresholds behave consistently as
        # the person moves toward or away from the camera. Inference runs before
        # the preview is mirrored, preserving anatomical left/right pairing.
        def wrist_hip_gap(wrist_index: int, hip_index: int) -> float:
            dx = (lm[wrist_index].x - lm[hip_index].x) / shoulder_width
            dy = (lm[wrist_index].y - lm[hip_index].y) / torso_height
            return math.hypot(dx, dy)

        left_hip_gap = wrist_hip_gap(LW, LH)
        right_hip_gap = wrist_hip_gap(RW, RH)
        hands_on_hips = left_hip_gap <= 0.72 and right_hip_gap <= 0.72

        center_x = (lm[LS].x + lm[RS].x + lm[LH].x + lm[RH].x) / 4.0
        left_elbow_radius = abs(lm[LE].x - center_x) / shoulder_width
        right_elbow_radius = abs(lm[RE].x - center_x) / shoulder_width
        left_wrist_radius = abs(lm[LW].x - center_x) / shoulder_width
        right_wrist_radius = abs(lm[RW].x - center_x) / shoulder_width
        left_elbow_angle = joint_angle(lm[LS], lm[LE], lm[LW])
        right_elbow_angle = joint_angle(lm[RS], lm[RE], lm[RW])
        wrists_on_own_sides = (
            (lm[LW].x - center_x) * (lm[LH].x - center_x) > 0.0
            and (lm[RW].x - center_x) * (lm[RH].x - center_x) > 0.0
            and left_wrist_radius >= 0.22
            and right_wrist_radius >= 0.22
        )

        elbows_out = (
            outward(lm[LH], lm[LE], center_x)
            and outward(lm[RH], lm[RE], center_x)
            and left_elbow_radius >= left_wrist_radius + 0.12
            and right_elbow_radius >= right_wrist_radius + 0.12
            and left_elbow_radius >= 0.55
            and right_elbow_radius >= 0.55
        )
        elbows_bent = (
            25.0 <= left_elbow_angle <= 150.0
            and 25.0 <= right_elbow_angle <= 150.0
        )

        shoulder_mid_x = (lm[LS].x + lm[RS].x) / 2.0
        hip_mid_x = (lm[LH].x + lm[RH].x) / 2.0
        image_torso_tilt = math.degrees(
            math.atan2(abs(shoulder_mid_x - hip_mid_x), torso_height)
        )
        torso_tilt = image_torso_tilt
        if world_landmarks is not None:
            world = world_landmarks.landmark
            if len(world) > max(LS, RS, LH, RH):
                shoulder_mid = tuple(
                    (getattr(world[LS], axis) + getattr(world[RS], axis)) / 2.0
                    for axis in ("x", "y", "z")
                )
                hip_mid = tuple(
                    (getattr(world[LH], axis) + getattr(world[RH], axis)) / 2.0
                    for axis in ("x", "y", "z")
                )
                vertical = abs(shoulder_mid[1] - hip_mid[1])
                horizontal = math.hypot(
                    shoulder_mid[0] - hip_mid[0],
                    shoulder_mid[2] - hip_mid[2],
                )
                if vertical > 1e-6:
                    torso_tilt = max(
                        torso_tilt,
                        math.degrees(math.atan2(horizontal, vertical)),
                    )

        upright = torso_tilt <= 40.0
        if (
            not hands_on_hips
            or not wrists_on_own_sides
            or not elbows_out
            or not elbows_bent
            or not upright
        ):
            return Detection()

        proximity_score = 1.0 - clamp(max(left_hip_gap, right_hip_gap) / 0.72)
        elbow_score = clamp(
            min(
                left_elbow_radius - left_wrist_radius,
                right_elbow_radius - right_wrist_radius,
            )
            / 0.35
        )
        upright_score = clamp(1.0 - torso_tilt / 40.0)
        return Detection(
            "FOLLOW",
            visibility
            * (0.80 + 0.10 * proximity_score + 0.06 * elbow_score + 0.04 * upright_score),
        )

    def _stay(self, lm: Sequence, shoulder_width: float) -> Detection:
        visibility = minimum_visibility(lm, (LS, RS, LE, RE, LW, RW))
        if visibility < self.confidence_threshold:
            return Detection()
        center_x = (lm[LS].x + lm[RS].x) / 2.0
        left_angle = joint_angle(lm[LS], lm[LE], lm[LW])
        right_angle = joint_angle(lm[RS], lm[RE], lm[RW])
        vertical_error = max(
            abs(lm[LE].y - lm[LS].y),
            abs(lm[LW].y - lm[LS].y),
            abs(lm[RE].y - lm[RS].y),
            abs(lm[RW].y - lm[RS].y),
        ) / shoulder_width
        extended = min(
            distance(lm[LS], lm[LW]), distance(lm[RS], lm[RW])
        ) / shoulder_width
        if (
            left_angle < 155.0
            or right_angle < 155.0
            or vertical_error > 0.42
            or extended < 0.85
            or not outward(lm[LS], lm[LW], center_x)
            or not outward(lm[RS], lm[RW], center_x)
        ):
            return Detection()
        shape_score = 0.5 * clamp((min(left_angle, right_angle) - 145.0) / 30.0)
        shape_score += 0.5 * clamp(1.0 - vertical_error / 0.42)
        return Detection("STAY", visibility * (0.80 + 0.20 * shape_score))

    def _fetch(
        self,
        lm: Sequence,
        world_landmarks,
        shoulder_width: float,
        torso_height: float,
    ) -> Detection:
        """Left hand on the neck while the right arm points in any direction."""
        center_x = (lm[LS].x + lm[RS].x) / 2.0
        shoulder_y = (lm[LS].y + lm[RS].y) / 2.0
        visibility = visibility_quality(lm, (LS, RS, LW, RE, RW, LH, RH))
        if visibility < self.confidence_threshold:
            return Detection()

        # Landmark names remain anatomical after the frame is mirrored for the
        # selfie display. Do not infer left/right from raw image x coordinates.
        neck_y = shoulder_y - 0.18 * torso_height
        neck_x_error = abs(lm[LW].x - center_x) / shoulder_width
        neck_y_error = abs(lm[LW].y - neck_y) / torso_height
        neck_distance = math.hypot(neck_x_error, neck_y_error)
        left_hand_on_neck = neck_distance <= self.fetch_neck_radius

        right_angle = joint_angle(lm[RS], lm[RE], lm[RW])
        arm_dx = lm[RW].x - lm[RS].x
        arm_dy = lm[RW].y - lm[RS].y
        right_extension = distance(lm[RS], lm[RW]) / shoulder_width
        right_vertical = arm_dy / torso_height

        # A straight elbow alone can also occur when the elbow extends past the
        # wrist. Projection confirms that the elbow lies along the shoulder-to-
        # wrist pointing ray. There is intentionally no left/right x restriction.
        arm_length_squared = arm_dx * arm_dx + arm_dy * arm_dy
        elbow_projection = 0.0
        if arm_length_squared > 1e-8:
            elbow_projection = (
                (lm[RE].x - lm[RS].x) * arm_dx
                + (lm[RE].y - lm[RS].y) * arm_dy
            ) / arm_length_squared
        right_arm_points = (
            right_angle >= self.fetch_min_arm_angle
            and right_extension >= self.fetch_min_extension
            and right_vertical >= -0.45
            and 0.05 <= elbow_projection <= 1.05
        )
        if not (left_hand_on_neck and right_arm_points):
            return Detection()

        neck_score = clamp(1.0 - neck_distance / self.fetch_neck_radius)
        pointing_score = 0.5 * clamp(
            (right_angle - self.fetch_min_arm_angle + 10.0) / 35.0
        )
        pointing_score += 0.5 * clamp(
            (right_extension - self.fetch_min_extension + 0.15) / 0.85
        )

        # World z decreases toward the camera. Prefer FORWARD when depth motion
        # clearly dominates the arm's sideways motion; otherwise reverse raw x
        # so LEFT/RIGHT matches the mirrored preview seen by the user.
        screen_direction = "LEFT" if arm_dx > 0.0 else "RIGHT"
        if world_landmarks is not None:
            world = world_landmarks.landmark
            if len(world) > max(LS, RS, RW):
                world_shoulder_width = distance(
                    world[LS], world[RS], dimensions=3
                )
                if world_shoulder_width > 1e-6:
                    forward_reach = (
                        world[RS].z - world[RW].z
                    ) / world_shoulder_width
                    lateral_reach = abs(
                        world[RW].x - world[RS].x
                    ) / world_shoulder_width
                    if (
                        forward_reach >= 0.40
                        and forward_reach >= 0.75 * lateral_reach
                    ):
                        screen_direction = "FORWARD"
        return Detection(
            "FETCH",
            visibility * (0.78 + 0.12 * neck_score + 0.10 * pointing_score),
            f"points {screen_direction}",
        )

    def _give(
        self,
        lm: Sequence,
        shoulder_width: float,
        torso_height: float,
    ) -> Detection:
        """Both forearms crossed over the chest to form an X."""
        visibility = minimum_visibility(lm, (LS, RS, LE, RE, LW, RW))
        if visibility < self.confidence_threshold:
            return Detection()

        center_x = (lm[LS].x + lm[RS].x) / 2.0
        shoulder_y = (lm[LS].y + lm[RS].y) / 2.0
        left_to_right_shoulder = distance(lm[LW], lm[RS]) / shoulder_width
        right_to_left_shoulder = distance(lm[RW], lm[LS]) / shoulder_width
        left_to_own_shoulder = distance(lm[LW], lm[LS]) / shoulder_width
        right_to_own_shoulder = distance(lm[RW], lm[RS]) / shoulder_width
        left_height = (lm[LW].y - shoulder_y) / torso_height
        right_height = (lm[RW].y - shoulder_y) / torso_height

        left_wrist_crossed = (
            (lm[LW].x - center_x) * (lm[LS].x - center_x) < 0.0
        )
        right_wrist_crossed = (
            (lm[RW].x - center_x) * (lm[RS].x - center_x) < 0.0
        )
        forearms_crossed = segments_cross(lm[LE], lm[LW], lm[RE], lm[RW])

        if (
            left_to_right_shoulder > 0.78
            or right_to_left_shoulder > 0.78
            or left_to_own_shoulder < 0.55
            or right_to_own_shoulder < 0.55
            or not (-0.25 <= left_height <= 0.75)
            or not (-0.25 <= right_height <= 0.75)
            or not left_wrist_crossed
            or not right_wrist_crossed
            or not forearms_crossed
        ):
            return Detection()

        proximity_score = 1.0 - clamp(
            max(left_to_right_shoulder, right_to_left_shoulder) / 0.78
        )
        height_score = 1.0 - clamp(
            max(abs(left_height - 0.25), abs(right_height - 0.25)) / 0.65
        )
        return Detection(
            "GIVE",
            visibility * (0.80 + 0.12 * proximity_score + 0.08 * height_score),
        )

    def _come(
        self, lm: Sequence, shoulder_width: float, torso_height: float
    ) -> Detection:
        """Joined hands above the face with relaxed, roof-shaped arms."""
        arm_visibility = minimum_visibility(lm, (LS, RS, LE, RE, LW, RW))
        visible_face = [
            lm[index]
            for index in (NOSE, LEFT_EYE, RIGHT_EYE)
            if lm[index].visibility >= self.confidence_threshold
        ]
        if arm_visibility < self.confidence_threshold or not visible_face:
            return Detection()

        # Image y increases downward. Requiring each wrist above the highest
        # reliable eye/nose landmark prevents hands joined at chest level from
        # matching while allowing bent and asymmetric arms.
        face_level_y = min(point.y for point in visible_face)
        wrists_above_face = lm[LW].y < face_level_y and lm[RW].y < face_level_y
        wrist_gap = distance(lm[LW], lm[RW]) / shoulder_width
        body_center_x = (lm[LS].x + lm[RS].x) / 2.0
        wrist_center_x = (lm[LW].x + lm[RW].x) / 2.0
        hands_centered = (
            abs(wrist_center_x - body_center_x) / shoulder_width <= 0.55
        )
        left_elbow_drop = (lm[LE].y - lm[LS].y) / torso_height
        right_elbow_drop = (lm[RE].y - lm[RS].y) / torso_height
        elbows_high_enough = (
            left_elbow_drop <= self.come_elbow_tolerance
            and right_elbow_drop <= self.come_elbow_tolerance
        )
        if (
            not wrists_above_face
            or wrist_gap > self.come_hand_gap
            or not hands_centered
            or not elbows_high_enough
        ):
            return Detection()

        height_margin = min(face_level_y - lm[LW].y, face_level_y - lm[RW].y)
        height_score = clamp(height_margin / (0.35 * torso_height))
        together_score = clamp(1.0 - wrist_gap / self.come_hand_gap)
        elbow_score = clamp(
            1.0
            - max(left_elbow_drop, right_elbow_drop, 0.0)
            / self.come_elbow_tolerance
        )
        return Detection(
            "COME",
            arm_visibility
            * (
                0.80
                + 0.09 * height_score
                + 0.08 * together_score
                + 0.03 * elbow_score
            ),
        )


class GestureStabilizer:
    """Prevent flicker by requiring a label to persist for a fixed time."""

    def __init__(
        self,
        confirmation_seconds: float = 0.4,
        follow_confirmation_seconds: float = 0.5,
        clear_seconds: float = 0.4,
    ):
        self.confirmation_seconds = confirmation_seconds
        self.follow_confirmation_seconds = follow_confirmation_seconds
        self.clear_seconds = clear_seconds
        self.stable = Detection()
        self.candidate = "NONE"
        self.candidate_since = 0.0
        self.candidate_confidence = 0.0
        self.candidate_detail = ""

    def update(self, observation: Detection, now: float) -> Detection:
        if observation.label == self.stable.label:
            self.candidate = observation.label
            self.candidate_since = now
            if observation.label != "NONE":
                self.stable = observation
            return self.stable

        if observation.label == self.candidate:
            self.candidate_confidence = min(
                self.candidate_confidence, observation.confidence
            )
            self.candidate_detail = observation.detail
        else:
            self.candidate = observation.label
            self.candidate_since = now
            self.candidate_confidence = observation.confidence
            self.candidate_detail = observation.detail

        if observation.label == "NONE":
            needed = self.clear_seconds
        elif observation.label == "FOLLOW":
            needed = self.follow_confirmation_seconds
        else:
            needed = self.confirmation_seconds
        if now - self.candidate_since >= needed:
            self.stable = Detection(
                self.candidate,
                self.candidate_confidence,
                self.candidate_detail,
            )
            self.candidate_since = now
        return self.stable


def _fourcc_text(code: int) -> str:
    return bytes((code >> shift) & 0xFF for shift in (24, 16, 8, 0)).decode(
        "ascii", errors="replace"
    )


def enumerate_avfoundation_cameras() -> list[CameraDeviceInfo]:
    """Return native macOS camera names and advertised capture formats."""
    if platform.system() != "Darwin":
        return []

    devices = AVFoundation.AVCaptureDevice.devicesWithMediaType_(
        AVFoundation.AVMediaTypeVideo
    )
    result = []
    for index, device in enumerate(devices):
        formats = []
        seen = set()
        for native_format in device.formats():
            description = native_format.formatDescription()
            dimensions = CoreMedia.CMVideoFormatDescriptionGetDimensions(description)
            pixel_code = CoreMedia.CMFormatDescriptionGetMediaSubType(description)
            pixel_format = _fourcc_text(int(pixel_code))
            frame_rates = tuple(
                sorted(
                    {
                        round(float(rate.maxFrameRate()), 2)
                        for rate in native_format.videoSupportedFrameRateRanges()
                    },
                    reverse=True,
                )
            )
            key = (
                int(dimensions.width),
                int(dimensions.height),
                pixel_format,
                frame_rates,
            )
            if key in seen:
                continue
            seen.add(key)
            formats.append(CaptureFormat(*key))
        result.append(
            CameraDeviceInfo(
                avfoundation_index=index,
                name=str(device.localizedName()),
                unique_id=str(device.uniqueID()),
                formats=tuple(formats),
            )
        )
    return result


def print_camera_inventory(devices: Sequence[CameraDeviceInfo]) -> None:
    permission_codes = {
        int(AVFoundation.AVAuthorizationStatusNotDetermined): "not determined",
        int(AVFoundation.AVAuthorizationStatusRestricted): "restricted",
        int(AVFoundation.AVAuthorizationStatusDenied): "denied",
        int(AVFoundation.AVAuthorizationStatusAuthorized): "authorized",
    }
    permission = int(
        AVFoundation.AVCaptureDevice.authorizationStatusForMediaType_(
            AVFoundation.AVMediaTypeVideo
        )
    )
    print(
        "AVFoundation cameras "
        f"(permission: {permission_codes.get(permission, str(permission))}):"
    )
    if not devices:
        print("  none")
        return
    for device in devices:
        print(
            f"  AV[{device.avfoundation_index}] {device.name} "
            f"(id={device.unique_id})"
        )
        for item in device.formats:
            rates = ",".join(f"{rate:g}" for rate in item.frame_rates)
            print(
                f"    {item.width}x{item.height} "
                f"pixel={item.pixel_format} fps={rates}"
            )
    print(
        "  Note: OpenCV's numeric order can differ from the AVFoundation list; "
        "capture probing maps them below."
    )


def read_frame_with_retries(camera, attempts: int = 20):
    last_error = "camera returned no image"
    for attempt in range(1, attempts + 1):
        try:
            ok, frame = camera.read()
        except cv2.error as exc:
            ok, frame = False, None
            last_error = str(exc)
        if ok and frame is not None and frame.size:
            return frame, ""
        if attempt < attempts:
            time.sleep(0.03)
    return None, last_error


def probe_opencv_camera(
    index: int,
    width: int,
    height: int,
    fps: float,
    attempts: int = 20,
):
    camera = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    if not camera.isOpened():
        camera.release()
        return None, f"OpenCV could not open AVFoundation index {index}"

    set_width = camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    set_height = camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    set_fps = camera.set(cv2.CAP_PROP_FPS, fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    frame, read_error = read_frame_with_retries(camera, attempts=attempts)
    if frame is None:
        negotiated = (
            int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            camera.get(cv2.CAP_PROP_FPS),
        )
        camera.release()
        return None, (
            f"index {index} opened but produced no frame after {attempts} reads; "
            f"requested={width}x{height}@{fps:g}, negotiated="
            f"{negotiated[0]}x{negotiated[1]}@{negotiated[2]:g}, "
            f"set_results=({set_width}, {set_height}, {set_fps}), "
            f"last_error={read_error}"
        )

    actual_height, actual_width = frame.shape[:2]
    actual_fps = float(camera.get(cv2.CAP_PROP_FPS))
    return (
        CameraSession(
            capture=camera,
            device_name="unmapped camera",
            opencv_index=index,
            requested_width=width,
            requested_height=height,
            requested_fps=fps,
            actual_width=actual_width,
            actual_height=actual_height,
            actual_fps=actual_fps,
            source_pixel_format="unknown",
            stereo_side_by_side=False,
            first_frame=frame,
        ),
        "",
    )


def zed_probe_modes(device: CameraDeviceInfo) -> list[CaptureFormat]:
    preferred_sizes = ((2560, 720), (1344, 376), (3840, 1080), (4416, 1242))
    by_size = {(item.width, item.height): item for item in device.formats}
    ordered = [by_size[size] for size in preferred_sizes if size in by_size]
    ordered.extend(item for item in device.formats if item not in ordered)
    return ordered


def open_zed_camera(
    devices: Sequence[CameraDeviceInfo], requested_index: int | None
) -> CameraSession | None:
    zed_devices = [device for device in devices if "zed" in device.name.casefold()]
    if not zed_devices:
        print(
            "ZED selection failed: AVFoundation does not report a device whose "
            "name contains 'ZED'.",
            file=sys.stderr,
        )
        return None
    zed = zed_devices[0]
    modes = zed_probe_modes(zed)
    if not modes:
        print(
            f"ZED selection failed: {zed.name} advertises no video formats.",
            file=sys.stderr,
        )
        return None

    indices = (
        [requested_index]
        if requested_index is not None
        else list(range(len(devices)))
    )
    errors = []
    print(f"Locating {zed.name!r} across OpenCV indices {indices}...")
    for item in modes:
        fps = 30.0 if any(rate >= 30.0 for rate in item.frame_rates) else max(
            item.frame_rates, default=15.0
        )
        for index in indices:
            session, error = probe_opencv_camera(
                index,
                item.width,
                item.height,
                fps,
            )
            if session is None:
                errors.append(error)
                print(f"  probe OpenCV[{index}] failed: {error}")
                continue

            actual = (session.actual_width, session.actual_height)
            expected = (item.width, item.height)
            print(
                f"  probe OpenCV[{index}] requested {item.width}x{item.height} "
                f"{item.pixel_format}@{fps:g}; captured "
                f"{session.actual_width}x{session.actual_height} BGR@"
                f"{session.actual_fps:g}"
            )
            if actual == expected and session.actual_width / session.actual_height > 2.4:
                session.device_name = zed.name
                session.source_pixel_format = item.pixel_format
                session.stereo_side_by_side = True
                return session
            session.capture.release()
            errors.append(
                f"OpenCV[{index}] negotiated {actual[0]}x{actual[1]}, "
                f"not ZED mode {expected[0]}x{expected[1]}"
            )

    print(
        "ZED selection failed: AVFoundation sees the camera, but OpenCV could "
        "not capture any advertised side-by-side ZED mode. No webcam fallback "
        "was selected.",
        file=sys.stderr,
    )
    for error in errors:
        print(f"  - {error}", file=sys.stderr)
    print(
        "Check macOS Camera permission, close other camera applications, use a "
        "USB 3 port/cable, and try --preview. The full ZED SDK is not available "
        "on macOS; this program relies on the camera's UVC stream.",
        file=sys.stderr,
    )
    return None


def identify_regular_camera(
    devices: Sequence[CameraDeviceInfo], width: int, height: int
) -> str:
    matches = [
        device.name
        for device in devices
        if any(
            item.width == width and item.height == height for item in device.formats
        )
    ]
    return matches[0] if len(matches) == 1 else "unmapped camera"


def open_regular_camera(
    devices: Sequence[CameraDeviceInfo], requested_index: int | None
) -> CameraSession | None:
    if requested_index is not None:
        session, error = probe_opencv_camera(requested_index, 1280, 720, 30.0)
        if session is None:
            print(f"Camera selection failed: {error}", file=sys.stderr)
            return None
        session.device_name = identify_regular_camera(
            devices, session.actual_width, session.actual_height
        )
        return session

    regular_devices = [
        device for device in devices if "zed" not in device.name.casefold()
    ]
    if not regular_devices:
        print("No non-ZED AVFoundation camera was found.", file=sys.stderr)
        return None
    target = regular_devices[0]
    preferred_sizes = ((1280, 720), (1920, 1080), (640, 480))
    by_size = {(item.width, item.height): item for item in target.formats}
    modes = [by_size[size] for size in preferred_sizes if size in by_size]
    modes.extend(item for item in target.formats if item not in modes)
    print(f"Locating {target.name!r} across OpenCV indices...")
    for item in modes:
        fps = min(30.0, max(item.frame_rates, default=30.0))
        for index in range(len(devices)):
            session, error = probe_opencv_camera(
                index, item.width, item.height, fps
            )
            if session is None:
                print(f"  probe OpenCV[{index}] failed: {error}")
                continue
            actual = (session.actual_width, session.actual_height)
            print(
                f"  probe OpenCV[{index}] requested {item.width}x{item.height}; "
                f"captured {actual[0]}x{actual[1]}"
            )
            if actual == (item.width, item.height):
                session.device_name = target.name
                session.source_pixel_format = item.pixel_format
                return session
            session.capture.release()
    print(
        f"AVFoundation sees {target.name!r}, but OpenCV could not verify its feed.",
        file=sys.stderr,
    )
    return None


def prepare_camera_frame(session: CameraSession, frame):
    if not session.stereo_side_by_side:
        return frame
    height, width = frame.shape[:2]
    if width != session.actual_width or height != session.actual_height:
        raise RuntimeError(
            "ZED frame size changed after verification: "
            f"expected {session.actual_width}x{session.actual_height}, "
            f"received {width}x{height}"
        )
    return frame[:, : width // 2]


def print_selected_camera(session: CameraSession) -> None:
    layout = (
        "side-by-side stereo; using left eye"
        if session.stereo_side_by_side
        else "single image"
    )
    print(
        f"Selected OpenCV[{session.opencv_index}]: {session.device_name}; "
        f"requested={session.requested_width}x{session.requested_height}@"
        f"{session.requested_fps:g}; captured="
        f"{session.actual_width}x{session.actual_height} BGR@{session.actual_fps:g}; "
        f"source_pixel={session.source_pixel_format}; layout={layout}"
    )


def print_capture_read_error(session: CameraSession, message: str) -> None:
    camera = session.capture
    print(
        f"Camera frame capture failed for OpenCV[{session.opencv_index}] "
        f"{session.device_name}: {message}; current_properties="
        f"{int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
        f"{int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))}@"
        f"{camera.get(cv2.CAP_PROP_FPS):g}; backend="
        f"{camera.getBackendName() if camera.isOpened() else 'closed'}",
        file=sys.stderr,
    )


def run_camera_preview(session: CameraSession) -> int:
    print("Camera-only preview active. Press Q in the preview window to quit.")
    frame = session.first_frame
    session.first_frame = None
    try:
        while True:
            if frame is None:
                frame, error = read_frame_with_retries(session.capture, attempts=8)
                if frame is None:
                    print_capture_read_error(session, error)
                    return 1
            try:
                view = prepare_camera_frame(session, frame)
            except RuntimeError as exc:
                print_capture_read_error(session, str(exc))
                return 1
            view = cv2.flip(view, 1)
            cv2.putText(
                view,
                f"{session.device_name} | {view.shape[1]}x{view.shape[0]} | Q: quit",
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (40, 220, 40),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Camera Preview", view)
            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                return 0
            frame = None
    except KeyboardInterrupt:
        return 0
    finally:
        session.capture.release()
        cv2.destroyAllWindows()


def draw_status(frame, detection: Detection, fps: float) -> None:
    color = (40, 210, 40) if detection.label != "NONE" else (80, 80, 255)
    detail = f" ({detection.detail})" if detection.detail else ""
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 92), (20, 20, 20), -1)
    cv2.putText(
        frame,
        f"Gesture: {detection.label}{detail}",
        (22, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.05,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"confidence {detection.confidence:.2f}   {fps:.0f} FPS   Q: quit",
        (24, 76),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (225, 225, 225),
        1,
        cv2.LINE_AA,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera",
        type=int,
        default=None,
        help=(
            "limit ZED probing to one OpenCV index, or select this index with "
            "--no-zed (default: auto-detect by verified capture format)"
        ),
    )
    parser.add_argument(
        "--zed",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="auto-detect ZED UVC stereo and select its left eye (default: enabled)",
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="list AVFoundation names and formats, then exit",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="show the selected camera without running MediaPipe",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.65,
        help="minimum gesture/landmark confidence from 0 to 1 (default: 0.65)",
    )
    parser.add_argument(
        "--fetch-min-arm-angle",
        type=float,
        default=105.0,
        help="minimum straightness of the pointing elbow in degrees (default: 105)",
    )
    parser.add_argument(
        "--fetch-min-extension",
        type=float,
        default=0.75,
        help="minimum shoulder-to-wrist distance in shoulder widths (default: 0.75)",
    )
    parser.add_argument(
        "--fetch-neck-radius",
        type=float,
        default=0.55,
        help="maximum normalized left-wrist distance from the neck (default: 0.55)",
    )
    parser.add_argument(
        "--come-hand-gap",
        type=float,
        default=0.35,
        help="maximum COME wrist separation in shoulder widths (default: 0.35)",
    )
    parser.add_argument(
        "--come-elbow-tolerance",
        type=float,
        default=0.25,
        help=(
            "maximum COME elbow drop below its shoulder in torso heights "
            "(default: 0.25)"
        ),
    )
    args = parser.parse_args()
    if not 0.0 < args.confidence <= 1.0:
        parser.error("--confidence must be greater than 0 and at most 1")
    if not 90.0 <= args.fetch_min_arm_angle < 180.0:
        parser.error("--fetch-min-arm-angle must be from 90 up to 180 degrees")
    if args.fetch_min_extension <= 0.0:
        parser.error("--fetch-min-extension must be positive")
    if args.fetch_neck_radius <= 0.0:
        parser.error("--fetch-neck-radius must be positive")
    if args.come_hand_gap <= 0.0:
        parser.error("--come-hand-gap must be positive")
    if args.come_elbow_tolerance <= 0.0:
        parser.error("--come-elbow-tolerance must be positive")
    return args


def main() -> int:
    args = parse_args()
    devices = enumerate_avfoundation_cameras()
    print_camera_inventory(devices)
    if args.list_cameras:
        return 0

    session = (
        open_zed_camera(devices, args.camera)
        if args.zed
        else open_regular_camera(devices, args.camera)
    )
    if session is None:
        return 1
    print_selected_camera(session)
    if args.preview:
        return run_camera_preview(session)

    camera = session.capture
    recognizer = GestureRecognizer(
        args.confidence,
        fetch_min_arm_angle=args.fetch_min_arm_angle,
        fetch_min_extension=args.fetch_min_extension,
        fetch_neck_radius=args.fetch_neck_radius,
        come_hand_gap=args.come_hand_gap,
        come_elbow_tolerance=args.come_elbow_tolerance,
    )
    stabilizer = GestureStabilizer()
    last_printed = None
    last_frame_time = time.monotonic()
    smoothed_fps = 0.0

    print("Stand where your shoulders, arms, and hips are visible.")
    print("Press Q in the camera window to quit.")
    pending_frame = session.first_frame
    session.first_frame = None
    try:
        with POSE.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.55,
            min_tracking_confidence=0.55,
        ) as pose:
            while True:
                if pending_frame is not None:
                    frame = pending_frame
                    pending_frame = None
                else:
                    frame, error = read_frame_with_retries(camera, attempts=8)
                    if frame is None:
                        print_capture_read_error(session, error)
                        break
                try:
                    frame = prepare_camera_frame(session, frame)
                except RuntimeError as exc:
                    print_capture_read_error(session, str(exc))
                    break

                # Run inference on the camera's unmirrored image so MediaPipe's
                # LEFT/RIGHT landmark names remain anatomical.
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                rgb.flags.writeable = False
                results = pose.process(rgb)
                now = time.monotonic()
                observation = recognizer.update(
                    results.pose_landmarks,
                    results.pose_world_landmarks,
                )
                stable = stabilizer.update(observation, now)

                if stable.label != last_printed:
                    print(f"Detected: {stable.label}", flush=True)
                    last_printed = stable.label

                if results.pose_landmarks is not None:
                    DRAWING.draw_landmarks(frame, results.pose_landmarks, CONNECTIONS)

                # Mirror only the final annotated image for an intuitive selfie
                # view. Text is added afterward so it remains readable.
                frame = cv2.flip(frame, 1)
                elapsed = max(now - last_frame_time, 1e-6)
                instant_fps = 1.0 / elapsed
                smoothed_fps = (
                    instant_fps if smoothed_fps == 0.0 else 0.9 * smoothed_fps + 0.1 * instant_fps
                )
                last_frame_time = now
                draw_status(frame, stable, smoothed_fps)
                cv2.imshow("Astronaut Gesture Test", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        camera.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
