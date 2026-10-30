"""ROS-independent arbitration of gesture and voice astronaut commands."""

from __future__ import annotations

import math
from dataclasses import dataclass

from astronaut_autonomy.mission_core import COMMANDS, PerceptionSample


VALID_INPUT_SOURCES = frozenset(("GESTURE", "VOICE"))


@dataclass(frozen=True)
class FusionConfig:
    minimum_gesture_confidence: float = 0.50
    minimum_voice_confidence: float = 0.70
    gesture_timeout_s: float = 0.75
    voice_timeout_s: float = 1.25
    depth_timeout_s: float = 0.75
    duplicate_voice_window_s: float = 1.50
    maximum_future_skew_s: float = 0.10

    def __post_init__(self):
        for value in (
            self.minimum_gesture_confidence,
            self.minimum_voice_confidence,
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError("fusion confidence thresholds must be in [0, 1]")
        for value in (
            self.gesture_timeout_s,
            self.voice_timeout_s,
            self.depth_timeout_s,
            self.duplicate_voice_window_s,
            self.maximum_future_skew_s,
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("fusion timeouts must be finite and nonnegative")


@dataclass(frozen=True)
class CommandObservation:
    command: str
    confidence: float
    source: str
    observed_at_s: float
    transcript: str = ""


@dataclass(frozen=True)
class FusionDecision:
    command: str
    confidence: float
    source: str
    reason: str
    perception: PerceptionSample | None
    safety_stop: bool = False


class CommandFusion:
    """Prefer safe, fresh inputs while retaining ZED depth for every source."""

    def __init__(self, config: FusionConfig | None = None):
        self.config = config or FusionConfig()
        self._gesture = None
        self._gesture_perception = None
        self._voice = None
        self._last_voice_command = None
        self._last_voice_time = -math.inf

    def update_gesture(
        self,
        sample: PerceptionSample,
        observed_at_s: float,
        source: str = "GESTURE",
        received_at_s: float | None = None,
    ) -> bool:
        observation = CommandObservation(
            command=sample.command,
            confidence=sample.confidence,
            source=source,
            observed_at_s=observed_at_s,
        )
        if not self._structurally_valid(
            observation,
            "GESTURE",
            allow_none=True,
        ):
            return False
        if received_at_s is not None and not self._age_is_valid(
            observed_at_s,
            received_at_s,
            self.config.gesture_timeout_s,
        ):
            return False
        self._gesture = observation
        self._gesture_perception = sample
        return True

    def update_voice(
        self,
        observation: CommandObservation,
        received_at_s: float | None = None,
    ) -> bool:
        if not self._structurally_valid(observation, "VOICE"):
            return False
        if received_at_s is not None and not self._age_is_valid(
            observation.observed_at_s,
            received_at_s,
            self.config.voice_timeout_s,
        ):
            return False
        if (
            observation.command.upper() == self._last_voice_command
            and observation.observed_at_s - self._last_voice_time
            < self.config.duplicate_voice_window_s
        ):
            return False
        if observation.observed_at_s < self._last_voice_time:
            return False
        self._voice = observation
        self._last_voice_command = observation.command.upper()
        self._last_voice_time = observation.observed_at_s
        return True

    def resolve(self, now_s: float) -> FusionDecision:
        if not math.isfinite(now_s):
            raise ValueError("fusion time must be finite")
        gesture = self._fresh_command(
            self._gesture,
            now_s,
            self.config.gesture_timeout_s,
            self.config.minimum_gesture_confidence,
        )
        voice = self._fresh_command(
            self._voice,
            now_s,
            self.config.voice_timeout_s,
            self.config.minimum_voice_confidence,
        )
        perception = self._fresh_perception(now_s)

        if gesture is not None and voice is not None:
            if gesture.command == voice.command:
                return self._decision(
                    gesture.command,
                    min(gesture.confidence, voice.confidence),
                    "FUSED",
                    "modalities_agree",
                    perception,
                )
            if gesture.command == "STAY" or voice.command == "STAY":
                stop = gesture if gesture.command == "STAY" else voice
                return self._decision(
                    "STAY",
                    stop.confidence,
                    stop.source,
                    "stay_safety_priority",
                    perception,
                )
            # A confident disagreement is not a neutral observation. NONE
            # deliberately leaves the active mission unchanged, so emitting it
            # here could let an already-moving rover continue. Convert every
            # conflict into an explicit, non-sequence-advancing safety hold.
            return self._decision(
                "STAY",
                min(gesture.confidence, voice.confidence),
                "FUSION",
                "conflict_safety_stop",
                perception,
                safety_stop=True,
            )
        if gesture is not None:
            return self._decision(
                gesture.command,
                gesture.confidence,
                "GESTURE",
                "gesture_selected",
                perception,
            )
        if voice is not None:
            return self._decision(
                voice.command,
                voice.confidence,
                "VOICE",
                "voice_fallback",
                perception,
            )
        return self._decision(
            "NONE", 0.0, "FUSION", "no_fresh_command", perception
        )

    @staticmethod
    def _structurally_valid(
        observation: CommandObservation,
        expected_source: str,
        allow_none: bool = False,
    ) -> bool:
        command = observation.command.strip().upper()
        return (
            observation.source.strip().upper() == expected_source
            and command in COMMANDS
            and (allow_none or command != "NONE")
            and math.isfinite(observation.confidence)
            and 0.0 <= observation.confidence <= 1.0
            and math.isfinite(observation.observed_at_s)
        )

    @staticmethod
    def _fresh_command(observation, now_s, timeout_s, minimum_confidence):
        if observation is None:
            return None
        age = now_s - observation.observed_at_s
        if age < 0.0 or age > timeout_s:
            return None
        if observation.confidence < minimum_confidence:
            return None
        return CommandObservation(
            command=observation.command.strip().upper(),
            confidence=observation.confidence,
            source=observation.source.strip().upper(),
            observed_at_s=observation.observed_at_s,
            transcript=observation.transcript,
        )

    def _fresh_perception(self, now_s):
        if self._gesture is None or self._gesture_perception is None:
            return None
        age = now_s - self._gesture.observed_at_s
        if age < 0.0 or age > self.config.depth_timeout_s:
            return None
        return self._gesture_perception

    def _age_is_valid(self, observed_at_s, received_at_s, timeout_s):
        if not math.isfinite(received_at_s):
            return False
        age = received_at_s - observed_at_s
        return -self.config.maximum_future_skew_s <= age <= timeout_s

    @staticmethod
    def _decision(
        command,
        confidence,
        source,
        reason,
        perception,
        *,
        safety_stop=False,
    ):
        return FusionDecision(
            command=command,
            confidence=float(confidence),
            source=source,
            reason=reason,
            perception=perception,
            safety_stop=bool(safety_stop),
        )
