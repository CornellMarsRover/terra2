"""ROS-independent astronaut mission state machine."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from astronaut_autonomy.tracking_control import (
    TrackingConfig,
    VelocityCommand,
    stopped_command,
    tracking_velocity,
)


COMMANDS = frozenset(("FOLLOW", "STAY", "FETCH", "COME", "GIVE", "NONE"))
COMPETITION_SEQUENCE = ("FOLLOW", "STAY", "FETCH", "COME", "GIVE")


class MissionState(str, Enum):
    IDLE = "IDLE"
    FOLLOWING = "FOLLOWING"
    STAYING = "STAYING"
    FETCHING = "FETCHING"
    COMING = "COMING"
    GIVING = "GIVING"
    ERROR = "ERROR"


class ManipulationStatus(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class ManipulationAction(str, Enum):
    NONE = "NONE"
    START_FETCH = "START_FETCH"
    START_GIVE = "START_GIVE"
    CANCEL = "CANCEL"


@dataclass(frozen=True)
class PerceptionSample:
    command: str
    gesture: str = "NONE"
    confidence: float = 0.0
    astronaut_detected: bool = False
    distance_valid: bool = False
    depth_status: str = "UNAVAILABLE"
    distance_m: float = math.nan
    bearing_deg: float = math.nan

    @property
    def valid_target(self) -> bool:
        return (
            self.astronaut_detected
            and self.distance_valid
            and self.depth_status.upper() == "VALID"
            and math.isfinite(self.distance_m)
            and self.distance_m > 0.0
            and math.isfinite(self.bearing_deg)
        )


@dataclass(frozen=True)
class MissionConfig:
    command_debounce_s: float = 0.35
    perception_timeout_s: float = 0.75
    manipulation_timeout_s: float = 30.0
    minimum_command_confidence: float = 0.50
    enforce_competition_sequence: bool = False

    def __post_init__(self):
        positive = (
            self.command_debounce_s,
            self.perception_timeout_s,
            self.manipulation_timeout_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("mission timeouts must be finite and positive")
        if not 0.0 <= self.minimum_command_confidence <= 1.0:
            raise ValueError("minimum command confidence must be in [0, 1]")


@dataclass(frozen=True)
class MissionDecision:
    state: MissionState
    velocity: VelocityCommand
    manipulation_action: ManipulationAction = ManipulationAction.NONE
    reason: str = ""
    expected_command: str = ""
    mission_complete: bool = False


class AstronautMissionStateMachine:
    """Debounced mission transitions with fail-safe motion output."""

    _TRACKING_STATES = frozenset((MissionState.FOLLOWING, MissionState.COMING))
    _MANIPULATION_STATES = frozenset(
        (MissionState.FETCHING, MissionState.GIVING)
    )

    def __init__(
        self,
        mission_config: MissionConfig | None = None,
        tracking_config: TrackingConfig | None = None,
    ):
        self.config = mission_config or MissionConfig()
        self.tracking_config = tracking_config or TrackingConfig()
        self.state = MissionState.IDLE
        self.reason = "waiting_for_command"
        self._perception = None
        self._perception_time = None
        self._last_valid_target_time = None
        self._candidate = "NONE"
        self._candidate_since = None
        self._latched_command = None
        self._pending_action = ManipulationAction.NONE
        self._manipulation_started_at = None
        self._manipulation_status = ManipulationStatus.IDLE
        self._manipulation_status_time = None
        self._estop_seen = False
        self._sequence_index = 0
        self.mission_complete = False

    @property
    def expected_command(self) -> str:
        if not self.config.enforce_competition_sequence:
            return ""
        if self._sequence_index >= len(COMPETITION_SEQUENCE):
            return "COMPLETE"
        return COMPETITION_SEQUENCE[self._sequence_index]

    def observe_perception(self, sample: PerceptionSample, now_s: float) -> bool:
        """Consume one perception update and return whether a command changed state."""
        self._require_time(now_s)
        command = sample.command.upper()
        if command not in COMMANDS or sample.confidence < self.config.minimum_command_confidence:
            command = "NONE"

        self._perception = sample
        self._perception_time = float(now_s)
        if sample.valid_target:
            self._last_valid_target_time = float(now_s)

        if command == "NONE":
            self._candidate = "NONE"
            self._candidate_since = float(now_s)
            # A neutral observation rearms the previous command. It does not
            # stop a mission; STAY is the explicit stop command.
            self._latched_command = None
            return False

        if command != self._candidate:
            self._candidate = command
            self._candidate_since = float(now_s)
            return False
        if self._candidate_since is None:
            self._candidate_since = float(now_s)
            return False
        if now_s - self._candidate_since < self.config.command_debounce_s:
            return False
        if command == self._latched_command:
            return False
        if not self._can_accept(command):
            return False
        if not self._sequence_allows(command):
            self.reason = f"out_of_order_expected_{self.expected_command.lower()}"
            return False
        if command in ("FOLLOW", "COME", "FETCH") and not sample.valid_target:
            self.reason = f"{command.lower()}_waiting_for_valid_depth"
            return False

        self._accept(command, float(now_s))
        self._latched_command = command
        return True

    def observe_manipulation(
        self,
        status: ManipulationStatus | str,
        now_s: float,
    ) -> None:
        self._require_time(now_s)
        self._manipulation_status = ManipulationStatus(status)
        self._manipulation_status_time = float(now_s)

    def tick(self, now_s: float, emergency_stop: bool = False) -> MissionDecision:
        self._require_time(now_s)
        if emergency_stop:
            if not self._estop_seen:
                if self.state in self._MANIPULATION_STATES:
                    self._pending_action = ManipulationAction.CANCEL
                if self.state == MissionState.ERROR:
                    self.reason = "emergency_stop_error_latched"
                else:
                    self.state = MissionState.STAYING
                    self.reason = "emergency_stop"
                self._latched_command = None
                self._candidate = "NONE"
                self._candidate_since = float(now_s)
            self._estop_seen = True
            return self._decision(stopped_command("emergency_stop"))
        self._estop_seen = False

        if self.state in self._MANIPULATION_STATES:
            self._update_manipulation(float(now_s))
            return self._decision(stopped_command(self.reason))

        if self.state in self._TRACKING_STATES:
            velocity = self._tracking_decision(float(now_s))
            return self._decision(velocity)

        return self._decision(stopped_command(self.reason))

    def reset_error(self) -> bool:
        if self.state != MissionState.ERROR:
            return False
        self.state = MissionState.IDLE
        self.reason = "error_reset"
        self._latched_command = None
        self._candidate = "NONE"
        self._candidate_since = None
        self._manipulation_started_at = None
        self._manipulation_status = ManipulationStatus.IDLE
        return True

    def _can_accept(self, command: str) -> bool:
        if self.state == MissionState.ERROR:
            return command == "STAY"
        if self.state in self._MANIPULATION_STATES:
            return command == "STAY"
        return command in COMMANDS - {"NONE"}

    def _sequence_allows(self, command: str) -> bool:
        if not self.config.enforce_competition_sequence:
            return True
        # STAY is always honored as a safety stop. It advances competition
        # progress only when it is the expected command.
        return command == "STAY" or command == self.expected_command

    def _advance_sequence(self, completed_command: str) -> None:
        if (
            self.config.enforce_competition_sequence
            and self.expected_command == completed_command
        ):
            self._sequence_index += 1
            self.mission_complete = self._sequence_index >= len(
                COMPETITION_SEQUENCE
            )

    def _accept(self, command: str, now_s: float) -> None:
        if command == "STAY":
            if self.state in self._MANIPULATION_STATES:
                self._pending_action = ManipulationAction.CANCEL
            self.state = MissionState.STAYING
            self.reason = "stay_command"
            self._manipulation_started_at = None
            self._advance_sequence("STAY")
            return
        if command == "FOLLOW":
            self.state = MissionState.FOLLOWING
            self.reason = "follow_command"
            self._advance_sequence("FOLLOW")
            return
        if command == "COME":
            self.state = MissionState.COMING
            self.reason = "come_command"
            return
        if command == "FETCH":
            self.state = MissionState.FETCHING
            self.reason = "fetch_started"
            self._manipulation_started_at = now_s
            self._manipulation_status = ManipulationStatus.IDLE
            self._pending_action = ManipulationAction.START_FETCH
            return
        if command == "GIVE":
            self.state = MissionState.GIVING
            self.reason = "give_started"
            self._manipulation_started_at = now_s
            self._manipulation_status = ManipulationStatus.IDLE
            self._pending_action = ManipulationAction.START_GIVE

    def _tracking_decision(self, now_s: float) -> VelocityCommand:
        if self._perception is None or self._perception_time is None:
            return self._tracking_loss(now_s, "no_perception")
        age = now_s - self._perception_time
        if age < 0.0 or age > self.config.perception_timeout_s:
            return self._tracking_loss(now_s, "perception_timeout")
        if not self._perception.valid_target:
            return self._tracking_loss(now_s, "invalid_or_stale_depth")

        target_distance = (
            self.tracking_config.follow_distance_m
            if self.state == MissionState.FOLLOWING
            else self.tracking_config.come_distance_m
        )
        self.reason = "tracking"
        velocity = tracking_velocity(
            self._perception.distance_m,
            self._perception.bearing_deg,
            target_distance,
            self.tracking_config,
        )
        if self.state == MissionState.COMING and velocity.reason == "at_standoff":
            self._advance_sequence("COME")
        return velocity

    def _tracking_loss(self, now_s: float, reason: str) -> VelocityCommand:
        valid_age = (
            math.inf
            if self._last_valid_target_time is None
            else now_s - self._last_valid_target_time
        )
        self.reason = reason
        if valid_age > self.config.perception_timeout_s:
            self.state = MissionState.ERROR
            self.reason = f"{reason}_error"
        return stopped_command(reason)

    def _update_manipulation(self, now_s: float) -> None:
        started = self._manipulation_started_at
        if started is None:
            self._enter_error("manipulation_start_missing", cancel=False)
            return
        fresh_status = (
            self._manipulation_status_time is not None
            and self._manipulation_status_time >= started
        )
        if fresh_status and self._manipulation_status == ManipulationStatus.SUCCEEDED:
            completed_command = (
                "FETCH" if self.state == MissionState.FETCHING else "GIVE"
            )
            self.state = MissionState.STAYING
            self.reason = "manipulation_succeeded"
            self._manipulation_started_at = None
            self._advance_sequence(completed_command)
            return
        if fresh_status and self._manipulation_status in (
            ManipulationStatus.FAILED,
            ManipulationStatus.CANCELED,
        ):
            self._enter_error(
                f"manipulation_{self._manipulation_status.value.lower()}",
                cancel=False,
            )
            return
        if now_s - started > self.config.manipulation_timeout_s:
            self._enter_error("manipulation_timeout", cancel=True)
            return
        self.reason = "manipulation_running"

    def _enter_error(self, reason: str, *, cancel: bool) -> None:
        if cancel:
            self._pending_action = ManipulationAction.CANCEL
        self.state = MissionState.ERROR
        self.reason = reason
        self._manipulation_started_at = None

    def _decision(self, velocity: VelocityCommand) -> MissionDecision:
        action = self._pending_action
        self._pending_action = ManipulationAction.NONE
        return MissionDecision(
            state=self.state,
            velocity=velocity,
            manipulation_action=action,
            reason=self.reason,
            expected_command=self.expected_command,
            mission_complete=self.mission_complete,
        )

    @staticmethod
    def _require_time(now_s: float) -> None:
        if not math.isfinite(now_s):
            raise ValueError("time must be finite")
