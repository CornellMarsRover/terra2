"""Closed-loop, hardware-independent URC astronaut mission simulation."""

from __future__ import annotations

import csv
import json
import math
import random
from dataclasses import asdict, dataclass, replace
from pathlib import Path

from astronaut_autonomy.command_fusion import (
    CommandFusion,
    CommandObservation,
)
from astronaut_autonomy.mission_core import (
    AstronautMissionStateMachine,
    ManipulationAction,
    ManipulationStatus,
    MissionConfig,
    MissionState,
    PerceptionSample,
)
from astronaut_autonomy.tracking_control import TrackingConfig, VelocityCommand


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class CompetitionSimulationConfig:
    time_step_s: float = 0.1
    maximum_duration_s: float = 90.0
    rover_maximum_speed_mps: float = 2.0
    rover_maximum_turn_rate_rad_s: float = 1.4
    initial_astronaut_x_m: float = 6.0
    initial_astronaut_y_m: float = 1.5
    astronaut_follow_speed_mps: float = 0.25
    follow_walk_duration_s: float = 8.0
    stay_duration_s: float = 2.0
    remote_command_distance_m: float = 21.0
    astronaut_relocation_speed_mps: float = 4.0
    out_of_order_duration_s: float = 0.8
    manipulation_duration_s: float = 0.75
    range_noise_std_m: float = 0.0
    bearing_noise_std_deg: float = 0.0
    random_seed: int = 2027
    inject_failures: bool = True
    minimum_safe_distance_m: float = 0.75
    command_input_mode: str = "gesture"

    def __post_init__(self):
        positive = (
            self.time_step_s,
            self.maximum_duration_s,
            self.rover_maximum_speed_mps,
            self.rover_maximum_turn_rate_rad_s,
            self.remote_command_distance_m,
            self.manipulation_duration_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError(
                "simulation timing, limits, and distances must be positive"
            )
        if self.remote_command_distance_m < 20.0:
            raise ValueError("competition FETCH/COME distance must be at least 20 m")
        if self.range_noise_std_m < 0.0 or self.bearing_noise_std_deg < 0.0:
            raise ValueError("noise standard deviations cannot be negative")
        if self.command_input_mode not in {
            "gesture",
            "voice",
            "matching",
            "conflict",
        }:
            raise ValueError(
                "command_input_mode must be gesture, voice, matching, or conflict"
            )


@dataclass(frozen=True)
class ScenarioObservation:
    elapsed_s: float
    phase: str
    command: str
    gesture: str
    confidence: float
    astronaut_detected: bool
    distance_valid: bool
    depth_status: str
    distance_m: float
    bearing_deg: float
    emergency_stop: bool

    def perception(self) -> PerceptionSample:
        return PerceptionSample(
            command=self.command,
            gesture=self.gesture,
            confidence=self.confidence,
            astronaut_detected=self.astronaut_detected,
            distance_valid=self.distance_valid,
            depth_status=self.depth_status,
            distance_m=self.distance_m,
            bearing_deg=self.bearing_deg,
        )


@dataclass
class SimulationReport:
    succeeded: bool
    completion_time_s: float
    recognized_commands: list[str]
    state_transitions: list[dict[str, object]]
    starting_distance_m: float
    final_distance_m: float
    follow_stop_distance_m: float | None
    remote_command_distance_m: float | None
    come_stop_distance_m: float | None
    safety_violations: list[str]
    injected_events: list[str]
    unexpected_commands_rejected: int
    trajectory_samples: int
    failure_reason: str = ""

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


class MockRover:
    """Applies normalized controller output to a planar rover model."""

    def __init__(
        self,
        maximum_speed_mps: float,
        maximum_turn_rate_rad_s: float,
    ):
        self.pose = Pose2D()
        self.maximum_speed_mps = maximum_speed_mps
        self.maximum_turn_rate_rad_s = maximum_turn_rate_rad_s

    def step(
        self,
        command: VelocityCommand,
        dt_s: float,
        *,
        emergency_stop: bool = False,
    ) -> tuple[float, float]:
        linear = 0.0 if emergency_stop else command.linear_x
        angular = 0.0 if emergency_stop else command.angular_z
        linear_mps = _clamp(linear, -1.0, 1.0) * self.maximum_speed_mps
        angular_rad_s = (
            _clamp(angular, -1.0, 1.0) * self.maximum_turn_rate_rad_s
        )
        self.pose.yaw_rad = _wrap_angle(
            self.pose.yaw_rad + angular_rad_s * dt_s
        )
        self.pose.x += math.cos(self.pose.yaw_rad) * linear_mps * dt_s
        self.pose.y += math.sin(self.pose.yaw_rad) * linear_mps * dt_s
        return linear_mps, angular_rad_s


class CompetitionScenario:
    """Owns world state and competition event sequencing, but no controller."""

    def __init__(self, config: CompetitionSimulationConfig | None = None):
        self.config = config or CompetitionSimulationConfig()
        self.rover = MockRover(
            self.config.rover_maximum_speed_mps,
            self.config.rover_maximum_turn_rate_rad_s,
        )
        self.astronaut = Pose2D(
            self.config.initial_astronaut_x_m,
            self.config.initial_astronaut_y_m,
        )
        self.phase = "FOLLOW"
        self.phase_started_s = 0.0
        self.started_distance_m = self.true_measurement()[0]
        self.follow_stop_distance_m = None
        self.remote_command_distance_m = None
        self.come_stop_distance_m = None
        self.unexpected_commands_rejected = 0
        self.injected_events: list[str] = []
        self.safety_violations: list[str] = []
        self.recognized_commands: list[str] = []
        self.state_transitions: list[dict[str, object]] = []
        self.trajectory: list[dict[str, object]] = []
        self._rng = random.Random(self.config.random_seed)
        self._stay_pose = None
        self._last_state = MissionState.IDLE
        self._operation = None
        self._operation_complete_s = None
        self.complete = False

    def true_measurement(self) -> tuple[float, float]:
        dx = self.astronaut.x - self.rover.pose.x
        dy = self.astronaut.y - self.rover.pose.y
        distance = math.hypot(dx, dy)
        bearing = math.degrees(
            _wrap_angle(math.atan2(dy, dx) - self.rover.pose.yaw_rad)
        )
        return distance, bearing

    def observation(self, now_s: float) -> ScenarioObservation:
        elapsed = now_s - self.phase_started_s
        distance, bearing = self.true_measurement()
        command = self.phase if self.phase in {
            "FOLLOW", "STAY", "FETCH", "COME", "GIVE"
        } else "STAY"
        gesture = command
        detected = True
        valid = True
        depth_status = "VALID"
        estop = False

        if self.phase == "OUT_OF_ORDER":
            command = gesture = "GIVE"
        elif self.phase == "FETCH":
            # FETCH can be issued with the astronaut over 20 m away. Command
            # recognition is independent of the arm's nearby-object ranging.
            valid = False
            depth_status = "UNAVAILABLE"
        elif self.phase == "COMPLETE":
            command = gesture = "NONE"
        elif self.phase == "COME" and self.config.inject_failures:
            if 1.0 <= elapsed < 1.3:
                command = gesture = "NONE"
                detected = False
                valid = False
                depth_status = "NO_POSE"
                self._record_event("temporary_tracking_loss")
            elif 2.0 <= elapsed < 2.3:
                valid = False
                depth_status = "STALE"
                self._record_event("stale_invalid_depth")
            elif 3.0 <= elapsed < 3.5:
                estop = True
                self._record_event("emergency_stop")

        if valid:
            distance += self._rng.gauss(0.0, self.config.range_noise_std_m)
            bearing += self._rng.gauss(0.0, self.config.bearing_noise_std_deg)
        else:
            distance = math.nan
            bearing = math.nan

        return ScenarioObservation(
            elapsed_s=elapsed,
            phase=self.phase,
            command=command,
            gesture=gesture,
            confidence=0.94 if command != "NONE" else 0.0,
            astronaut_detected=detected,
            distance_valid=valid,
            depth_status=depth_status,
            distance_m=distance,
            bearing_deg=bearing,
            emergency_stop=estop,
        )

    def advance_world(self, now_s: float, dt_s: float) -> None:
        elapsed = now_s - self.phase_started_s
        if self.phase == "FOLLOW" and elapsed < self.config.follow_walk_duration_s:
            self.astronaut.x += self.config.astronaut_follow_speed_mps * dt_s
        elif self.phase == "RELOCATE":
            distance, _ = self.true_measurement()
            if distance < self.config.remote_command_distance_m:
                self.astronaut.x += self.config.astronaut_relocation_speed_mps * dt_s

    def update_phase(
        self,
        now_s: float,
        state: MissionState,
        expected_command: str,
        mission_complete: bool,
    ) -> None:
        distance, _ = self.true_measurement()
        elapsed = now_s - self.phase_started_s
        if self.phase == "FOLLOW":
            if (
                elapsed >= self.config.follow_walk_duration_s
                and state
                in (
                    MissionState.FOLLOWING,
                    MissionState.STAYING,
                )
                and (
                    state == MissionState.FOLLOWING
                    or self.config.command_input_mode == "conflict"
                )
                and distance <= 3.0
            ):
                self.follow_stop_distance_m = distance
                self._set_phase("STAY", now_s)
        elif self.phase == "STAY":
            if state == MissionState.STAYING and self._stay_pose is None:
                self._stay_pose = Pose2D(
                    self.rover.pose.x,
                    self.rover.pose.y,
                    self.rover.pose.yaw_rad,
                )
                self.phase_started_s = now_s
            if (
                state == MissionState.STAYING
                and self._stay_pose is not None
                and elapsed >= self.config.stay_duration_s
            ):
                self._set_phase("RELOCATE", now_s)
        elif self.phase == "RELOCATE":
            if distance >= self.config.remote_command_distance_m:
                self.remote_command_distance_m = distance
                self._set_phase("OUT_OF_ORDER", now_s)
        elif self.phase == "OUT_OF_ORDER":
            if elapsed >= self.config.out_of_order_duration_s:
                if state == MissionState.GIVING:
                    self.safety_violations.append(
                        "out-of-order GIVE was incorrectly accepted"
                    )
                else:
                    self.unexpected_commands_rejected += 1
                self._set_phase("FETCH", now_s)
        elif self.phase == "FETCH":
            if expected_command == "COME" and state == MissionState.STAYING:
                self._set_phase("COME", now_s)
        elif self.phase == "COME":
            if expected_command == "GIVE":
                self.come_stop_distance_m = distance
                self._set_phase("GIVE", now_s)
        elif self.phase == "GIVE" and mission_complete:
            self.complete = True
            self._set_phase("COMPLETE", now_s)

    def note_state(self, now_s: float, state: MissionState, reason: str) -> None:
        if state == self._last_state:
            return
        self.state_transitions.append(
            {
                "time_s": round(now_s, 3),
                "from": self._last_state.value,
                "to": state.value,
                "reason": reason,
            }
        )
        self._last_state = state

    def note_command(self, command: str) -> None:
        if not self.recognized_commands or self.recognized_commands[-1] != command:
            self.recognized_commands.append(command)

    def start_manipulation(self, action: ManipulationAction, now_s: float) -> None:
        self._operation = action.value.removeprefix("START_")
        self._operation_complete_s = now_s + self.config.manipulation_duration_s

    def manipulation_status(self, now_s: float) -> ManipulationStatus | None:
        if self._operation is None:
            return None
        if now_s >= self._operation_complete_s:
            self._operation = None
            self._operation_complete_s = None
            return ManipulationStatus.SUCCEEDED
        return ManipulationStatus.RUNNING

    def cancel_manipulation(self) -> None:
        self._operation = None
        self._operation_complete_s = None

    def check_safety(
        self,
        observation: ScenarioObservation,
        command: VelocityCommand,
        state: MissionState,
    ) -> None:
        moving = not command.stopped
        if observation.emergency_stop and moving:
            self._violation_once("nonzero controller output during emergency stop")
        if (
            not observation.distance_valid
            or not observation.astronaut_detected
        ) and moving:
            self._violation_once("nonzero controller output with invalid perception")
        if state == MissionState.STAYING and moving:
            self._violation_once("nonzero controller output while STAYING")
        distance, _ = self.true_measurement()
        if moving and distance < self.config.minimum_safe_distance_m:
            self._violation_once("rover moved inside minimum safe distance")

    def check_stay_position(self) -> None:
        if self.phase != "STAY" or self._stay_pose is None:
            return
        displacement = math.hypot(
            self.rover.pose.x - self._stay_pose.x,
            self.rover.pose.y - self._stay_pose.y,
        )
        if displacement > 1e-6:
            self._violation_once("rover position changed during STAY")

    def log(
        self,
        now_s: float,
        observation: ScenarioObservation,
        state: MissionState,
        command: VelocityCommand,
        applied_linear_mps: float,
        applied_angular_rad_s: float,
    ) -> None:
        distance, bearing = self.true_measurement()
        self.trajectory.append(
            {
                "time_s": round(now_s, 3),
                "phase": observation.phase,
                "perception_command": observation.command,
                "mission_state": state.value,
                "rover_x_m": self.rover.pose.x,
                "rover_y_m": self.rover.pose.y,
                "rover_yaw_deg": math.degrees(self.rover.pose.yaw_rad),
                "astronaut_x_m": self.astronaut.x,
                "astronaut_y_m": self.astronaut.y,
                "distance_m": distance,
                "bearing_deg": bearing,
                "distance_valid": observation.distance_valid,
                "emergency_stop": observation.emergency_stop,
                "normalized_linear": command.linear_x,
                "normalized_angular": command.angular_z,
                "applied_linear_mps": applied_linear_mps,
                "applied_angular_rad_s": applied_angular_rad_s,
            }
        )

    def report(self, elapsed_s: float, failure_reason: str = "") -> SimulationReport:
        final_distance, _ = self.true_measurement()
        sequence_ok = self.recognized_commands == [
            "FOLLOW", "STAY", "FETCH", "COME", "GIVE"
        ]
        ranges_ok = (
            self.follow_stop_distance_m is not None
            and self.follow_stop_distance_m <= 3.0
            and self.remote_command_distance_m is not None
            and self.remote_command_distance_m >= 20.0
            and self.come_stop_distance_m is not None
        )
        return SimulationReport(
            succeeded=(
                self.complete
                and sequence_ok
                and ranges_ok
                and not self.safety_violations
                and not failure_reason
            ),
            completion_time_s=round(elapsed_s, 3),
            recognized_commands=list(self.recognized_commands),
            state_transitions=list(self.state_transitions),
            starting_distance_m=self.started_distance_m,
            final_distance_m=final_distance,
            follow_stop_distance_m=self.follow_stop_distance_m,
            remote_command_distance_m=self.remote_command_distance_m,
            come_stop_distance_m=self.come_stop_distance_m,
            safety_violations=list(self.safety_violations),
            injected_events=list(self.injected_events),
            unexpected_commands_rejected=self.unexpected_commands_rejected,
            trajectory_samples=len(self.trajectory),
            failure_reason=failure_reason,
        )

    def _set_phase(self, phase: str, now_s: float) -> None:
        self.phase = phase
        self.phase_started_s = now_s

    def _record_event(self, event: str) -> None:
        if event not in self.injected_events:
            self.injected_events.append(event)

    def _violation_once(self, violation: str) -> None:
        if violation not in self.safety_violations:
            self.safety_violations.append(violation)


def run_competition_simulation(
    config: CompetitionSimulationConfig | None = None,
    *,
    csv_path: str | Path | None = None,
    report_path: str | Path | None = None,
) -> SimulationReport:
    """Run the complete closed loop without importing ROS."""
    simulation_config = config or CompetitionSimulationConfig()
    scenario = CompetitionScenario(simulation_config)
    machine = AstronautMissionStateMachine(
        mission_config=MissionConfig(
            command_debounce_s=0.35,
            perception_timeout_s=0.75,
            manipulation_timeout_s=5.0,
            minimum_command_confidence=0.50,
            enforce_competition_sequence=True,
        ),
        tracking_config=TrackingConfig(
            follow_distance_m=2.0,
            come_distance_m=1.25,
            distance_deadband_m=0.15,
            maximum_linear=0.50,
            maximum_angular=0.60,
        ),
    )
    fusion = CommandFusion()
    now_s = 0.0
    failure_reason = ""

    while now_s <= simulation_config.maximum_duration_s and not scenario.complete:
        scenario.advance_world(now_s, simulation_config.time_step_s)
        observation = scenario.observation(now_s)
        fused_sample = _fuse_simulated_observation(
            fusion,
            observation,
            now_s,
            simulation_config.command_input_mode,
        )
        accepted = machine.observe_perception(fused_sample, now_s)
        if accepted:
            if not fused_sample.safety_stop:
                scenario.note_command(fused_sample.command)

        manipulation_status = scenario.manipulation_status(now_s)
        if manipulation_status is not None:
            machine.observe_manipulation(manipulation_status, now_s)

        decision = machine.tick(now_s, emergency_stop=observation.emergency_stop)
        scenario.note_state(now_s, decision.state, decision.reason)
        if decision.manipulation_action in (
            ManipulationAction.START_FETCH,
            ManipulationAction.START_GIVE,
        ):
            scenario.start_manipulation(decision.manipulation_action, now_s)
            machine.observe_manipulation(ManipulationStatus.RUNNING, now_s)
        elif decision.manipulation_action == ManipulationAction.CANCEL:
            scenario.cancel_manipulation()

        scenario.check_safety(observation, decision.velocity, decision.state)
        applied_linear, applied_angular = scenario.rover.step(
            decision.velocity,
            simulation_config.time_step_s,
            emergency_stop=observation.emergency_stop,
        )
        scenario.check_stay_position()
        scenario.log(
            now_s,
            observation,
            decision.state,
            decision.velocity,
            applied_linear,
            applied_angular,
        )
        scenario.update_phase(
            now_s,
            decision.state,
            decision.expected_command,
            decision.mission_complete,
        )
        if decision.state == MissionState.ERROR:
            failure_reason = decision.reason
            break
        now_s = round(now_s + simulation_config.time_step_s, 10)

    if not scenario.complete and not failure_reason:
        failure_reason = "simulation_timeout"
    report = scenario.report(now_s, failure_reason)
    if csv_path is not None:
        write_trajectory_csv(csv_path, scenario.trajectory)
    if report_path is not None:
        write_report_json(report_path, report)
    return report


def _fuse_simulated_observation(
    fusion: CommandFusion,
    observation: ScenarioObservation,
    now_s: float,
    mode: str,
) -> PerceptionSample:
    """Route deterministic simulation inputs through the real fusion core."""
    raw = observation.perception()
    depth_only = replace(raw, command="NONE", gesture="NONE", confidence=0.0)

    if mode in {"gesture", "matching", "conflict"}:
        fusion.update_gesture(raw, now_s, received_at_s=now_s)
    else:
        fusion.update_gesture(depth_only, now_s, received_at_s=now_s)

    voice_command = None
    if mode in {"voice", "matching"} and raw.command != "NONE":
        voice_command = raw.command
    elif (
        mode == "conflict"
        and observation.phase == "FOLLOW"
        and 7.0 <= observation.elapsed_s < 7.4
    ):
        voice_command = "COME"

    if voice_command is not None:
        fusion.update_voice(
            CommandObservation(
                command=voice_command,
                confidence=0.95,
                source="VOICE",
                observed_at_s=now_s,
                transcript=voice_command.lower(),
            ),
            received_at_s=now_s,
        )

    decision = fusion.resolve(now_s)
    depth = decision.perception
    return PerceptionSample(
        command=decision.command,
        gesture=depth.gesture if depth is not None else "NONE",
        confidence=decision.confidence,
        astronaut_detected=(
            depth.astronaut_detected if depth is not None else False
        ),
        distance_valid=depth.distance_valid if depth is not None else False,
        depth_status=depth.depth_status if depth is not None else "UNAVAILABLE",
        distance_m=depth.distance_m if depth is not None else math.nan,
        bearing_deg=depth.bearing_deg if depth is not None else math.nan,
        source=decision.source,
        decision_reason=decision.reason,
        safety_stop=decision.safety_stop,
    )


def write_trajectory_csv(path: str | Path, rows: list[dict[str, object]]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        output.write_text("", encoding="utf-8")
        return
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_report_json(path: str | Path, report: SimulationReport) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report.to_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
