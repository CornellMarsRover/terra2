import math
import unittest

from astronaut_autonomy.mission_core import (
    AstronautMissionStateMachine,
    ManipulationAction,
    ManipulationStatus,
    MissionConfig,
    MissionState,
    PerceptionSample,
)
from astronaut_autonomy.tracking_control import TrackingConfig


def sample(
    command,
    *,
    valid=True,
    detected=True,
    distance=3.0,
    bearing=0.0,
    confidence=0.9,
):
    return PerceptionSample(
        command=command,
        gesture=command,
        confidence=confidence,
        astronaut_detected=detected,
        distance_valid=valid,
        depth_status="VALID" if valid else "UNAVAILABLE",
        distance_m=distance if valid else math.nan,
        bearing_deg=bearing if valid else math.nan,
    )


class MissionCoreTests(unittest.TestCase):
    def setUp(self):
        self.machine = AstronautMissionStateMachine(
            mission_config=MissionConfig(
                command_debounce_s=0.3,
                perception_timeout_s=0.6,
                manipulation_timeout_s=1.0,
                minimum_command_confidence=0.5,
            ),
            tracking_config=TrackingConfig(
                follow_distance_m=2.0,
                come_distance_m=1.0,
                maximum_linear=0.5,
                maximum_angular=0.5,
            ),
        )

    def accept(self, command, start=0.0, **kwargs):
        accepted_first = self.machine.observe_perception(
            sample(command, **kwargs), start
        )
        if command == "STAY":
            self.assertTrue(accepted_first)
            return start
        accepted_second = self.machine.observe_perception(
            sample(command, **kwargs), start + 0.31
        )
        self.assertFalse(accepted_first)
        self.assertTrue(accepted_second)
        return start + 0.31

    def neutral(self, now):
        self.machine.observe_perception(
            sample("NONE", valid=False, detected=False, confidence=0.0), now
        )

    def test_follow_stay_and_come_transitions(self):
        now = self.accept("FOLLOW", bearing=10.0)
        decision = self.machine.tick(now)
        self.assertEqual(decision.state, MissionState.FOLLOWING)
        self.assertGreater(decision.velocity.linear_x, 0.0)

        self.neutral(0.4)
        now = self.accept("STAY", start=0.5, valid=False)
        decision = self.machine.tick(now)
        self.assertEqual(decision.state, MissionState.STAYING)
        self.assertTrue(decision.velocity.stopped)

        self.neutral(0.9)
        now = self.accept("COME", start=1.0, distance=2.0, bearing=-8.0)
        decision = self.machine.tick(now)
        self.assertEqual(decision.state, MissionState.COMING)
        self.assertGreater(decision.velocity.linear_x, 0.0)
        self.assertLess(decision.velocity.angular_z, 0.0)

    def test_stay_does_not_require_depth(self):
        now = self.accept("STAY", valid=False, detected=False)
        decision = self.machine.tick(now)
        self.assertEqual(decision.state, MissionState.STAYING)
        self.assertTrue(decision.velocity.stopped)

    def test_none_is_neutral_but_invalid_depth_stops_drive(self):
        now = self.accept("FOLLOW")
        self.assertFalse(self.machine.tick(now).velocity.stopped)
        self.machine.observe_perception(
            sample("NONE", confidence=0.0, distance=3.0), 0.4
        )
        self.assertFalse(self.machine.tick(0.4).velocity.stopped)
        self.machine.observe_perception(
            sample("NONE", confidence=0.0, valid=False), 0.5
        )
        self.assertTrue(self.machine.tick(0.5).velocity.stopped)

    def test_fetch_command_is_independent_of_astronaut_depth(self):
        now = self.accept("FETCH", valid=False, detected=True)
        decision = self.machine.tick(now)
        self.assertEqual(decision.state, MissionState.FETCHING)
        self.assertEqual(
            decision.manipulation_action,
            ManipulationAction.START_FETCH,
        )

    def test_conflict_safety_stop_does_not_advance_sequence(self):
        machine = AstronautMissionStateMachine(
            mission_config=MissionConfig(
                command_debounce_s=0.1,
                perception_timeout_s=0.6,
                manipulation_timeout_s=1.0,
                enforce_competition_sequence=True,
            )
        )
        machine.observe_perception(sample("FOLLOW"), 0.0)
        self.assertTrue(machine.observe_perception(sample("FOLLOW"), 0.11))
        self.assertEqual(machine.expected_command, "STAY")

        conflict = PerceptionSample(
            command="STAY",
            confidence=0.9,
            astronaut_detected=True,
            distance_valid=True,
            depth_status="VALID",
            distance_m=3.0,
            bearing_deg=0.0,
            source="FUSION",
            decision_reason="conflict_safety_stop",
            safety_stop=True,
        )
        self.assertTrue(machine.observe_perception(conflict, 0.2))
        self.assertEqual(machine.state, MissionState.STAYING)
        self.assertEqual(machine.expected_command, "STAY")
        self.assertTrue(machine.tick(0.2).velocity.stopped)

        self.assertTrue(
            machine.observe_perception(sample("STAY", valid=False), 0.3)
        )
        self.assertEqual(machine.expected_command, "FETCH")

    def test_tracking_stops_immediately_on_invalid_depth_then_errors(self):
        now = self.accept("FOLLOW")
        self.machine.tick(now)
        self.machine.observe_perception(
            sample("NONE", valid=False, detected=True, confidence=0.0), 0.4
        )
        decision = self.machine.tick(0.4)
        self.assertEqual(decision.state, MissionState.FOLLOWING)
        self.assertTrue(decision.velocity.stopped)
        decision = self.machine.tick(1.0)
        self.assertEqual(decision.state, MissionState.ERROR)
        self.assertTrue(decision.velocity.stopped)

    def test_lost_tracking_times_out_safely(self):
        now = self.accept("COME")
        self.machine.tick(now)
        decision = self.machine.tick(now + 0.61)
        self.assertEqual(decision.state, MissionState.ERROR)
        self.assertTrue(decision.velocity.stopped)

    def test_low_confidence_command_is_not_accepted(self):
        self.machine.observe_perception(sample("FOLLOW", confidence=0.2), 0.0)
        accepted = self.machine.observe_perception(
            sample("FOLLOW", confidence=0.2), 1.0
        )
        self.assertFalse(accepted)
        self.assertEqual(self.machine.state, MissionState.IDLE)

    def test_same_held_fetch_command_only_starts_once(self):
        now = self.accept("FETCH")
        decision = self.machine.tick(now)
        self.assertEqual(decision.state, MissionState.FETCHING)
        self.assertEqual(
            decision.manipulation_action,
            ManipulationAction.START_FETCH,
        )
        self.machine.observe_manipulation(ManipulationStatus.RUNNING, 0.4)
        self.machine.observe_perception(sample("FETCH"), 0.7)
        self.machine.observe_perception(sample("FETCH"), 1.1)
        decision = self.machine.tick(1.1)
        self.assertEqual(decision.manipulation_action, ManipulationAction.NONE)

    def test_fetch_and_give_success_transitions(self):
        now = self.accept("FETCH")
        self.machine.tick(now)
        self.machine.observe_manipulation(ManipulationStatus.SUCCEEDED, 0.5)
        decision = self.machine.tick(0.5)
        self.assertEqual(decision.state, MissionState.STAYING)

        self.neutral(0.6)
        now = self.accept("GIVE", start=0.7, valid=False, detected=True)
        decision = self.machine.tick(now)
        self.assertEqual(decision.state, MissionState.GIVING)
        self.assertEqual(
            decision.manipulation_action,
            ManipulationAction.START_GIVE,
        )
        self.machine.observe_manipulation(ManipulationStatus.SUCCEEDED, 1.1)
        self.assertEqual(self.machine.tick(1.1).state, MissionState.STAYING)

    def test_manipulation_failure_enters_error(self):
        now = self.accept("GIVE", valid=False)
        self.machine.tick(now)
        self.machine.observe_manipulation(ManipulationStatus.FAILED, 0.5)
        decision = self.machine.tick(0.5)
        self.assertEqual(decision.state, MissionState.ERROR)
        self.assertTrue(decision.velocity.stopped)

    def test_manipulation_timeout_cancels_and_errors(self):
        now = self.accept("FETCH")
        self.machine.tick(now)
        decision = self.machine.tick(now + 1.01)
        self.assertEqual(decision.state, MissionState.ERROR)
        self.assertEqual(decision.manipulation_action, ManipulationAction.CANCEL)

    def test_stay_cancels_active_manipulation(self):
        now = self.accept("FETCH")
        self.machine.tick(now)
        self.neutral(0.4)
        now = self.accept("STAY", start=0.5, valid=False)
        decision = self.machine.tick(now)
        self.assertEqual(decision.state, MissionState.STAYING)
        self.assertEqual(decision.manipulation_action, ManipulationAction.CANCEL)

    def test_other_commands_do_not_interrupt_manipulation(self):
        now = self.accept("FETCH")
        self.machine.tick(now)
        self.neutral(0.4)
        self.machine.observe_perception(sample("COME"), 0.5)
        accepted = self.machine.observe_perception(sample("COME"), 0.9)
        self.assertFalse(accepted)
        self.assertEqual(self.machine.state, MissionState.FETCHING)

    def test_emergency_stop_cancels_manipulation_and_holds_stay(self):
        now = self.accept("GIVE", valid=False)
        self.machine.tick(now)
        decision = self.machine.tick(0.4, emergency_stop=True)
        self.assertEqual(decision.state, MissionState.STAYING)
        self.assertTrue(decision.velocity.stopped)
        self.assertEqual(decision.manipulation_action, ManipulationAction.CANCEL)
        self.assertEqual(self.machine.tick(0.5).state, MissionState.STAYING)

    def test_emergency_stop_prevents_follow_motion(self):
        now = self.accept("FOLLOW")
        decision = self.machine.tick(now, emergency_stop=True)
        self.assertTrue(decision.velocity.stopped)
        self.assertEqual(decision.state, MissionState.STAYING)

    def test_emergency_stop_does_not_clear_existing_error(self):
        now = self.accept("FOLLOW")
        self.machine.tick(now + 0.61)
        decision = self.machine.tick(now + 0.7, emergency_stop=True)
        self.assertEqual(decision.state, MissionState.ERROR)
        self.assertTrue(decision.velocity.stopped)

    def test_error_recovers_with_stay_command(self):
        now = self.accept("FOLLOW")
        self.machine.tick(now + 0.61)
        self.assertEqual(self.machine.state, MissionState.ERROR)
        self.neutral(1.0)
        now = self.accept("STAY", start=1.1, valid=False)
        self.assertEqual(self.machine.tick(now).state, MissionState.STAYING)

    def test_error_can_be_reset_explicitly(self):
        now = self.accept("FOLLOW")
        self.machine.tick(now + 0.61)
        self.assertTrue(self.machine.reset_error())
        self.assertEqual(self.machine.state, MissionState.IDLE)
        self.assertFalse(self.machine.reset_error())

    def test_competition_sequence_rejects_out_of_order_and_completes(self):
        machine = AstronautMissionStateMachine(
            mission_config=MissionConfig(
                command_debounce_s=0.1,
                perception_timeout_s=0.6,
                manipulation_timeout_s=1.0,
                enforce_competition_sequence=True,
            )
        )

        def observe_twice(command, start, **kwargs):
            first = machine.observe_perception(sample(command, **kwargs), start)
            if command == "STAY":
                return first
            return machine.observe_perception(
                sample(command, **kwargs), start + 0.11
            )

        self.assertEqual(machine.expected_command, "FOLLOW")
        self.assertTrue(observe_twice("FOLLOW", 0.0))
        self.assertEqual(machine.expected_command, "STAY")
        machine.observe_perception(sample("NONE", confidence=0.0), 0.12)
        self.assertTrue(observe_twice("STAY", 0.2, valid=False))
        self.assertEqual(machine.expected_command, "FETCH")

        machine.observe_perception(sample("NONE", confidence=0.0), 0.32)
        self.assertFalse(observe_twice("GIVE", 0.4, valid=False))
        self.assertEqual(machine.state, MissionState.STAYING)
        self.assertEqual(machine.expected_command, "FETCH")

        self.assertTrue(observe_twice("FETCH", 0.6))
        self.assertEqual(
            machine.tick(0.71).manipulation_action,
            ManipulationAction.START_FETCH,
        )
        machine.observe_manipulation(ManipulationStatus.SUCCEEDED, 0.8)
        self.assertEqual(machine.tick(0.8).state, MissionState.STAYING)
        self.assertEqual(machine.expected_command, "COME")

        machine.observe_perception(sample("NONE", confidence=0.0), 0.81)
        self.assertTrue(observe_twice("COME", 0.9, distance=1.25))
        decision = machine.tick(1.01)
        self.assertEqual(decision.expected_command, "GIVE")

        machine.observe_perception(sample("NONE", confidence=0.0), 1.02)
        self.assertTrue(observe_twice("GIVE", 1.1, valid=False))
        machine.tick(1.21)
        machine.observe_manipulation(ManipulationStatus.SUCCEEDED, 1.3)
        decision = machine.tick(1.3)
        self.assertTrue(decision.mission_complete)
        self.assertEqual(decision.expected_command, "COMPLETE")


if __name__ == "__main__":
    unittest.main()
