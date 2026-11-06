import math
import unittest

from astronaut_autonomy.command_fusion import (
    CommandFusion,
    CommandObservation,
    FusionConfig,
)
from astronaut_autonomy.mission_core import (
    AstronautMissionStateMachine,
    MissionConfig,
    MissionState,
    PerceptionSample,
)


def perception(
    command="NONE",
    *,
    confidence=0.0,
    valid=True,
    detected=True,
    distance=3.0,
    bearing=0.0,
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


def voice(command, at=0.0, confidence=0.9, source="VOICE"):
    return CommandObservation(
        command=command,
        confidence=confidence,
        source=source,
        observed_at_s=at,
        transcript=command.lower(),
    )


def fused_sample(decision):
    depth = decision.perception
    return PerceptionSample(
        command=decision.command,
        gesture=depth.gesture if depth else "NONE",
        confidence=decision.confidence,
        astronaut_detected=depth.astronaut_detected if depth else False,
        distance_valid=depth.distance_valid if depth else False,
        depth_status=depth.depth_status if depth else "UNAVAILABLE",
        distance_m=depth.distance_m if depth else math.nan,
        bearing_deg=depth.bearing_deg if depth else math.nan,
        source=decision.source,
        decision_reason=decision.reason,
        safety_stop=decision.safety_stop,
    )


class CommandFusionTests(unittest.TestCase):
    def setUp(self):
        self.fusion = CommandFusion(
            FusionConfig(
                gesture_timeout_s=0.75,
                voice_timeout_s=1.25,
                depth_timeout_s=0.75,
                duplicate_voice_window_s=1.5,
            )
        )

    def test_voice_is_fallback_when_gesture_is_neutral(self):
        self.fusion.update_gesture(perception(valid=True), 1.0)
        self.assertTrue(self.fusion.update_voice(voice("COME", at=1.0)))
        decision = self.fusion.resolve(1.1)
        self.assertEqual(decision.command, "COME")
        self.assertEqual(decision.source, "VOICE")
        self.assertTrue(decision.perception.valid_target)

    def test_conflicting_follow_and_voice_produce_safety_stop(self):
        self.fusion.update_gesture(
            perception("FOLLOW", confidence=0.9),
            2.0,
        )
        self.fusion.update_voice(voice("COME", at=2.0))
        decision = self.fusion.resolve(2.1)
        self.assertEqual(decision.command, "STAY")
        self.assertEqual(decision.reason, "conflict_safety_stop")
        self.assertTrue(decision.safety_stop)

    def test_stay_has_safety_priority_from_either_modality(self):
        self.fusion.update_gesture(
            perception("FETCH", confidence=0.9),
            3.0,
        )
        self.fusion.update_voice(voice("STAY", at=3.0))
        decision = self.fusion.resolve(3.1)
        self.assertEqual(decision.command, "STAY")
        self.assertEqual(decision.reason, "stay_safety_priority")

    def test_non_follow_conflict_is_safety_stop(self):
        self.fusion.update_gesture(
            perception("FETCH", confidence=0.9),
            4.0,
        )
        self.fusion.update_voice(voice("GIVE", at=4.0))
        decision = self.fusion.resolve(4.1)
        self.assertEqual(decision.command, "STAY")
        self.assertEqual(decision.reason, "conflict_safety_stop")
        self.assertTrue(decision.safety_stop)

    def test_agreement_is_fused(self):
        self.fusion.update_gesture(perception("COME", confidence=0.8), 5.0)
        self.fusion.update_voice(voice("COME", at=5.0, confidence=0.9))
        decision = self.fusion.resolve(5.1)
        self.assertEqual(decision.command, "COME")
        self.assertEqual(decision.source, "FUSED")
        self.assertAlmostEqual(decision.confidence, 0.8)

    def test_low_confidence_and_stale_observations_are_rejected(self):
        self.fusion.update_gesture(perception(valid=True), 6.0)
        self.fusion.update_voice(voice("FETCH", at=6.0, confidence=0.2))
        self.assertEqual(self.fusion.resolve(6.1).command, "NONE")
        self.fusion.update_voice(voice("COME", at=8.0))
        self.assertEqual(self.fusion.resolve(9.3).command, "NONE")

    def test_invalid_or_missing_source_is_rejected(self):
        self.assertFalse(self.fusion.update_voice(voice("COME", source="")))
        self.assertFalse(
            self.fusion.update_voice(voice("COME", source="GESTURE"))
        )
        self.assertEqual(self.fusion.resolve(0.1).command, "NONE")

    def test_stale_and_future_dated_inputs_are_rejected_on_arrival(self):
        self.assertFalse(
            self.fusion.update_voice(
                voice("COME", at=1.0),
                received_at_s=3.0,
            )
        )
        self.assertFalse(
            self.fusion.update_voice(
                voice("COME", at=4.0),
                received_at_s=3.0,
            )
        )

    def test_duplicate_voice_does_not_extend_freshness(self):
        self.assertTrue(self.fusion.update_voice(voice("COME", at=10.0)))
        self.assertFalse(self.fusion.update_voice(voice("COME", at=10.5)))
        self.assertEqual(self.fusion.resolve(11.3).command, "NONE")

    def test_temporary_failure_recovers_with_new_gesture(self):
        self.fusion.update_gesture(perception("FOLLOW", confidence=0.9), 12.0)
        self.assertEqual(self.fusion.resolve(12.1).command, "FOLLOW")
        self.assertEqual(self.fusion.resolve(13.0).command, "NONE")
        self.fusion.update_gesture(perception("STAY", confidence=0.9), 13.1)
        self.assertEqual(self.fusion.resolve(13.2).command, "STAY")


class FusedMissionIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.fusion = CommandFusion()
        self.mission = AstronautMissionStateMachine(
            mission_config=MissionConfig(
                command_debounce_s=0.30,
                perception_timeout_s=0.60,
                manipulation_timeout_s=1.0,
                minimum_command_confidence=0.50,
            )
        )

    def observe(self, now):
        decision = self.fusion.resolve(now)
        return self.mission.observe_perception(fused_sample(decision), now)

    def test_voice_follow_uses_gesture_depth_and_does_not_double_trigger(self):
        self.fusion.update_gesture(perception(valid=True), 0.01)
        self.fusion.update_voice(voice("FOLLOW", at=0.01))
        self.assertFalse(self.observe(0.01))
        self.assertTrue(self.observe(0.32))
        self.assertEqual(self.mission.tick(0.32).state, MissionState.FOLLOWING)
        self.assertFalse(self.observe(0.50))

    def test_voice_movement_with_invalid_depth_is_rejected(self):
        self.fusion.update_gesture(
            perception(valid=False, detected=True),
            1.0,
        )
        self.fusion.update_voice(voice("COME", at=1.0))
        self.observe(1.0)
        self.assertFalse(self.observe(1.31))
        self.assertEqual(self.mission.state, MissionState.IDLE)

    def test_switch_to_voice_stay_and_emergency_stop_remain_safe(self):
        self.fusion.update_gesture(perception("FOLLOW", confidence=0.9), 2.0)
        self.observe(2.0)
        self.assertTrue(self.observe(2.31))
        self.assertEqual(self.mission.tick(2.31).state, MissionState.FOLLOWING)

        self.fusion.update_voice(voice("STAY", at=2.4))
        self.assertTrue(self.observe(2.4))
        self.assertEqual(self.mission.tick(2.4).state, MissionState.STAYING)
        stopped = self.mission.tick(2.8, emergency_stop=True)
        self.assertTrue(stopped.velocity.stopped)
        self.assertEqual(stopped.state, MissionState.STAYING)

    def test_conflict_stops_existing_motion_immediately(self):
        self.fusion.update_gesture(perception("FOLLOW", confidence=0.9), 3.0)
        self.observe(3.0)
        self.assertTrue(self.observe(3.31))
        moving = self.mission.tick(3.31)
        self.assertFalse(moving.velocity.stopped)

        self.fusion.update_voice(voice("COME", at=3.4))
        self.assertTrue(self.observe(3.4))
        stopped = self.mission.tick(3.4)
        self.assertEqual(stopped.state, MissionState.STAYING)
        self.assertTrue(stopped.velocity.stopped)
        self.assertEqual(stopped.reason, "conflict_safety_stop")

    def test_remote_fetch_does_not_require_astronaut_depth(self):
        self.fusion.update_gesture(
            perception(valid=False, detected=True),
            4.0,
        )
        self.fusion.update_voice(voice("FETCH", at=4.0))
        self.assertFalse(self.observe(4.0))
        self.assertTrue(self.observe(4.31))
        decision = self.mission.tick(4.31)
        self.assertEqual(decision.state, MissionState.FETCHING)

    def test_loss_of_perception_stops_then_times_out(self):
        self.fusion.update_gesture(perception("FOLLOW", confidence=0.9), 5.0)
        self.observe(5.0)
        self.assertTrue(self.observe(5.31))
        self.assertFalse(self.mission.tick(5.31).velocity.stopped)

        self.fusion.update_gesture(
            perception(valid=False, detected=False),
            5.4,
        )
        self.observe(5.4)
        self.assertTrue(self.mission.tick(5.4).velocity.stopped)
        expired = self.mission.tick(6.01)
        self.assertEqual(expired.state, MissionState.ERROR)
        self.assertTrue(expired.velocity.stopped)


if __name__ == "__main__":
    unittest.main()
