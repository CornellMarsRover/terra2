import unittest

from astronaut_autonomy.mission_core import (
    AstronautMissionStateMachine,
    ManipulationAction,
    ManipulationStatus,
    MissionConfig,
    MissionState,
    PerceptionSample,
)
from astronaut_autonomy.simulation_core import scripted_perception


class MissionSimulationIntegrationTests(unittest.TestCase):
    def test_complete_script_is_safe_and_reaches_each_mission_state(self):
        machine = AstronautMissionStateMachine(
            mission_config=MissionConfig(
                command_debounce_s=0.3,
                perception_timeout_s=0.6,
                manipulation_timeout_s=2.0,
            )
        )
        states = {machine.state}
        action_counts = {
            ManipulationAction.START_FETCH: 0,
            ManipulationAction.START_GIVE: 0,
        }
        operation_completion = None

        for step in range(150):
            now = step * 0.1
            simulated = scripted_perception(now)
            sample = PerceptionSample(
                command=simulated.command,
                gesture=simulated.gesture,
                confidence=simulated.confidence,
                astronaut_detected=simulated.astronaut_detected,
                distance_valid=simulated.distance_valid,
                depth_status=simulated.depth_status,
                distance_m=simulated.distance_m,
                bearing_deg=simulated.bearing_deg,
            )
            machine.observe_perception(sample, now)
            if operation_completion is not None and now >= operation_completion:
                machine.observe_manipulation(ManipulationStatus.SUCCEEDED, now)
                operation_completion = None
            decision = machine.tick(now)
            states.add(decision.state)

            if decision.manipulation_action in action_counts:
                action_counts[decision.manipulation_action] += 1
                machine.observe_manipulation(ManipulationStatus.RUNNING, now)
                operation_completion = now + 0.4

            if not simulated.distance_valid or not simulated.astronaut_detected:
                self.assertTrue(decision.velocity.stopped)

        expected = {
            MissionState.IDLE,
            MissionState.FOLLOWING,
            MissionState.STAYING,
            MissionState.FETCHING,
            MissionState.COMING,
            MissionState.GIVING,
            MissionState.ERROR,
        }
        self.assertEqual(states, expected)
        self.assertEqual(action_counts[ManipulationAction.START_FETCH], 1)
        self.assertEqual(action_counts[ManipulationAction.START_GIVE], 1)


if __name__ == "__main__":
    unittest.main()
