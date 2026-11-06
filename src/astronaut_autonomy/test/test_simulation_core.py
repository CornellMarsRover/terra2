import unittest

from astronaut_autonomy.simulation_core import scripted_perception


class SimulationCoreTests(unittest.TestCase):
    def test_script_contains_all_five_commands(self):
        samples = [scripted_perception(index * 0.1) for index in range(150)]
        commands = {sample.command for sample in samples}
        self.assertTrue({"FOLLOW", "STAY", "FETCH", "COME", "GIVE"} <= commands)

    def test_script_contains_invalid_depth_and_lost_tracking(self):
        invalid = scripted_perception(4.5)
        lost = scripted_perception(5.5)
        self.assertFalse(invalid.distance_valid)
        self.assertTrue(invalid.astronaut_detected)
        self.assertFalse(lost.distance_valid)
        self.assertFalse(lost.astronaut_detected)

    def test_follow_target_moves_over_time(self):
        early = scripted_perception(1.1)
        late = scripted_perception(3.8)
        self.assertGreater(early.distance_m, late.distance_m)
        self.assertGreater(abs(early.bearing_deg), abs(late.bearing_deg))

    def test_script_repeats_commands_for_debouncing(self):
        self.assertEqual(scripted_perception(1.2).command, "FOLLOW")
        self.assertEqual(scripted_perception(3.7).command, "FOLLOW")


if __name__ == "__main__":
    unittest.main()
