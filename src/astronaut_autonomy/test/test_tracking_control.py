import unittest

from astronaut_autonomy.tracking_control import (
    TrackingConfig,
    tracking_velocity,
)


class TrackingControlTests(unittest.TestCase):
    def setUp(self):
        self.config = TrackingConfig(
            follow_distance_m=2.0,
            come_distance_m=1.0,
            distance_deadband_m=0.1,
            bearing_deadband_deg=4.0,
            rotate_in_place_deg=30.0,
            linear_gain=0.5,
            angular_gain=1.0,
            maximum_linear=0.4,
            maximum_angular=0.5,
        )

    def test_far_target_moves_forward_with_bounded_turn(self):
        command = tracking_velocity(4.0, 10.0, 2.0, self.config)
        self.assertGreater(command.linear_x, 0.0)
        self.assertLessEqual(command.linear_x, 0.4)
        self.assertGreater(command.angular_z, 0.0)
        self.assertLessEqual(command.angular_z, 0.5)

    def test_large_bearing_rotates_in_place(self):
        command = tracking_velocity(4.0, -45.0, 2.0, self.config)
        self.assertEqual(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)
        self.assertEqual(command.reason, "aligning")

    def test_target_distance_stops_forward_motion(self):
        command = tracking_velocity(2.05, 0.0, 2.0, self.config)
        self.assertTrue(command.stopped)
        self.assertEqual(command.reason, "at_standoff")

    def test_too_close_never_reverses(self):
        command = tracking_velocity(1.0, 0.0, 2.0, self.config)
        self.assertEqual(command.linear_x, 0.0)

    def test_invalid_perception_stops(self):
        command = tracking_velocity(
            4.0, 0.0, 2.0, self.config, perception_valid=False
        )
        self.assertTrue(command.stopped)
        self.assertEqual(command.reason, "invalid_perception")

    def test_emergency_stop_overrides_valid_target(self):
        command = tracking_velocity(
            4.0, 0.0, 2.0, self.config, emergency_stop=True
        )
        self.assertTrue(command.stopped)
        self.assertEqual(command.reason, "emergency_stop")

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            TrackingConfig(maximum_linear=1.1)


if __name__ == "__main__":
    unittest.main()
