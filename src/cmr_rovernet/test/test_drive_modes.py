import unittest

from cmr_rovernet.drive_modes import (
    DriveMode,
    drive_mode_for_dpad,
    parse_drive_mode,
    shape_motion_for_mode,
)


class DriveModeTests(unittest.TestCase):
    def test_cardinal_dpad_selects_each_drive_mode(self):
        self.assertIs(
            drive_mode_for_dpad(0),
            DriveMode.TRANSLATION_ROTATION,
        )
        self.assertIs(drive_mode_for_dpad(2), DriveMode.ACKERMANN)
        self.assertIs(drive_mode_for_dpad(4), DriveMode.POINT_TURN)
        self.assertIs(drive_mode_for_dpad(6), DriveMode.STEADY_HEADING)

    def test_neutral_diagonal_and_unknown_dpad_do_not_change_mode(self):
        for dpad in (1, 3, 5, 7, 8, 99):
            with self.subTest(dpad=dpad):
                self.assertIsNone(drive_mode_for_dpad(dpad))

    def test_translation_rotation_preserves_all_axes(self):
        self.assertEqual(
            shape_motion_for_mode("translation_rotation", 0.7, -0.3, 0.4),
            (0.7, -0.3, 0.4),
        )

    def test_steady_heading_removes_rotation_only(self):
        self.assertEqual(
            shape_motion_for_mode(DriveMode.STEADY_HEADING, 0.7, -0.3, 0.4),
            (0.7, -0.3, 0.0),
        )

    def test_point_turn_removes_translation_only(self):
        self.assertEqual(
            shape_motion_for_mode(DriveMode.POINT_TURN, 0.7, -0.3, 0.4),
            (0.0, 0.0, 0.4),
        )

    def test_ackermann_removes_strafe_and_prevents_stationary_spin(self):
        self.assertEqual(
            shape_motion_for_mode(DriveMode.ACKERMANN, 0.5, -0.3, 0.8),
            (0.5, 0.0, 0.4),
        )
        self.assertEqual(
            shape_motion_for_mode(DriveMode.ACKERMANN, 0.0, 0.0, 0.8),
            (0.0, 0.0, 0.0),
        )

    def test_invalid_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_drive_mode("tank")


if __name__ == "__main__":
    unittest.main()
