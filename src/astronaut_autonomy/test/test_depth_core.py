import math
import struct
import unittest

import numpy as np

from astronaut_autonomy.depth_core import (
    TargetEstimate,
    TargetTracker,
    command_requires_distance,
    estimate_target_from_xyz,
    horizontal_direction,
    map_image_point_to_depth,
    organized_xyz_from_buffer,
    safe_command_for,
    torso_sample_pixels,
)


def torso_pose():
    pose = [[0.0, 0.0, 0.0] for _ in range(17)]
    pose[5] = [30.0, 20.0, 0.9]
    pose[6] = [70.0, 20.0, 0.9]
    pose[11] = [35.0, 80.0, 0.9]
    pose[12] = [65.0, 80.0, 0.9]
    return pose


class DepthCoreTests(unittest.TestCase):
    def test_image_to_depth_mapping(self):
        self.assertEqual(
            map_image_point_to_depth(50, 25, (101, 51), (11, 6)),
            (5, 2),
        )

    def test_organized_buffer_decoder_honors_row_padding(self):
        data = bytearray(80)
        values = (
            (1.0, 2.0, 3.0),
            (4.0, 5.0, 6.0),
            (7.0, 8.0, 9.0),
            (10.0, 11.0, 12.0),
        )
        for index, value in enumerate(values):
            row, column = divmod(index, 2)
            struct.pack_into("<fff", data, row * 40 + column * 16, *value)
        xyz = organized_xyz_from_buffer(
            data,
            height=2,
            width=2,
            point_step=16,
            row_step=40,
            x_offset=0,
            y_offset=4,
            z_offset=8,
        )
        np.testing.assert_allclose(xyz[1, 1], values[3])

    def test_robust_torso_distance_rejects_outlier(self):
        pose = torso_pose()
        xyz = np.empty((10, 10, 3), dtype=np.float32)
        xyz[:] = (3.0, 1.0, 0.2)
        first_x, first_y = torso_sample_pixels(
            pose, (100, 100), (10, 10)
        )[0]
        xyz[first_y, first_x] = (14.0, -8.0, 6.0)
        estimate = estimate_target_from_xyz(pose, xyz, (100, 100))
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.position_m[0], 3.0)
        self.assertAlmostEqual(estimate.distance_m, math.sqrt(10.04), places=4)
        self.assertEqual(estimate.direction, "LEFT")

    def test_invalid_depth_returns_none(self):
        xyz = np.full((10, 10, 3), np.nan, dtype=np.float32)
        self.assertIsNone(
            estimate_target_from_xyz(torso_pose(), xyz, (100, 100))
        )

    def test_direction_uses_forward_left_up_camera_axes(self):
        self.assertEqual(horizontal_direction(3.0, 1.0)[0], "LEFT")
        self.assertEqual(horizontal_direction(3.0, -1.0)[0], "RIGHT")
        self.assertEqual(horizontal_direction(3.0, 0.1)[0], "CENTER")

    def test_only_follow_is_gated_by_astronaut_depth(self):
        self.assertTrue(command_requires_distance("FOLLOW"))
        self.assertFalse(command_requires_distance("FETCH"))
        self.assertFalse(command_requires_distance("COME"))
        self.assertEqual(safe_command_for("FOLLOW", False), "NONE")
        self.assertEqual(safe_command_for("FETCH", False), "FETCH")
        self.assertEqual(safe_command_for("COME", False), "COME")
        self.assertEqual(safe_command_for("STAY", False), "STAY")

    def test_tracker_smooths_and_marks_old_measurement_stale(self):
        tracker = TargetTracker(smoothing_alpha=0.5, stale_after_s=0.6)
        first = TargetEstimate((2.0, 0.0, 0.0), 2.0, "CENTER", 0.0, 20)
        second = TargetEstimate((4.0, 0.0, 0.0), 4.0, "CENTER", 0.0, 20)
        tracker.update(True, first, 1.0)
        tracked = tracker.update(True, second, 1.1)
        self.assertEqual(tracked.depth_status, "VALID")
        self.assertAlmostEqual(tracked.estimate.distance_m, 3.0)
        stale = tracker.update(True, None, 1.5)
        self.assertEqual(stale.depth_status, "STALE")
        expired = tracker.update(True, None, 1.8)
        self.assertEqual(expired.depth_status, "UNAVAILABLE")


if __name__ == "__main__":
    unittest.main()
