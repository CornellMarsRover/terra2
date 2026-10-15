import math
import unittest
from types import SimpleNamespace

import numpy as np

from astronaut_tracker import (
    AstronautTracker,
    estimate_torso_depth,
    horizontal_direction,
    map_video_point_to_depth,
)
from zed_depth import DepthFrame


def point(x, y, visibility=0.99):
    return SimpleNamespace(x=x, y=y, z=0.0, visibility=visibility)


def torso_pose():
    landmarks = [point(0.5, 0.5) for _ in range(33)]
    landmarks[11] = point(0.35, 0.25)
    landmarks[12] = point(0.65, 0.25)
    landmarks[23] = point(0.40, 0.75)
    landmarks[24] = point(0.60, 0.75)
    return SimpleNamespace(landmark=landmarks)


def depth_frame(x=0.0, y=0.0, z=4.0, valid=True, simulated=False):
    xyz = np.empty((60, 80, 3), dtype=np.float32)
    xyz[:, :, 0] = x
    xyz[:, :, 1] = y
    xyz[:, :, 2] = z
    if not valid:
        xyz[:] = np.nan
    return DepthFrame(
        image_bgr=np.zeros((60, 80, 3), dtype=np.uint8),
        xyz_m=xyz,
        confidence=np.zeros((60, 80), dtype=np.float32),
        timestamp=0.0,
        source_status="SIMULATED" if simulated else "SDK",
        simulated=simulated,
    )


class AstronautTrackerTests(unittest.TestCase):
    def test_video_to_depth_coordinate_mapping(self):
        self.assertEqual(
            map_video_point_to_depth(0, 0, (640, 480), (1280, 720)),
            (0, 0),
        )
        self.assertEqual(
            map_video_point_to_depth(639, 479, (640, 480), (1280, 720)),
            (1279, 719),
        )
        center_x, center_y = map_video_point_to_depth(
            319.5,
            239.5,
            (640, 480),
            (1280, 720),
        )
        self.assertAlmostEqual(center_x, 640, delta=1)
        self.assertAlmostEqual(center_y, 360, delta=1)

    def test_valid_depth_uses_median_and_rejects_bad_samples(self):
        frame = depth_frame(x=-1.0, y=0.2, z=4.0)
        frame.xyz_m[20:24, 30:34] = np.inf
        frame.xyz_m[30:34, 40:44, 2] = 100.0
        estimate = estimate_torso_depth(
            torso_pose().landmark,
            frame,
            video_size=(640, 480),
        )
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.position_m[0], -1.0, places=3)
        self.assertAlmostEqual(estimate.position_m[2], 4.0, places=3)
        self.assertAlmostEqual(estimate.distance_m, math.sqrt(17.04), places=3)
        self.assertEqual(estimate.direction, "LEFT")

    def test_invalid_depth_is_unavailable(self):
        estimate = estimate_torso_depth(
            torso_pose().landmark,
            depth_frame(valid=False),
            video_size=(640, 480),
        )
        self.assertIsNone(estimate)

    def test_low_confidence_depth_is_rejected(self):
        frame = depth_frame(z=3.0)
        frame.confidence[:] = 95.0
        estimate = estimate_torso_depth(
            torso_pose().landmark,
            frame,
            video_size=(640, 480),
            maximum_confidence=80.0,
        )
        self.assertIsNone(estimate)

    def test_horizontal_directions(self):
        self.assertEqual(horizontal_direction(-2.0, 5.0)[0], "LEFT")
        self.assertEqual(horizontal_direction(0.1, 5.0)[0], "CENTER")
        self.assertEqual(horizontal_direction(2.0, 5.0)[0], "RIGHT")

    def test_temporary_pose_and_depth_loss_become_stale(self):
        tracker = AstronautTracker(stale_after_s=1.0, pose_grace_s=0.6)
        valid = tracker.update(
            torso_pose(),
            depth_frame(z=3.0),
            (640, 480),
            now=10.0,
        )
        self.assertEqual(valid.depth_status, "VALID")

        missing_pose = tracker.update(None, None, (640, 480), now=10.2)
        self.assertTrue(missing_pose.astronaut_detected)
        self.assertEqual(missing_pose.depth_status, "STALE")
        self.assertIsNotNone(missing_pose.distance_m)

        recovered_pose_bad_depth = tracker.update(
            torso_pose(),
            depth_frame(valid=False),
            (640, 480),
            now=10.5,
        )
        self.assertEqual(recovered_pose_bad_depth.depth_status, "STALE")

        expired = tracker.update(
            torso_pose(),
            depth_frame(valid=False),
            (640, 480),
            now=11.2,
        )
        self.assertEqual(expired.depth_status, "UNAVAILABLE")
        self.assertIsNone(expired.distance_m)

    def test_mock_measurement_is_never_labeled_valid(self):
        tracker = AstronautTracker()
        result = tracker.update(
            torso_pose(),
            depth_frame(z=3.0, simulated=True),
            (640, 480),
            now=1.0,
        )
        self.assertEqual(result.depth_status, "SIMULATED")
        self.assertTrue(result.simulated)


if __name__ == "__main__":
    unittest.main()
