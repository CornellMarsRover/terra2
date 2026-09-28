import unittest

from cmr_zed.gesture_core import GestureResult, GestureStabilizer, classify_pose


def pose_template():
    pose = [[0.0, 0.0, 0.0] for _ in range(17)]
    pose[5] = [40.0, 50.0, 0.9]
    pose[6] = [60.0, 50.0, 0.9]
    pose[7] = [40.0, 70.0, 0.9]
    pose[8] = [60.0, 70.0, 0.9]
    pose[9] = [40.0, 90.0, 0.9]
    pose[10] = [60.0, 90.0, 0.9]
    return pose


class GestureCoreTests(unittest.TestCase):
    def test_hands_up(self):
        pose = pose_template()
        pose[7] = [40.0, 35.0, 0.8]
        pose[8] = [60.0, 35.0, 0.8]
        pose[9] = [40.0, 20.0, 0.8]
        pose[10] = [60.0, 20.0, 0.8]
        self.assertEqual(classify_pose(pose).label, "hands_up")

    def test_left_arm_out(self):
        pose = pose_template()
        pose[7] = [25.0, 50.0, 0.8]
        pose[9] = [5.0, 50.0, 0.8]
        self.assertEqual(classify_pose(pose).label, "left_arm_out")

    def test_right_arm_out(self):
        pose = pose_template()
        pose[8] = [75.0, 50.0, 0.8]
        pose[10] = [95.0, 50.0, 0.8]
        self.assertEqual(classify_pose(pose).label, "right_arm_out")

    def test_both_arms_out(self):
        pose = pose_template()
        pose[7] = [25.0, 50.0, 0.8]
        pose[9] = [5.0, 50.0, 0.8]
        pose[8] = [75.0, 50.0, 0.8]
        pose[10] = [95.0, 50.0, 0.8]
        self.assertEqual(classify_pose(pose).label, "both_arms_out")

    def test_low_confidence_pose_is_ignored(self):
        pose = pose_template()
        pose[5][2] = 0.1
        self.assertEqual(classify_pose(pose).label, "none")

    def test_stabilizer_requires_repeated_observations_and_clears(self):
        stabilizer = GestureStabilizer(confirmation_frames=3, clear_frames=2)
        gesture = GestureResult("hands_up", 0.8)
        self.assertEqual(stabilizer.update(gesture).label, "none")
        self.assertEqual(stabilizer.update(gesture).label, "none")
        self.assertEqual(stabilizer.update(gesture).label, "hands_up")
        self.assertEqual(
            stabilizer.update(GestureResult("none", 0.0)).label,
            "hands_up",
        )
        self.assertEqual(
            stabilizer.update(GestureResult("none", 0.0)).label,
            "none",
        )


if __name__ == "__main__":
    unittest.main()
