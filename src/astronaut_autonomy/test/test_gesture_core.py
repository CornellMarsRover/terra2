import unittest

from astronaut_autonomy.gesture_core import (
    GestureResult,
    GestureStabilizer,
    classify_pose,
)


def pose_template():
    pose = [[0.0, 0.0, 0.0] for _ in range(17)]
    pose[0] = [50.0, 26.0, 0.9]
    pose[1] = [47.0, 24.0, 0.9]
    pose[2] = [53.0, 24.0, 0.9]
    pose[5] = [40.0, 40.0, 0.9]
    pose[6] = [60.0, 40.0, 0.9]
    pose[7] = [37.0, 58.0, 0.9]
    pose[8] = [63.0, 58.0, 0.9]
    pose[9] = [43.0, 72.0, 0.9]
    pose[10] = [57.0, 72.0, 0.9]
    pose[11] = [43.0, 80.0, 0.9]
    pose[12] = [57.0, 80.0, 0.9]
    return pose


class GestureCoreTests(unittest.TestCase):
    def test_give_crossed_forearms(self):
        pose = pose_template()
        pose[7] = [35.0, 60.0, 0.9]
        pose[8] = [65.0, 60.0, 0.9]
        pose[9] = [57.0, 44.0, 0.9]
        pose[10] = [43.0, 44.0, 0.9]
        self.assertEqual(classify_pose(pose).label, "GIVE")

    def test_fetch_can_point_left(self):
        pose = pose_template()
        pose[7] = [45.0, 42.0, 0.9]
        pose[9] = [50.0, 35.0, 0.9]
        pose[8] = [42.0, 55.0, 0.9]
        pose[10] = [20.0, 72.0, 0.9]
        result = classify_pose(pose)
        self.assertEqual(result.label, "FETCH")
        self.assertEqual(result.detail, "LEFT")

    def test_fetch_can_point_far_left_horizontally(self):
        pose = pose_template()
        pose[7] = [45.0, 42.0, 0.9]
        pose[9] = [50.0, 35.0, 0.9]
        pose[8] = [40.0, 40.0, 0.9]
        pose[10] = [10.0, 40.0, 0.9]
        result = classify_pose(pose)
        self.assertEqual(result.label, "FETCH")
        self.assertEqual(result.detail, "LEFT")

    def test_fetch_can_point_right(self):
        pose = pose_template()
        pose[7] = [45.0, 42.0, 0.9]
        pose[9] = [50.0, 35.0, 0.9]
        pose[8] = [76.0, 50.0, 0.9]
        pose[10] = [96.0, 64.0, 0.9]
        result = classify_pose(pose)
        self.assertEqual(result.label, "FETCH")
        self.assertEqual(result.detail, "RIGHT")

    def test_fetch_requires_left_hand_at_neck(self):
        pose = pose_template()
        pose[8] = [76.0, 50.0, 0.9]
        pose[10] = [96.0, 64.0, 0.9]
        self.assertNotEqual(classify_pose(pose).label, "FETCH")

    def test_come_allows_bent_elbows(self):
        pose = pose_template()
        pose[7] = [34.0, 42.0, 0.9]
        pose[8] = [66.0, 42.0, 0.9]
        pose[9] = [48.0, 15.0, 0.9]
        pose[10] = [52.0, 15.0, 0.9]
        self.assertEqual(classify_pose(pose).label, "COME")

    def test_follow_hands_on_hips(self):
        pose = pose_template()
        pose[7] = [20.0, 58.0, 0.9]
        pose[8] = [80.0, 58.0, 0.9]
        pose[9] = [43.0, 78.0, 0.9]
        pose[10] = [57.0, 78.0, 0.9]
        self.assertEqual(classify_pose(pose).label, "FOLLOW")

    def test_stay_t_pose(self):
        pose = pose_template()
        pose[7] = [20.0, 40.0, 0.9]
        pose[8] = [80.0, 40.0, 0.9]
        pose[9] = [0.0, 40.0, 0.9]
        pose[10] = [100.0, 40.0, 0.9]
        self.assertEqual(classify_pose(pose).label, "STAY")

    def test_low_confidence_pose_is_ignored(self):
        pose = pose_template()
        pose[5][2] = 0.1
        self.assertEqual(classify_pose(pose).label, "NONE")

    def test_stabilizer_requires_repeated_observations_and_clears(self):
        stabilizer = GestureStabilizer(confirmation_frames=3, clear_frames=2)
        gesture = GestureResult("FETCH", 0.8, "LEFT")
        self.assertEqual(stabilizer.update(gesture).label, "NONE")
        self.assertEqual(stabilizer.update(gesture).label, "NONE")
        stable = stabilizer.update(gesture)
        self.assertEqual(stable.label, "FETCH")
        self.assertEqual(stable.detail, "LEFT")
        self.assertEqual(
            stabilizer.update(GestureResult("NONE", 0.0)).label,
            "FETCH",
        )
        self.assertEqual(
            stabilizer.update(GestureResult("NONE", 0.0)).label,
            "NONE",
        )


if __name__ == "__main__":
    unittest.main()
