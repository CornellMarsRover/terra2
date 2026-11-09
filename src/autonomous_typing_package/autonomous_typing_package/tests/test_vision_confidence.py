"""Vision stability regression tests without a camera, ROS node, or arm driver."""
from collections import deque
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time

from autonomous_typing_package.arm_coordinator import ArmCoordinator, State
from autonomous_typing_package.pose_confidence import (
    MAX_XY_VARIANCE_M2, xy_variance_is_acceptable,
)
from autonomous_typing_package.typing_mission_ros import TypingMission


def make_mission():
    mission = SimpleNamespace(
        STABILITY_WINDOW=20, VARIANCE_THRESHOLD=MAX_XY_VARIANCE_M2,
        origin_history=deque(maxlen=20), reference_tvec=None,
        corner_tag_ids={1, 2, 3, 4}, key_positions={'C': {'x': 0, 'y': 0}},
        pose_locked=False, pub_pose=Mock(), get_logger=lambda: Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=1_000_000_000)),
    )
    for name in ('update_reference_frame', 'invalidate_reference', 'compute_origin_variance',
                 'is_confident', 'compute_key_pose_stamped', 'publish_key_pose'):
        setattr(mission, name, MethodType(getattr(TypingMission, name), mission))
    return mission


def update(mission, centre=(0.0, 0.0, 0.7)):
    return mission.update_reference_frame(
        np.array([[1], [2], [3], [4]]), np.tile(centre, (4, 1, 1))
    )


class VisionConfidenceTests(unittest.TestCase):
    def test_twenty_mm_is_standard_deviation_not_variance(self):
        self.assertAlmostEqual(MAX_XY_VARIANCE_M2, 0.0004)
        self.assertTrue(xy_variance_is_acceptable(0.020 ** 2, 0.020 ** 2))
        self.assertFalse(xy_variance_is_acceptable(0.021 ** 2, 0.0))
        self.assertFalse(xy_variance_is_acceptable(0.0, 0.021 ** 2))

    def test_invalid_covariance_is_rejected(self):
        for invalid in (float('nan'), float('inf'), -0.1):
            self.assertFalse(xy_variance_is_acceptable(invalid, 0.0))
            self.assertFalse(xy_variance_is_acceptable(0.0, invalid))

    def test_incomplete_window_does_not_publish(self):
        mission = make_mission()
        for _ in range(19):
            update(mission)
            mission.publish_key_pose('C')
        mission.pub_pose.publish.assert_not_called()
        self.assertFalse(mission.pose_locked)

    def test_sample_with_five_and_twelve_mm_variation_locks(self):
        mission = make_mission()
        for i in range(20):
            sign = 1 if i % 2 else -1
            update(mission, (sign * 0.005, sign * 0.012, 0.7))
        mission.publish_key_pose('C')
        mission.pub_pose.publish.assert_called_once()
        self.assertTrue(mission.pose_locked)
        msg = mission.pub_pose.publish.call_args.args[0]
        self.assertAlmostEqual(msg.pose.covariance[0], 0.005 ** 2)
        self.assertAlmostEqual(msg.pose.covariance[7], 0.012 ** 2)

    def test_unstable_window_does_not_publish(self):
        mission = make_mission()
        for i in range(20):
            update(mission, ((1 if i % 2 else -1) * 0.03, 0.0, 0.7))
        mission.publish_key_pose('C')
        mission.pub_pose.publish.assert_not_called()

    def test_missing_markers_invalidate_and_require_new_window(self):
        mission = make_mission()
        for _ in range(20):
            update(mission)
        self.assertFalse(mission.update_reference_frame(None, None))
        self.assertIsNone(mission.reference_tvec)
        self.assertEqual(len(mission.origin_history), 0)
        mission.publish_key_pose('C')
        for _ in range(19):
            update(mission)
            mission.publish_key_pose('C')
        mission.pub_pose.publish.assert_not_called()
        update(mission)
        mission.publish_key_pose('C')
        mission.pub_pose.publish.assert_called_once()

    def test_partial_duplicate_and_nonfinite_detections_invalidate(self):
        for ids, positions in (
            (np.array([[1], [2], [3]]), np.zeros((3, 1, 3))),
            (np.array([[1], [2], [3], [4], [4]]), np.zeros((5, 1, 3))),
            (np.array([[1], [2], [3], [4]]), np.full((4, 1, 3), np.nan)),
        ):
            mission = make_mission()
            update(mission)
            self.assertFalse(mission.update_reference_frame(ids, positions))
            self.assertIsNone(mission.reference_tvec)
            self.assertEqual(len(mission.origin_history), 0)

    def test_coordinator_uses_same_limit_and_rejects_unstable_input(self):
        self.assertEqual(ArmCoordinator.VARIANCE_THRESHOLD, TypingMission.VARIANCE_THRESHOLD)
        for variance, accepted in ((0.012 ** 2, True), (0.03 ** 2, False),
                                   (float('nan'), False), (-0.1, False)):
            node = SimpleNamespace(
                state=State.WAITING_FOR_LOCK, variance_threshold=MAX_XY_VARIANCE_M2,
                target_frame='base_link', locked_pose=None, get_logger=lambda: Mock(),
                _transition=Mock(),
            )
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'base_link'
            msg.pose.covariance[0] = variance
            msg.pose.covariance[7] = 0.0
            ArmCoordinator.key_callback(node, msg)
            self.assertEqual(node.locked_pose is not None, accepted)


if __name__ == '__main__':
    unittest.main()
