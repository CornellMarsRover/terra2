import rclpy
import pytest
from geometry_msgs.msg import PoseStamped
from types import SimpleNamespace

from autonomous_typing_package.arm_coordinator import ArmCoordinator


def setup_module():
    if not rclpy.ok():
        rclpy.init()


def teardown_module():
    if rclpy.ok():
        rclpy.shutdown()


def make_locked_pose(z=0.25):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = 0.35
    pose.pose.position.y = 0.0
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose


def test_preapproach_pose_offsets_above_locked_pose():
    node = ArmCoordinator()
    try:
        node.locked_pose = make_locked_pose()
        pose = node._get_preapproach_pose()
        assert pose.header.frame_id == "base_link"
        assert pose.pose.position.z == pytest.approx(0.30)
    finally:
        node.destroy_node()


def test_press_pose_offsets_below_locked_pose():
    node = ArmCoordinator()
    try:
        node.locked_pose = make_locked_pose()
        pose = node._get_press_pose()
        assert pose.header.frame_id == "base_link"
        assert pose.pose.position.z == pytest.approx(0.247)
    finally:
        node.destroy_node()


def test_servo_twist_points_from_current_position_to_target():
    node = ArmCoordinator()
    try:
        target = make_locked_pose(z=0.25)
        target.pose.position.x = 0.40
        target.pose.position.y = -0.10
        current = SimpleNamespace(x=0.35, y=0.0, z=0.20)

        twist = node._make_servo_twist(target, current)

        assert twist.header.frame_id == "base_link"
        assert twist.twist.linear.x > 0.0
        assert twist.twist.linear.y < 0.0
        assert twist.twist.linear.z > 0.0
    finally:
        node.destroy_node()


def test_servo_twist_is_bounded():
    node = ArmCoordinator()
    try:
        target = make_locked_pose(z=2.0)
        target.pose.position.x = 10.0
        target.pose.position.y = -10.0
        current = SimpleNamespace(x=0.0, y=0.0, z=0.0)

        twist = node._make_servo_twist(target, current)

        assert twist.twist.linear.x == pytest.approx(node.max_linear_command)
        assert twist.twist.linear.y == pytest.approx(-node.max_linear_command)
        assert twist.twist.linear.z == pytest.approx(node.max_linear_command)
    finally:
        node.destroy_node()
