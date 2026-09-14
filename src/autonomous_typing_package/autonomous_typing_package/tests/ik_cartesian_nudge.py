#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros


MAIN_ARM_JOINTS = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist_rotate",
    "wrist_twist",
    "wrist_rotate_two",
]


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Move the end effector a small Cartesian offset from its current pose "
            "using MoveIt IK, then publish the solved multi-joint trajectory."
        )
    )
    parser.add_argument("--group", default="main_arm")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--ee-frame", default="end_effector")
    parser.add_argument("--topic", default="/main_arm_controller/joint_trajectory")
    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.01)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--velocity", type=float, default=0.05)
    parser.add_argument("--max-joint-delta", type=float, default=0.05)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--joint-state-timeout", type=float, default=2.0)
    parser.add_argument("--tf-timeout", type=float, default=2.0)
    parser.add_argument("--ik-timeout", type=float, default=0.5)
    return parser


def seconds_to_duration(seconds):
    msg = Duration()
    msg.sec = int(seconds)
    msg.nanosec = int((seconds - int(seconds)) * 1e9)
    return msg


class IkCartesianNudge(Node):
    def __init__(self, args):
        super().__init__("ik_cartesian_nudge")
        self.args = args
        self.latest_joint_state = None
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )
        self.trajectory_pub = self.create_publisher(JointTrajectory, args.topic, 10)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _joint_state_callback(self, msg):
        if all(name in msg.name for name in MAIN_ARM_JOINTS):
            self.latest_joint_state = msg

    def wait_for_inputs(self):
        deadline = time.time() + self.args.joint_state_timeout
        while rclpy.ok() and self.latest_joint_state is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.latest_joint_state is None:
            raise RuntimeError(
                "No /joint_states containing all main arm joints. "
                "Start the robot state / joint state publisher before running IK nudge."
            )

        if not self.ik_client.wait_for_service(timeout_sec=self.args.ik_timeout):
            raise RuntimeError(
                "MoveIt /compute_ik service is not available. "
                "Start move_group before running IK nudge."
            )

    def current_ee_pose(self):
        deadline = time.time() + self.args.tf_timeout
        last_error = None
        transform = None
        while rclpy.ok() and time.time() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.args.base_frame,
                    self.args.ee_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
                break
            except Exception as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.05)

        if transform is None:
            raise RuntimeError(
                f"Could not look up {self.args.base_frame}->{self.args.ee_frame}: {last_error}"
            )

        pose = PoseStamped()
        pose.header.frame_id = self.args.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def target_pose(self):
        pose = self.current_ee_pose()
        pose.pose.position.x += self.args.dx
        pose.pose.position.y += self.args.dy
        pose.pose.position.z += self.args.dz
        return pose

    def solve_ik(self, pose):
        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self.args.group
        request.ik_request.ik_link_name = self.args.ee_frame
        request.ik_request.robot_state = RobotState(joint_state=self.latest_joint_state)
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout = seconds_to_duration(self.args.ik_timeout)

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.args.ik_timeout + 0.5)
        if not future.done() or future.result() is None:
            raise RuntimeError("MoveIt IK request timed out.")

        response = future.result()
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt IK failed with error code {response.error_code.val}.")

        solution = response.solution.joint_state
        return {
            name: solution.position[index]
            for index, name in enumerate(solution.name)
            if name in MAIN_ARM_JOINTS
        }

    def current_joint_positions(self):
        return {
            name: self.latest_joint_state.position[self.latest_joint_state.name.index(name)]
            for name in MAIN_ARM_JOINTS
        }

    def validate_joint_delta(self, joint_positions):
        current = self.current_joint_positions()
        deltas = {
            name: joint_positions[name] - current[name]
            for name in MAIN_ARM_JOINTS
        }
        max_name, max_delta = max(deltas.items(), key=lambda item: abs(item[1]))
        if abs(max_delta) > self.args.max_joint_delta:
            raise RuntimeError(
                "IK solution rejected: "
                f"{max_name} would move {max_delta:.4f} rad, "
                f"limit is {self.args.max_joint_delta:.4f} rad. "
                f"Deltas: {', '.join(f'{name}={delta:.4f}' for name, delta in deltas.items())}"
            )

    def publish_trajectory(self, joint_positions):
        missing = [name for name in MAIN_ARM_JOINTS if name not in joint_positions]
        if missing:
            raise RuntimeError(f"IK solution missing arm joints: {missing}")

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.base_frame
        msg.joint_names = MAIN_ARM_JOINTS

        point = JointTrajectoryPoint()
        point.positions = [joint_positions[name] for name in MAIN_ARM_JOINTS]
        point.velocities = [self.args.velocity] * len(MAIN_ARM_JOINTS)
        point.time_from_start = seconds_to_duration(self.args.duration)
        msg.points = [point]

        for _ in range(self.args.repeat):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.trajectory_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.1)

    def run(self):
        self.wait_for_inputs()
        target = self.target_pose()
        self.get_logger().info(
            "IK target: "
            f"x={target.pose.position.x:.4f} "
            f"y={target.pose.position.y:.4f} "
            f"z={target.pose.position.z:.4f} "
            f"offset=({self.args.dx:.4f}, {self.args.dy:.4f}, {self.args.dz:.4f})"
        )
        joint_positions = self.solve_ik(target)
        self.validate_joint_delta(joint_positions)
        self.get_logger().info(
            "IK solution: "
            + ", ".join(f"{name}={joint_positions[name]:.4f}" for name in MAIN_ARM_JOINTS)
        )
        self.publish_trajectory(joint_positions)
        self.get_logger().info(f"Published IK nudge trajectory on {self.args.topic}.")


def main(args=None):
    parsed = build_parser().parse_args(args=args)
    rclpy.init()
    node = IkCartesianNudge(parsed)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
