#!/usr/bin/env python3

import argparse
import time

import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
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
            "Publish an absolute JointTrajectory directly to armnet's input topic. "
            "For a safe current-position-relative hardware nudge, use direct_moteus_nudge."
        ),
    )
    parser.add_argument("--topic", default="/main_arm_controller/joint_trajectory")
    parser.add_argument("--joint", choices=JOINT_NAMES, default="wrist_twist")
    parser.add_argument("--delta", type=float, default=0.02)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--return-home", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--allow-absolute",
        action="store_true",
        help="Actually publish absolute joint positions. This is not a relative nudge.",
    )
    return parser


def make_trajectory(joint, position, duration):
    msg = JointTrajectory()
    msg.joint_names = [joint]

    point = JointTrajectoryPoint()
    point.positions = [position]
    point.velocities = [0.05]
    point.time_from_start.sec = int(duration)
    point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
    msg.points = [point]
    return msg


def publish_repeated(node, publisher, msg, repeat):
    for _ in range(repeat):
        msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.1)


def main(args=None):
    parser = build_parser()
    parsed = parser.parse_args(args=args)
    if not parsed.allow_absolute:
        parser.error(
            "direct_joint_nudge_pub sends absolute joint targets. "
            "Use direct_moteus_nudge for a small relative hardware nudge, "
            "or pass --allow-absolute if you really want this ROS trajectory test."
        )

    rclpy.init()
    node = rclpy.create_node("direct_joint_nudge_pub")
    pub = node.create_publisher(JointTrajectory, parsed.topic, 10)

    try:
        target_msg = make_trajectory(parsed.joint, parsed.delta, parsed.duration)
        publish_repeated(node, pub, target_msg, parsed.repeat)
        node.get_logger().info(
            f"Published direct nudge on {parsed.topic}: "
            f"{parsed.joint} -> {parsed.delta:.3f} rad"
        )

        if parsed.return_home:
            time.sleep(parsed.settle)
            home_msg = make_trajectory(parsed.joint, 0.0, parsed.duration)
            publish_repeated(node, pub, home_msg, parsed.repeat)
            node.get_logger().info(f"Published {parsed.joint} return-to-zero trajectory.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
