#!/usr/bin/env python3

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

from .typing_mission_ros import quat_from_rpy


def build_parser():
    parser = argparse.ArgumentParser(
        description="Publish one high-confidence key pose for coordinator testing.",
    )
    parser.add_argument("--topic", default="/key_location")
    parser.add_argument("--frame", default="base_link")
    parser.add_argument("--x", type=float, default=0.35)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.25)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=math.pi)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--variance", type=float, default=0.0)
    parser.add_argument("--repeat", type=int, default=5)
    return parser


def main(args=None):
    parser = build_parser()
    parsed = parser.parse_args(args=args)

    rclpy.init()
    node = rclpy.create_node("manual_key_pose_pub")
    pub = node.create_publisher(PoseWithCovarianceStamped, parsed.topic, 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = parsed.frame
    msg.pose.pose.position.x = parsed.x
    msg.pose.pose.position.y = parsed.y
    msg.pose.pose.position.z = parsed.z
    qx, qy, qz, qw = quat_from_rpy(parsed.roll, parsed.pitch, parsed.yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.pose.covariance[0] = parsed.variance
    msg.pose.covariance[7] = parsed.variance
    msg.pose.covariance[14] = parsed.variance

    rate = node.create_rate(10)
    for _ in range(parsed.repeat):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        rate.sleep()

    node.get_logger().info(
        f"Published {parsed.repeat} key pose sample(s) on {parsed.topic} "
        f"in frame {parsed.frame}: x={parsed.x:.3f}, y={parsed.y:.3f}, z={parsed.z:.3f}"
    )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
