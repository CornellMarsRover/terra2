#!/usr/bin/env python3

import argparse
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Publish a fake high-confidence key pose near the current end effector "
            "for small arm coordinator tests."
        ),
    )
    parser.add_argument("--topic", default="/key_location")
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--ee-frame", default="end_effector")
    parser.add_argument("--x-offset", type=float, default=0.0)
    parser.add_argument("--y-offset", type=float, default=0.0)
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--variance", type=float, default=0.0)
    parser.add_argument("--repeat", type=int, default=20)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=8.0)
    return parser


def lookup_end_effector_transform(node, tf_buffer, target_frame, ee_frame, timeout):
    deadline = time.time() + timeout
    last_error = None

    while time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            return tf_buffer.lookup_transform(
                target_frame,
                ee_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            last_error = exc

    raise RuntimeError(
        f"Could not find TF {target_frame} -> {ee_frame} within {timeout:.1f}s"
        + (f": {last_error}" if last_error else "")
    )


def main(args=None):
    parsed = build_parser().parse_args(args=args)

    rclpy.init()
    node = rclpy.create_node("fake_near_ee_key_pose_pub")
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)
    pub = node.create_publisher(PoseWithCovarianceStamped, parsed.topic, 10)

    try:
        transform = lookup_end_effector_transform(
            node,
            tf_buffer,
            parsed.target_frame,
            parsed.ee_frame,
            parsed.timeout,
        )

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = parsed.target_frame
        msg.pose.pose.position.x = transform.transform.translation.x + parsed.x_offset
        msg.pose.pose.position.y = transform.transform.translation.y + parsed.y_offset
        msg.pose.pose.position.z = transform.transform.translation.z + parsed.z_offset
        msg.pose.pose.orientation = transform.transform.rotation
        msg.pose.covariance[0] = parsed.variance
        msg.pose.covariance[7] = parsed.variance
        msg.pose.covariance[14] = parsed.variance

        sleep_seconds = 1.0 / parsed.rate if parsed.rate > 0.0 else 0.05
        for _ in range(parsed.repeat):
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(sleep_seconds)

        node.get_logger().info(
            f"Published {parsed.repeat} fake key pose sample(s) on {parsed.topic} "
            f"in {parsed.target_frame}: "
            f"x={msg.pose.pose.position.x:.3f}, "
            f"y={msg.pose.pose.position.y:.3f}, "
            f"z={msg.pose.pose.position.z:.3f}"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
