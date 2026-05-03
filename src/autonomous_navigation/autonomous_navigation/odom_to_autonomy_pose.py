#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomToAutonomyPose(Node):
    """Adapt Gazebo odometry into the autonomy pose topic."""

    def __init__(self) -> None:
        super().__init__("odom_to_autonomy_pose")
        self.pose_pub = self.create_publisher(TwistStamped, "/autonomy/pose/robot/global", 10)
        self.create_subscription(Odometry, "/drives/odom", self.odom_callback, 10)
        self.get_logger().info("Bridging /drives/odom -> /autonomy/pose/robot/global")

    def odom_callback(self, msg: Odometry) -> None:
        pose_msg = TwistStamped()
        pose_msg.header = msg.header
        pose_msg.twist.linear.x = msg.pose.pose.position.x
        pose_msg.twist.linear.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        pose_msg.twist.angular.z = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.pose_pub.publish(pose_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToAutonomyPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
