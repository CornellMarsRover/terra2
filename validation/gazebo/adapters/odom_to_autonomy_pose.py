#!/usr/bin/env python3
import math
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomToAutonomyPose(Node):
    def __init__(self):
        super().__init__('odom_to_autonomy_pose')
        self.output = self.create_publisher(
            TwistStamped, '/autonomy/pose/robot/global', 10)
        self.create_subscription(Odometry, '/drives/odom', self.convert, 10)

    def convert(self, msg):
        out = TwistStamped(header=msg.header)
        out.twist.linear.x = msg.pose.pose.position.x
        out.twist.linear.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        out.twist.angular.z = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        self.output.publish(out)


def main():
    rclpy.init()
    node = OdomToAutonomyPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
