#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Point, Twist


class TestRealtimePlotterPublisher(Node):
    def __init__(self):
        super().__init__('test_realtime_plotter_publisher')

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/autonomy/pose/robot/global',
            10
        )
        self.target_pub = self.create_publisher(
            Point,
            '/autonomy/target_object/position',
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('radius', 3.0)
        self.declare_parameter('angular_speed', 0.25)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('target_x', 5.0)
        self.declare_parameter('target_y', 2.0)
        self.declare_parameter('frame_id', 'map')

        self.publish_rate = self.get_parameter('publish_rate').value
        self.radius = self.get_parameter('radius').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.frame_id = self.get_parameter('frame_id').value

        self.t = 0.0
        self.dt = 1.0 / float(self.publish_rate)

        timer_period = self.dt
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Test publisher started.')
        self.get_logger().info('Publishing fake robot pose, target point, and cmd_vel.')

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def timer_callback(self):
        # Circular trajectory
        x = self.center_x + self.radius * math.cos(self.angular_speed * self.t)
        y = self.center_y + self.radius * math.sin(self.angular_speed * self.t)

        # Velocity for circular motion
        vx = -self.radius * self.angular_speed * math.sin(self.angular_speed * self.t)
        vy = self.radius * self.angular_speed * math.cos(self.angular_speed * self.t)

        # Heading aligned with tangent of motion
        yaw = math.atan2(vy, vx)

        # ---------------- PoseStamped ----------------
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id

        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = 0.0

        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        # ---------------- Target Point ----------------
        target_msg = Point()
        target_msg.x = self.target_x
        target_msg.y = self.target_y
        target_msg.z = 0.0

        self.target_pub.publish(target_msg)

        # ---------------- cmd_vel ----------------
        cmd_msg = Twist()
        cmd_msg.linear.x = vx
        cmd_msg.linear.y = vy
        cmd_msg.linear.z = 0.0
        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = self.angular_speed

        self.cmd_pub.publish(cmd_msg)

        speed = math.sqrt(vx * vx + vy * vy)

        self.get_logger().info(
            f'Robot -> x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad | '
            f'Target -> x={self.target_x:.2f}, y={self.target_y:.2f} | '
            f'cmd_vel -> vx={vx:.2f}, vy={vy:.2f}, wz={self.angular_speed:.2f}, speed={speed:.2f}'
        )

        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = TestRealtimePlotterPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()