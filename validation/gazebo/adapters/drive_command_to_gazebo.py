#!/usr/bin/env python3
import math
import rclpy
from cmr_msgs.msg import DriveCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node

class DriveCommandToGazebo(Node):
    """Convert the selected hardware drive contract to a planar Gazebo twist."""

    def __init__(self):
        super().__init__('drive_command_to_gazebo')
        self.declare_parameter('meters_per_rps', 0.2)
        self.scale = float(self.get_parameter('meters_per_rps').value)
        self.output = self.create_publisher(Twist, '/drives/cmd_vel', 10)
        self.create_subscription(DriveCommand, '/cmd_vel', self.convert, 10)

    def convert(self, msg):
        vx, vy, omega = msg.vx, msg.vy, msg.omega
        rot = omega / math.sqrt(2.0)
        norm = max(1.0, *(math.hypot(vy + sy*rot, vx + sx*rot)
                          for sx in (-1, 1) for sy in (-1, 1)))
        speed = msg.speed_rps * self.scale / norm
        out = Twist()
        out.linear.x, out.linear.y = speed * vx, speed * vy
        out.angular.z = speed * omega / math.hypot(0.415, 0.415)
        self.output.publish(out)
        self.get_logger().info(
            f'drive vx={msg.vx:.2f} vy={msg.vy:.2f} omega={msg.omega:.2f} '
            f'rps={msg.speed_rps:.2f} -> twist=({out.linear.x:.2f}, '
            f'{out.linear.y:.2f}, {out.angular.z:.2f})',
            throttle_duration_sec=1.0)

def main():
    rclpy.init()
    node = DriveCommandToGazebo()
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
