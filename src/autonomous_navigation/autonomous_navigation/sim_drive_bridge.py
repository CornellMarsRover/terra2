#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from cmr_msgs.msg import AutonomyDrive


class SimDriveBridge(Node):
    """Bridge autonomy motion topics into Gazebo planar velocity commands."""

    def __init__(self) -> None:
        super().__init__("sim_drive_bridge")

        self.declare_parameter("twist_topic", "/drives/cmd_vel")
        self.declare_parameter("command_timeout_s", 0.4)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("wheelbase_m", 0.83)
        self.declare_parameter("velocity_scale", 10.0)
        self.declare_parameter("point_turn_scale", 10.0)
        self.declare_parameter("max_linear_speed", 0.9)
        self.declare_parameter("max_angular_speed", 1.2)
        self.declare_parameter("linear_accel_limit", 0.75)
        self.declare_parameter("angular_accel_limit", 1.2)

        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.velocity_scale = float(self.get_parameter("velocity_scale").value)
        self.point_turn_scale = float(self.get_parameter("point_turn_scale").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.linear_accel_limit = float(self.get_parameter("linear_accel_limit").value)
        self.angular_accel_limit = float(self.get_parameter("angular_accel_limit").value)

        self.drive_pub = self.create_publisher(Twist, self.twist_topic, 10)
        self.create_subscription(
            AutonomyDrive, "/autonomy/move/ackerman", self.ackermann_callback, 10
        )
        self.create_subscription(
            Twist, "/autonomy/move/point_turn", self.point_turn_callback, 10
        )

        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.last_command_time = self.get_clock().now()
        self.last_publish_time = self.get_clock().now()
        self.last_source = "startup"

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_cmd)

        self.get_logger().info(
            f"Sim drive bridge publishing to {self.twist_topic} "
            f"(velocity_scale={self.velocity_scale:.2f}, point_turn_scale={self.point_turn_scale:.2f})"
        )

    def ackermann_callback(self, msg: AutonomyDrive) -> None:
        steering_candidates = [msg.fl_angle, msg.fr_angle, msg.bl_angle, msg.br_angle]
        steering_angles = [angle for angle in steering_candidates if abs(angle) > 1e-3]
        steering_deg = sum(steering_angles) / len(steering_angles) if steering_angles else 0.0
        steering_rad = math.radians(steering_deg)

        linear_x = self._clamp(msg.vel * self.velocity_scale, self.max_linear_speed)
        angular_z = 0.0
        if abs(steering_rad) > 1e-3:
            angular_z = linear_x * math.tan(steering_rad) / self.wheelbase_m
            angular_z = self._clamp(angular_z, self.max_angular_speed)

        self._set_command(linear_x, angular_z, "/autonomy/move/ackerman")

    def point_turn_callback(self, msg: Twist) -> None:
        angular_z = self._clamp(msg.angular.z * self.point_turn_scale, self.max_angular_speed)
        self._set_command(0.0, angular_z, "/autonomy/move/point_turn")

    def _set_command(self, linear_x: float, angular_z: float, source: str) -> None:
        self.target_linear_x = linear_x
        self.target_angular_z = angular_z
        self.last_command_time = self.get_clock().now()
        self.last_source = source
        self.get_logger().info(
            f"sim_drive source={source} linear_x={linear_x:+.3f} angular_z={angular_z:+.3f}",
            throttle_duration_sec=1.0,
        )

    def publish_cmd(self) -> None:
        now = self.get_clock().now()
        age = (now - self.last_command_time).nanoseconds / 1e9
        dt = max((now - self.last_publish_time).nanoseconds / 1e9, 1e-3)
        self.last_publish_time = now

        if age <= self.command_timeout_s:
            self.current_linear_x = self._slew(
                self.current_linear_x, self.target_linear_x, self.linear_accel_limit, dt
            )
            self.current_angular_z = self._slew(
                self.current_angular_z, self.target_angular_z, self.angular_accel_limit, dt
            )
        else:
            self.current_linear_x = self._slew(self.current_linear_x, 0.0, self.linear_accel_limit, dt)
            self.current_angular_z = self._slew(
                self.current_angular_z, 0.0, self.angular_accel_limit, dt
            )

        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.angular.z = self.current_angular_z
        self.drive_pub.publish(twist)

    @staticmethod
    def _clamp(value: float, max_abs: float) -> float:
        return max(-max_abs, min(max_abs, float(value)))

    @staticmethod
    def _slew(current: float, target: float, rate_limit: float, dt: float) -> float:
        max_delta = max(rate_limit * dt, 0.0)
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimDriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
