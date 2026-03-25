#!/usr/bin/env python3

import math
from typing import Iterable

import rclpy
from cmr_msgs.msg import AutonomyDrive
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class GazeboDriveBridge(Node):
    """Mirror rover drive commands into Gazebo.

    The bridge consumes the same drive topics as the rover stack and republishes
    them in a Gazebo-friendly form. By default it targets the planar move plugin
    on /gazebo_cmd_vel, with the older joint-effort mode retained as a fallback.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_drive_bridge")

        self.declare_parameter(
            "left_joints",
            [
                "left_front_joint",
                "left_mid_joint",
                "left_back_joint",
            ],
        )
        self.declare_parameter(
            "right_joints",
            [
                "right_front_joint",
                "right_mid_joint",
                "right_back_joint",
            ],
        )
        self.declare_parameter("model_name", "drives")
        self.declare_parameter("output_mode", "effort")
        self.declare_parameter("twist_topic", "/simple_bot/cmd_vel")
        self.declare_parameter("max_twist_linear", 0.8)
        self.declare_parameter("max_twist_angular", 0.8)
        self.declare_parameter("use_scoped_joint_names", False)
        self.declare_parameter("controller_deadzone", 0.1)
        self.declare_parameter("wheelbase_m", 0.83)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 8.0)
        self.declare_parameter("command_duration_s", 0.4)
        self.declare_parameter("linear_effort_gain", 40.0)
        self.declare_parameter("angular_effort_gain", 20.0)
        self.declare_parameter("left_effort_sign", 1.0)
        self.declare_parameter("right_effort_sign", 1.0)
        self.declare_parameter("max_effort", 60.0)

        self.left_joints = list(self.get_parameter("left_joints").value)
        self.right_joints = list(self.get_parameter("right_joints").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.output_mode = str(self.get_parameter("output_mode").value).strip().lower()
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.max_twist_linear = float(self.get_parameter("max_twist_linear").value)
        self.max_twist_angular = float(self.get_parameter("max_twist_angular").value)
        self.use_scoped_joint_names = bool(self.get_parameter("use_scoped_joint_names").value)
        self.controller_deadzone = float(self.get_parameter("controller_deadzone").value)
        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.command_duration_s = float(self.get_parameter("command_duration_s").value)
        self.linear_effort_gain = float(self.get_parameter("linear_effort_gain").value)
        self.angular_effort_gain = float(self.get_parameter("angular_effort_gain").value)
        self.left_effort_sign = float(self.get_parameter("left_effort_sign").value)
        self.right_effort_sign = float(self.get_parameter("right_effort_sign").value)
        self.max_effort = float(self.get_parameter("max_effort").value)

        self.apply_joint_effort = self.create_client(ApplyJointEffort, "/apply_joint_effort")
        self.clear_joint_efforts = self.create_client(JointRequest, "/clear_joint_efforts")
        self.twist_publisher = self.create_publisher(Twist, self.twist_topic, 10)

        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_command_time = self.get_clock().now()
        self.last_command_source = "startup"
        self.services_warned = False
        self.was_active = False
        self._joint_effort_failures = 0
        self._joint_effort_successes = 0
        self.create_subscription(Twist, "/cmd_vel_drives", self.cmd_vel_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Twist, "/autonomy/move/point_turn", self.point_turn_callback, 10)
        self.create_subscription(AutonomyDrive, "/autonomy/move/ackerman", self.ackermann_callback, 10)
        self.create_subscription(
            TwistStamped, "/drives_controller/cmd_vel", self.drives_controller_callback, 10
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_drive_effort)

        self.get_logger().info(
            "Gazebo drive bridge started. Listening on /cmd_vel_drives, /cmd_vel, "
            "/autonomy/move/point_turn, /autonomy/move/ackerman, and /drives_controller/cmd_vel."
        )
        self.get_logger().info(
            f"output_mode={self.output_mode} twist_topic={self.twist_topic} "
            f"left={self.left_joints} right={self.right_joints} "
            f"with gains linear={self.linear_effort_gain} angular={self.angular_effort_gain}. "
            f"scoped_names={self.use_scoped_joint_names} model={self.model_name}"
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        self._set_target(msg.linear.x, msg.angular.z, "/cmd_vel_drives")
        if abs(msg.linear.y) > 1e-3:
            self.get_logger().warn(
                "Ignoring lateral cmd_vel component for Gazebo bridge; the rover model is not holonomic.",
                throttle_duration_sec=5.0,
            )

    def point_turn_callback(self, msg: Twist) -> None:
        self._set_target(0.0, msg.angular.z, "/autonomy/move/point_turn")

    def ackermann_callback(self, msg: AutonomyDrive) -> None:
        steering_angles = [msg.fl_angle, msg.fr_angle, msg.bl_angle, msg.br_angle]
        nonzero_angles = [angle for angle in steering_angles if abs(angle) > 1e-3]
        steering_deg = sum(nonzero_angles) / len(nonzero_angles) if nonzero_angles else 0.0
        steering_rad = math.radians(steering_deg)
        angular_z = 0.0
        if abs(steering_rad) > 1e-3:
            angular_z = msg.vel * math.tan(steering_rad) / self.wheelbase_m
        self._set_target(msg.vel, angular_z, "/autonomy/move/ackerman")

    def drives_controller_callback(self, msg: TwistStamped) -> None:
        # The existing remote controller path publishes stick axes in a nonstandard
        # Twist layout: forward is linear.y (inverted), lateral is linear.x, and turn
        # is usually angular.x. Fall back to standard Twist fields when available.
        linear_x = msg.twist.linear.x
        if abs(msg.twist.linear.y) > self.controller_deadzone:
            linear_x = -msg.twist.linear.y
        elif abs(linear_x) < self.controller_deadzone:
            linear_x = 0.0

        angular_z = msg.twist.angular.z
        if abs(angular_z) < self.controller_deadzone:
            angular_z = msg.twist.angular.x
        if abs(angular_z) < self.controller_deadzone:
            angular_z = 0.0

        self._set_target(linear_x, angular_z, "/drives_controller/cmd_vel")
        if abs(msg.twist.linear.x) > self.controller_deadzone and abs(msg.twist.linear.y) > self.controller_deadzone:
            self.get_logger().warn(
                "Ignoring lateral /drives_controller/cmd_vel component for Gazebo bridge.",
                throttle_duration_sec=5.0,
            )

    def _set_target(self, linear_x: float, angular_z: float, source: str) -> None:
        self.target_linear_x = float(linear_x)
        self.target_angular_z = float(angular_z)
        self.last_command_time = self.get_clock().now()
        self.last_command_source = source
        self.get_logger().info(
            f"Drive command from {source}: linear_x={self.target_linear_x:.3f}, "
            f"angular_z={self.target_angular_z:.3f}",
            throttle_duration_sec=1.0,
        )

    def publish_drive_effort(self) -> None:
        if self.output_mode == "twist":
            self._publish_twist_command()
            return

        if not self.apply_joint_effort.service_is_ready() or not self.clear_joint_efforts.service_is_ready():
            if not self.services_warned:
                self.get_logger().warn(
                    "Gazebo effort services are not ready yet. Waiting for /apply_joint_effort "
                    "and /clear_joint_efforts."
                )
                self.services_warned = True
            return

        self.services_warned = False

        command_age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if command_age > self.command_timeout_s or (
            abs(self.target_linear_x) < 1e-4 and abs(self.target_angular_z) < 1e-4
        ):
            if self.was_active:
                self._clear_all_efforts()
                self.was_active = False
            return

        linear_term = self.linear_effort_gain * self.target_linear_x
        angular_term = self.angular_effort_gain * self.target_angular_z

        left_effort = self._clamp_effort(self.left_effort_sign * (linear_term - angular_term))
        right_effort = self._clamp_effort(self.right_effort_sign * (linear_term + angular_term))

        for joint_name in self.left_joints:
            self._apply_joint_effort(joint_name, left_effort)
        for joint_name in self.right_joints:
            self._apply_joint_effort(joint_name, right_effort)

        self.was_active = True
        self.get_logger().debug(
            f"Applied Gazebo efforts from {self.last_command_source}: "
            f"left={left_effort:.3f}, right={right_effort:.3f}"
        )

    def _publish_twist_command(self) -> None:
        command_age = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        msg = Twist()

        if command_age <= self.command_timeout_s:
            msg.linear.x = max(-self.max_twist_linear, min(self.max_twist_linear, self.target_linear_x))
            msg.angular.z = max(-self.max_twist_angular, min(self.max_twist_angular, self.target_angular_z))
            self.was_active = abs(self.target_linear_x) > 1e-4 or abs(self.target_angular_z) > 1e-4
        else:
            self.was_active = False

        self.twist_publisher.publish(msg)

    def _apply_joint_effort(self, joint_name: str, effort: float) -> None:
        request = ApplyJointEffort.Request()
        request.joint_name = self._gazebo_joint_name(joint_name)
        request.effort = effort
        request.start_time.sec = 0
        request.start_time.nanosec = 0
        duration_nsec = int(self.command_duration_s * 1e9)
        request.duration.sec = duration_nsec // 1_000_000_000
        request.duration.nanosec = duration_nsec % 1_000_000_000
        future = self.apply_joint_effort.call_async(request)
        future.add_done_callback(
            lambda fut, requested_joint=request.joint_name, requested_effort=effort: self._handle_apply_response(
                fut, requested_joint, requested_effort
            )
        )

    def _clear_all_efforts(self) -> None:
        for joint_name in self._all_joints():
            request = JointRequest.Request()
            request.joint_name = self._gazebo_joint_name(joint_name)
            self.clear_joint_efforts.call_async(request)

    def _all_joints(self) -> Iterable[str]:
        return [*self.left_joints, *self.right_joints]

    def _gazebo_joint_name(self, joint_name: str) -> str:
        if self.use_scoped_joint_names and "::" not in joint_name:
            return f"{self.model_name}::{joint_name}"
        return joint_name

    def _handle_apply_response(self, future, joint_name: str, effort: float) -> None:
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive logging for ROS futures
            self._joint_effort_failures += 1
            self.get_logger().warn(
                f"Gazebo apply_joint_effort call failed for {joint_name}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        if not response.success:
            self._joint_effort_failures += 1
            self.get_logger().warn(
                f"Gazebo rejected joint effort for {joint_name} effort={effort:.3f}: "
                f"{response.status_message}",
                throttle_duration_sec=2.0,
            )
            return

        if self._joint_effort_successes == 0:
            self.get_logger().info(
                f"Gazebo accepted joint effort commands; example joint={joint_name} effort={effort:.3f}"
            )
        self._joint_effort_successes += 1

    def _clamp_effort(self, effort: float) -> float:
        return max(-self.max_effort, min(self.max_effort, effort))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboDriveBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
