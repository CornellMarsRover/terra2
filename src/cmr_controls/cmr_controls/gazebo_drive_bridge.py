#!/usr/bin/env python3

import math
from typing import Dict, Iterable, Tuple

import rclpy
from cmr_msgs.msg import AutonomyDrive
from gazebo_msgs.srv import ApplyJointEffort, JointRequest
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GazeboDriveBridge(Node):
    """Mirror rover drive commands into Gazebo.

    The bridge consumes the same topics as the hardware drive stack and emits:
    - a holonomic chassis twist for Gazebo's planar-move plugin
    - steer and wheel joint trajectories so the sim looks like a swerve rover

    The older joint-effort mode is retained as a fallback, but the `gazebo_sim`
    branch now defaults to the twist + joint-trajectory path.
    """

    def __init__(self) -> None:
        super().__init__("gazebo_drive_bridge")

        self.declare_parameter("model_name", "drives")
        self.declare_parameter("output_mode", "twist")
        self.declare_parameter("twist_topic", "/drives/cmd_vel")
        self.declare_parameter("joint_command_topic", "/drives/joint_trajectory_cmd")
        self.declare_parameter("publish_joint_trajectory", True)
        self.declare_parameter("controller_deadzone", 0.1)
        self.declare_parameter("wheelbase_m", 0.83)
        self.declare_parameter("track_width_m", 0.83)
        self.declare_parameter("wheel_radius_m", 0.13)
        self.declare_parameter("ackermann_angular_gain", 1.8)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_twist_linear", 1.2)
        self.declare_parameter("max_twist_lateral", 1.2)
        self.declare_parameter("max_twist_angular", 1.0)
        self.declare_parameter(
            "steer_joints",
            [
                "front_left_steer_joint",
                "front_right_steer_joint",
                "back_left_steer_joint",
                "back_right_steer_joint",
            ],
        )
        self.declare_parameter(
            "wheel_joints",
            [
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "back_left_wheel_joint",
                "back_right_wheel_joint",
            ],
        )

        # Effort-mode fallback parameters.
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
        self.declare_parameter("use_scoped_joint_names", False)
        self.declare_parameter("command_duration_s", 0.4)
        self.declare_parameter("linear_effort_gain", 40.0)
        self.declare_parameter("angular_effort_gain", 20.0)
        self.declare_parameter("left_effort_sign", 1.0)
        self.declare_parameter("right_effort_sign", 1.0)
        self.declare_parameter("max_effort", 60.0)

        self.model_name = str(self.get_parameter("model_name").value)
        self.output_mode = str(self.get_parameter("output_mode").value).strip().lower()
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.joint_command_topic = str(self.get_parameter("joint_command_topic").value)
        self.publish_joint_trajectory = bool(self.get_parameter("publish_joint_trajectory").value)
        self.controller_deadzone = float(self.get_parameter("controller_deadzone").value)
        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.track_width_m = float(self.get_parameter("track_width_m").value)
        self.wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        self.ackermann_angular_gain = float(self.get_parameter("ackermann_angular_gain").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.max_twist_linear = float(self.get_parameter("max_twist_linear").value)
        self.max_twist_lateral = float(self.get_parameter("max_twist_lateral").value)
        self.max_twist_angular = float(self.get_parameter("max_twist_angular").value)
        self.steer_joints = list(self.get_parameter("steer_joints").value)
        self.wheel_joints = list(self.get_parameter("wheel_joints").value)

        self.left_joints = list(self.get_parameter("left_joints").value)
        self.right_joints = list(self.get_parameter("right_joints").value)
        self.use_scoped_joint_names = bool(self.get_parameter("use_scoped_joint_names").value)
        self.command_duration_s = float(self.get_parameter("command_duration_s").value)
        self.linear_effort_gain = float(self.get_parameter("linear_effort_gain").value)
        self.angular_effort_gain = float(self.get_parameter("angular_effort_gain").value)
        self.left_effort_sign = float(self.get_parameter("left_effort_sign").value)
        self.right_effort_sign = float(self.get_parameter("right_effort_sign").value)
        self.max_effort = float(self.get_parameter("max_effort").value)

        self.apply_joint_effort = self.create_client(ApplyJointEffort, "/apply_joint_effort")
        self.clear_joint_efforts = self.create_client(JointRequest, "/clear_joint_efforts")
        self.twist_publisher = self.create_publisher(Twist, self.twist_topic, 10)
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, self.joint_command_topic, 10
        )

        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.last_command_time = self.get_clock().now()
        self.last_command_source = "startup"
        self.last_joint_update_time = self.get_clock().now()
        self.was_active = False
        self.services_warned = False
        self._joint_effort_failures = 0
        self._joint_effort_successes = 0

        self.module_order = [
            "front_left",
            "front_right",
            "back_left",
            "back_right",
        ]
        half_wheelbase = self.wheelbase_m / 2.0
        half_track = self.track_width_m / 2.0
        self.module_offsets: Dict[str, Tuple[float, float]] = {
            "front_left": (half_wheelbase, half_track),
            "front_right": (half_wheelbase, -half_track),
            "back_left": (-half_wheelbase, half_track),
            "back_right": (-half_wheelbase, -half_track),
        }
        self.module_steer_positions = {name: 0.0 for name in self.module_order}
        self.module_wheel_positions = {name: 0.0 for name in self.module_order}

        self.create_subscription(Twist, "/cmd_vel_drives", self.cmd_vel_drives_callback, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Twist, "/autonomy/move/point_turn", self.point_turn_callback, 10)
        self.create_subscription(AutonomyDrive, "/autonomy/move/ackerman", self.ackermann_callback, 10)
        self.create_subscription(
            TwistStamped, "/drives_controller/cmd_vel", self.drives_controller_callback, 10
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_drive_command)

        self.get_logger().info(
            "Gazebo drive bridge started. Listening on /cmd_vel_drives, /cmd_vel, "
            "/autonomy/move/point_turn, /autonomy/move/ackerman, and /drives_controller/cmd_vel."
        )
        self.get_logger().info(
            f"output_mode={self.output_mode} twist_topic={self.twist_topic} "
            f"joint_command_topic={self.joint_command_topic} publish_joint_trajectory={self.publish_joint_trajectory} "
            f"wheelbase={self.wheelbase_m:.3f} track={self.track_width_m:.3f} wheel_radius={self.wheel_radius_m:.3f}"
        )

    def cmd_vel_drives_callback(self, msg: Twist) -> None:
        self._set_target(msg.linear.x, msg.linear.y, msg.angular.z, "/cmd_vel_drives")

    def cmd_vel_callback(self, msg: Twist) -> None:
        self._set_target(msg.linear.x, msg.linear.y, msg.angular.z, "/cmd_vel")

    def point_turn_callback(self, msg: Twist) -> None:
        self._set_target(0.0, 0.0, msg.angular.z, "/autonomy/move/point_turn")

    def ackermann_callback(self, msg: AutonomyDrive) -> None:
        steering_angles = [msg.fl_angle, msg.fr_angle, msg.bl_angle, msg.br_angle]
        nonzero_angles = [angle for angle in steering_angles if abs(angle) > 1e-3]
        steering_deg = sum(nonzero_angles) / len(nonzero_angles) if nonzero_angles else 0.0
        steering_rad = math.radians(steering_deg)
        angular_z = 0.0
        if abs(steering_rad) > 1e-3:
            angular_z = (
                self.ackermann_angular_gain * msg.vel * math.tan(steering_rad) / self.wheelbase_m
            )
        self._set_target(msg.vel, 0.0, angular_z, "/autonomy/move/ackerman")

    def drives_controller_callback(self, msg: TwistStamped) -> None:
        linear_x = self._apply_deadzone(-msg.twist.linear.y)
        linear_y = self._apply_deadzone(msg.twist.linear.x)
        angular_z = self._apply_deadzone(msg.twist.angular.z)
        if angular_z == 0.0:
            angular_z = self._apply_deadzone(msg.twist.angular.x)
        self._set_target(linear_x, linear_y, angular_z, "/drives_controller/cmd_vel")

    def _set_target(self, linear_x: float, linear_y: float, angular_z: float, source: str) -> None:
        self.target_linear_x = float(linear_x)
        self.target_linear_y = float(linear_y)
        self.target_angular_z = float(angular_z)
        self.last_command_time = self.get_clock().now()
        self.last_command_source = source
        self.get_logger().info(
            f"Drive command from {source}: linear_x={self.target_linear_x:.3f}, "
            f"linear_y={self.target_linear_y:.3f}, angular_z={self.target_angular_z:.3f}",
            throttle_duration_sec=1.0,
        )

    def publish_drive_command(self) -> None:
        if self.output_mode == "twist":
            self._publish_twist_and_swerve()
            return

        self._publish_effort_fallback()

    def _publish_twist_and_swerve(self) -> None:
        now = self.get_clock().now()
        dt = max(0.0, min((now - self.last_joint_update_time).nanoseconds / 1e9, 0.25))
        self.last_joint_update_time = now

        command_age = (now - self.last_command_time).nanoseconds / 1e9
        vx = 0.0
        vy = 0.0
        omega = 0.0
        if command_age <= self.command_timeout_s:
            vx = max(-self.max_twist_linear, min(self.max_twist_linear, self.target_linear_x))
            vy = max(-self.max_twist_lateral, min(self.max_twist_lateral, self.target_linear_y))
            omega = max(-self.max_twist_angular, min(self.max_twist_angular, self.target_angular_z))
            self.was_active = abs(vx) > 1e-4 or abs(vy) > 1e-4 or abs(omega) > 1e-4
        else:
            self.was_active = False

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.twist_publisher.publish(twist)

        if self.publish_joint_trajectory:
            self._publish_joint_trajectory(vx, vy, omega, dt)

    def _publish_joint_trajectory(self, vx: float, vy: float, omega: float, dt: float) -> None:
        module_states = self._compute_module_states(vx, vy, omega)
        joint_names = []
        joint_positions = []

        for module_name, steer_joint in zip(self.module_order, self.steer_joints):
            angle, wheel_speed = module_states[module_name]
            optimized_angle, optimized_speed = self._optimize_state(
                angle, wheel_speed, self.module_steer_positions[module_name]
            )
            self.module_steer_positions[module_name] = optimized_angle
            self.module_wheel_positions[module_name] += (optimized_speed / self.wheel_radius_m) * dt
            joint_names.append(steer_joint)
            joint_positions.append(self.module_steer_positions[module_name])

        for module_name, wheel_joint in zip(self.module_order, self.wheel_joints):
            joint_names.append(wheel_joint)
            joint_positions.append(self.module_wheel_positions[module_name])

        msg = JointTrajectory()
        msg.header.frame_id = "base_link"
        msg.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(5e7)
        msg.points.append(point)
        self.joint_trajectory_publisher.publish(msg)

    def _compute_module_states(self, vx: float, vy: float, omega: float) -> Dict[str, Tuple[float, float]]:
        states: Dict[str, Tuple[float, float]] = {}
        for module_name, (module_x, module_y) in self.module_offsets.items():
            wheel_vx = vx - (omega * module_y)
            wheel_vy = vy + (omega * module_x)
            wheel_speed = math.hypot(wheel_vx, wheel_vy)
            if wheel_speed < 1e-6:
                wheel_angle = self.module_steer_positions[module_name]
            else:
                wheel_angle = math.atan2(wheel_vy, wheel_vx)
            states[module_name] = (wheel_angle, wheel_speed)
        return states

    def _optimize_state(
        self, target_angle: float, target_speed: float, current_angle: float
    ) -> Tuple[float, float]:
        delta = self._wrap_angle(target_angle - current_angle)
        if abs(delta) > (math.pi / 2.0):
            delta = self._wrap_angle(delta + math.pi)
            target_speed = -target_speed
        return current_angle + delta, target_speed

    def _publish_effort_fallback(self) -> None:
        if abs(self.target_linear_y) > 1e-3:
            self.get_logger().warn(
                "Effort fallback ignores lateral motion; use output_mode=twist for swerve simulation.",
                throttle_duration_sec=5.0,
            )

        if not self.apply_joint_effort.service_is_ready() or not self.clear_joint_efforts.service_is_ready():
            if not self.services_warned:
                self.get_logger().warn(
                    "Gazebo effort services are not ready yet. Waiting for /apply_joint_effort and /clear_joint_efforts."
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
        except Exception as exc:  # pragma: no cover
            self._joint_effort_failures += 1
            self.get_logger().warn(
                f"Gazebo apply_joint_effort call failed for {joint_name}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        if not response.success:
            self._joint_effort_failures += 1
            self.get_logger().warn(
                f"Gazebo rejected joint effort for {joint_name} effort={effort:.3f}: {response.status_message}",
                throttle_duration_sec=2.0,
            )
            return

        if self._joint_effort_successes == 0:
            self.get_logger().info(
                f"Gazebo accepted joint effort commands; example joint={joint_name} effort={effort:.3f}"
            )
        self._joint_effort_successes += 1

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.controller_deadzone:
            return 0.0
        return float(value)

    def _wrap_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

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
