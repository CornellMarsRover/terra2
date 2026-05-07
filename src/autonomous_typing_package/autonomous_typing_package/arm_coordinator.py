#!/usr/bin/env python3

import copy
import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from std_msgs.msg import String
import tf2_ros


class State(Enum):
    WAITING_FOR_LOCK = auto()
    MOVING_TO_PREAPPROACH = auto()
    APPROACHING = auto()
    PRESSING = auto()
    RETRACTING = auto()
    IDLE = auto()


class ArmCoordinator(Node):
    PREAPPROACH_OFFSET_Z = 0.05
    PRESS_OFFSET_Z = -0.003
    VARIANCE_THRESHOLD = 2.0
    IDLE_DURATION = 1.0  # seconds to pause in IDLE before accepting next key

    STATE_DURATIONS = {
        State.MOVING_TO_PREAPPROACH: 3.0,
        State.APPROACHING: 2.0,
        State.PRESSING: 0.5,
        State.RETRACTING: 2.0,
    }

    def __init__(self):
        super().__init__("arm_coordinator")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("key_pose_topic", "/key_location")
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("state_topic", "/coordinator_state")
        self.declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("ee_frame", "end_effector")
        self.declare_parameter("preapproach_offset_z", self.PREAPPROACH_OFFSET_Z)
        self.declare_parameter("press_offset_z", self.PRESS_OFFSET_Z)
        self.declare_parameter("variance_threshold", self.VARIANCE_THRESHOLD)
        self.declare_parameter("linear_gain", 2.0)
        self.declare_parameter("max_linear_command", 0.35)
        self.declare_parameter("angular_gain", 2.0)
        self.declare_parameter("max_angular_command", 0.5)
        self.declare_parameter("position_tolerance", 0.01)

        self.target_frame = self.get_parameter("target_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.preapproach_offset_z = self.get_parameter("preapproach_offset_z").value
        self.press_offset_z = self.get_parameter("press_offset_z").value
        self.variance_threshold = self.get_parameter("variance_threshold").value
        self.linear_gain = self.get_parameter("linear_gain").value
        self.max_linear_command = self.get_parameter("max_linear_command").value
        self.angular_gain = self.get_parameter("angular_gain").value
        self.max_angular_command = self.get_parameter("max_angular_command").value
        self.position_tolerance = self.get_parameter("position_tolerance").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = State.WAITING_FOR_LOCK
        self.locked_pose = None
        self.state_start = None

        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter("target_pose_topic").value,
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            self.get_parameter("state_topic").value,
            10,
        )
        self.servo_twist_pub = self.create_publisher(
            TwistStamped,
            self.get_parameter("servo_twist_topic").value,
            10,
        )

        self.key_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("key_pose_topic").value,
            self.key_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.state_machine_tick)

        self.get_logger().info(
            "Arm coordinator started; "
            f"waiting for high-confidence key pose in target frame '{self.target_frame}'. "
            f"Servo twists will command '{self.ee_frame}'."
        )

    def key_callback(self, msg: PoseWithCovarianceStamped):
        if self.state != State.WAITING_FOR_LOCK:
            return

        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]
        if max(var_x, var_y) > self.variance_threshold:
            self.get_logger().debug(
                f"Low confidence (var_x={var_x:.3f}, var_y={var_y:.3f}), still waiting."
            )
            return

        pose_in_camera = PoseStamped()
        pose_in_camera.header = msg.header
        pose_in_camera.pose = msg.pose.pose

        if pose_in_camera.header.frame_id == self.target_frame:
            pose_in_base = pose_in_camera
        else:
            try:
                pose_in_base = self.tf_buffer.transform(
                    pose_in_camera,
                    self.target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {e}")
                return

        self.locked_pose = pose_in_base
        self.get_logger().info(
            f"Pose locked in {self.target_frame}: "
            f"x={pose_in_base.pose.position.x:.3f} "
            f"y={pose_in_base.pose.position.y:.3f} "
            f"z={pose_in_base.pose.position.z:.3f}"
        )
        self._transition(State.MOVING_TO_PREAPPROACH)

    def state_machine_tick(self):
        self._publish_state_name()

        if self.state == State.WAITING_FOR_LOCK:
            self._publish_zero_twist()
            return

        if self.locked_pose is None:
            self.get_logger().warn("State machine active but locked_pose is None")
            self._publish_zero_twist()
            self._transition(State.WAITING_FOR_LOCK)
            return

        if self.state == State.MOVING_TO_PREAPPROACH:
            self._publish_target_and_servo_command(self._get_preapproach_pose())
            if self._state_elapsed() >= self.STATE_DURATIONS[State.MOVING_TO_PREAPPROACH]:
                self._transition(State.APPROACHING)

        elif self.state == State.APPROACHING:
            self._publish_target_and_servo_command(self._get_approach_pose())
            if self._state_elapsed() >= self.STATE_DURATIONS[State.APPROACHING]:
                self._transition(State.PRESSING)

        elif self.state == State.PRESSING:
            self._publish_target_and_servo_command(self._get_press_pose())
            if self._state_elapsed() >= self.STATE_DURATIONS[State.PRESSING]:
                self._transition(State.RETRACTING)

        elif self.state == State.RETRACTING:
            self._publish_target_and_servo_command(self._get_preapproach_pose())
            if self._state_elapsed() >= self.STATE_DURATIONS[State.RETRACTING]:
                self._transition(State.IDLE)

        elif self.state == State.IDLE:
            self._publish_zero_twist()
            # After a short pause, reset and wait for the next key from the mission node
            if self._state_elapsed() >= self.IDLE_DURATION:
                self.locked_pose = None
                self._transition(State.WAITING_FOR_LOCK)

    def _publish_target_and_servo_command(self, pose: PoseStamped):
        self._publish_target_pose(pose)
        ee_transform = self._lookup_ee_transform()
        if ee_transform is None:
            self._publish_zero_twist()
            return
        self.servo_twist_pub.publish(
            self._make_servo_twist(pose, ee_transform)
        )

    def _publish_target_pose(self, pose: PoseStamped):
        pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_pub.publish(pose)

    def _lookup_ee_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Could not look up {self.target_frame}->{self.ee_frame}: {exc}",
                throttle_duration_sec=1.0,
            )
            return None

    def _make_servo_twist(self, target_pose: PoseStamped, ee_transform) -> TwistStamped:
        current_pos = ee_transform.transform.translation
        current_rot = ee_transform.transform.rotation

        error_x = target_pose.pose.position.x - current_pos.x
        error_y = target_pose.pose.position.y - current_pos.y
        error_z = target_pose.pose.position.z - current_pos.z

        msg = self._zero_twist_msg()

        if not (
            abs(error_x) < self.position_tolerance
            and abs(error_y) < self.position_tolerance
            and abs(error_z) < self.position_tolerance
        ):
            msg.twist.linear.x = self._bounded_linear_command(error_x)
            msg.twist.linear.y = self._bounded_linear_command(error_y)
            msg.twist.linear.z = self._bounded_linear_command(error_z)

        # Angular correction: drive end effector toward target orientation
        target_q = target_pose.pose.orientation
        ax, ay, az = self._quaternion_error_to_angular(target_q, current_rot)
        msg.twist.angular.x = self._bounded_angular_command(ax)
        msg.twist.angular.y = self._bounded_angular_command(ay)
        msg.twist.angular.z = self._bounded_angular_command(az)

        return msg

    @staticmethod
    def _quaternion_error_to_angular(q_target, q_current):
        """Return angular error [x, y, z] as q_error = q_target * conj(q_current)."""
        tw, tx, ty, tz = q_target.w, q_target.x, q_target.y, q_target.z
        cw, cx, cy, cz = q_current.w, q_current.x, q_current.y, q_current.z

        # q_error = q_target * conj(q_current)
        ew = tw * cw + tx * cx + ty * cy + tz * cz
        ex = -tw * cx + tx * cw - ty * cz + tz * cy
        ey = -tw * cy + tx * cz + ty * cw - tz * cx
        ez = -tw * cz - tx * cy + ty * cx + tz * cw

        # Take the shorter arc
        if ew < 0:
            ex, ey, ez = -ex, -ey, -ez

        return ex, ey, ez

    def _bounded_linear_command(self, error: float) -> float:
        command = self.linear_gain * error
        return max(-self.max_linear_command, min(self.max_linear_command, command))

    def _bounded_angular_command(self, error: float) -> float:
        command = self.angular_gain * error
        return max(-self.max_angular_command, min(self.max_angular_command, command))

    def _publish_zero_twist(self):
        self.servo_twist_pub.publish(self._zero_twist_msg())

    def _zero_twist_msg(self) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        return msg

    def _get_preapproach_pose(self) -> PoseStamped:
        pose = self._copy_locked_pose()
        pose.pose.position.z += self.preapproach_offset_z
        return pose

    def _get_approach_pose(self) -> PoseStamped:
        return self._copy_locked_pose()

    def _get_press_pose(self) -> PoseStamped:
        pose = self._copy_locked_pose()
        pose.pose.position.z += self.press_offset_z
        return pose

    def _copy_locked_pose(self) -> PoseStamped:
        pose = copy.deepcopy(self.locked_pose)
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        return pose

    def _transition(self, new_state: State):
        self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_start = self.get_clock().now()

    def _state_elapsed(self) -> float:
        if self.state_start is None:
            return 0.0
        delta = self.get_clock().now() - self.state_start
        return delta.nanoseconds / 1e9

    def _publish_state_name(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = ArmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
