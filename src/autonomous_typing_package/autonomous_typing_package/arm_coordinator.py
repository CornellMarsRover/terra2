#!/usr/bin/env python3

import copy
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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

    STATE_DURATIONS = {
        State.MOVING_TO_PREAPPROACH: 3.0,
        State.APPROACHING: 2.0,
        State.PRESSING: 0.5,
        State.RETRACTING: 2.0,
    }

    def __init__(self):
        super().__init__("arm_coordinator")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = State.WAITING_FOR_LOCK
        self.locked_pose = None
        self.state_start = None

        self.target_pose_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self.state_pub = self.create_publisher(String, "/coordinator_state", 10)

        self.key_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/key_location",
            self.key_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.state_machine_tick)

        self.get_logger().info(
            "Arm coordinator started in Servo target-pose mode; waiting for high-confidence key pose."
        )

    def key_callback(self, msg: PoseWithCovarianceStamped):
        if self.state != State.WAITING_FOR_LOCK:
            return

        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]
        if max(var_x, var_y) > self.VARIANCE_THRESHOLD:
            self.get_logger().debug(
                f"Low confidence (var_x={var_x:.3f}, var_y={var_y:.3f}), still waiting."
            )
            return

        pose_in_camera = PoseStamped()
        pose_in_camera.header = msg.header
        pose_in_camera.pose = msg.pose.pose

        try:
            pose_in_base = self.tf_buffer.transform(
                pose_in_camera,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        self.locked_pose = pose_in_base
        self.get_logger().info(
            "Pose locked in base_link: "
            f"x={pose_in_base.pose.position.x:.3f} "
            f"y={pose_in_base.pose.position.y:.3f} "
            f"z={pose_in_base.pose.position.z:.3f}"
        )
        self._transition(State.MOVING_TO_PREAPPROACH)

    def state_machine_tick(self):
        self._publish_state_name()

        if self.state == State.WAITING_FOR_LOCK:
            return

        if self.locked_pose is None:
            self.get_logger().warn("State machine active but locked_pose is None")
            self._transition(State.WAITING_FOR_LOCK)
            return

        if self.state == State.MOVING_TO_PREAPPROACH:
            self._publish_target_pose(self._get_preapproach_pose())
            if self._state_elapsed() >= self.STATE_DURATIONS[State.MOVING_TO_PREAPPROACH]:
                self._transition(State.APPROACHING)

        elif self.state == State.APPROACHING:
            self._publish_target_pose(self._get_approach_pose())
            if self._state_elapsed() >= self.STATE_DURATIONS[State.APPROACHING]:
                self._transition(State.PRESSING)

        elif self.state == State.PRESSING:
            self._publish_target_pose(self._get_press_pose())
            if self._state_elapsed() >= self.STATE_DURATIONS[State.PRESSING]:
                self._transition(State.RETRACTING)

        elif self.state == State.RETRACTING:
            self._publish_target_pose(self._get_preapproach_pose())
            if self._state_elapsed() >= self.STATE_DURATIONS[State.RETRACTING]:
                self._transition(State.IDLE)

        elif self.state == State.IDLE:
            pass

    def _publish_target_pose(self, pose: PoseStamped):
        pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_pub.publish(pose)

    def _get_preapproach_pose(self) -> PoseStamped:
        pose = self._copy_locked_pose()
        pose.pose.position.z += self.PREAPPROACH_OFFSET_Z
        return pose

    def _get_approach_pose(self) -> PoseStamped:
        return self._copy_locked_pose()

    def _get_press_pose(self) -> PoseStamped:
        pose = self._copy_locked_pose()
        pose.pose.position.z += self.PRESS_OFFSET_Z
        return pose

    def _copy_locked_pose(self) -> PoseStamped:
        pose = copy.deepcopy(self.locked_pose)
        pose.header.frame_id = "base_link"
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()