#!/usr/bin/env python3
"""
Arm Coordinator Node: Receives high-confidence key poses from the camera node,
transforms them from camera_frame to base_link, and drives the arm through
the WAITING_FOR_LOCK -> PRE_APPROACH -> APPROACH -> PRESS -> RETRACT -> IDLE
state machine by publishing to /arm_target_pose.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs
import math
from enum import Enum, auto


# ---------------------------------------------------------------------------
# State machine states
# ---------------------------------------------------------------------------

class State(Enum):
    WAITING_FOR_LOCK  = auto()
    MOVING_TO_PREAPPROACH = auto()
    APPROACHING       = auto()
    PRESSING          = auto()
    RETRACTING        = auto()
    IDLE              = auto()


# ---------------------------------------------------------------------------
# Coordinator node
# ---------------------------------------------------------------------------

class ArmCoordinator(Node):

    # How far above the key to hover before descending (metres)
    PREAPPROACH_OFFSET_Z = 0.05

    # Confidence threshold — must match or be tighter than camera node
    VARIANCE_THRESHOLD = 2.0

    # How long to wait at each state before advancing (seconds).
    # TODO: Tune these to arm's actual motion speed.
    STATE_DURATIONS = {
        State.MOVING_TO_PREAPPROACH: 3.0,
        State.APPROACHING:           2.0,
        State.PRESSING:              0.5,
        State.RETRACTING:            2.0,
    }

    def __init__(self):
        super().__init__('arm_coordinator')

        # --- TF2 ---
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- State ---
        self.state        = State.WAITING_FOR_LOCK
        self.locked_pose  = None   # PoseStamped in base_link frame
        self.state_start  = None   # wall-clock time we entered current state

        # --- Sub / Pub ---
        self.key_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/key_location',
            self.key_callback,
            10
        )

        self.arm_pub = self.create_publisher(
            PoseStamped,
            '/arm_target_pose',
            10
        )

        # Optional: publish current state name for debugging / other nodes
        self.state_pub = self.create_publisher(String, '/coordinator_state', 10)

        # Timer drives the state machine at 10 Hz
        self.timer = self.create_timer(0.1, self.state_machine_tick)

        self.get_logger().info("Arm coordinator started — waiting for high-confidence key pose.")

    # -----------------------------------------------------------------------
    # Camera subscriber callback
    # -----------------------------------------------------------------------

    def key_callback(self, msg: PoseWithCovarianceStamped):
        """Only process the pose if we're still waiting and confidence is high."""
        if self.state != State.WAITING_FOR_LOCK:
            return

        # Check confidence from covariance diagonal
        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]
        if max(var_x, var_y) > self.VARIANCE_THRESHOLD:
            self.get_logger().debug(
                f"Low confidence (var_x={var_x:.3f}, var_y={var_y:.3f}), still waiting.")
            return

        # Transform from camera_frame -> base_link
        pose_in_camera = PoseStamped()
        pose_in_camera.header = msg.header   # frame_id = 'camera_frame'
        pose_in_camera.pose   = msg.pose.pose

        try:
            pose_in_base = self.tf_buffer.transform(pose_in_camera, 'base_link')
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        self.locked_pose = pose_in_base
        self.get_logger().info(
            f"Pose locked in base_link: "
            f"x={pose_in_base.pose.position.x:.3f} "
            f"y={pose_in_base.pose.position.y:.3f} "
            f"z={pose_in_base.pose.position.z:.3f}"
        )
        self._transition(State.MOVING_TO_PREAPPROACH)

    # -----------------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------------

    def state_machine_tick(self):
        """Called at 10 Hz. Publishes IK targets and advances states on timeout."""
        self._publish_state_name()

        if self.state == State.WAITING_FOR_LOCK:
            return  # Nothing to do — key_callback will advance us

        if self.state == State.MOVING_TO_PREAPPROACH:
            self._publish_preapproach()
            if self._state_elapsed() >= self.STATE_DURATIONS[State.MOVING_TO_PREAPPROACH]:
                self._transition(State.APPROACHING)

        elif self.state == State.APPROACHING:
            self._publish_approach()
            if self._state_elapsed() >= self.STATE_DURATIONS[State.APPROACHING]:
                self._transition(State.PRESSING)

        elif self.state == State.PRESSING:
            self._publish_press()
            if self._state_elapsed() >= self.STATE_DURATIONS[State.PRESSING]:
                self._transition(State.RETRACTING)

        elif self.state == State.RETRACTING:
            self._publish_preapproach()   # Retract back to pre-approach height
            if self._state_elapsed() >= self.STATE_DURATIONS[State.RETRACTING]:
                self._transition(State.IDLE)

        elif self.state == State.IDLE:
            pass  # Done — wait for external trigger or shutdown

    # -----------------------------------------------------------------------
    # Pose publishers for each phase
    # -----------------------------------------------------------------------

    def _publish_preapproach(self):
        """Hover above the key."""
        pose = self._copy_locked_pose()
        pose.pose.position.z += self.PREAPPROACH_OFFSET_Z
        self.arm_pub.publish(pose)

    def _publish_approach(self):
        """Move down to the key surface."""
        pose = self._copy_locked_pose()
        # No Z offset — go straight to the locked pose height
        self.arm_pub.publish(pose)

    def _publish_press(self):
        """Press the key (same position as approach; arm holds here)."""
        pose = self._copy_locked_pose()
        self.arm_pub.publish(pose)

    # -----------------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------------

    def _copy_locked_pose(self) -> PoseStamped:
        """Return a fresh PoseStamped copy of the locked pose with current timestamp."""
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = self.locked_pose.header.frame_id
        pose.pose            = self.locked_pose.pose
        return pose

    def _transition(self, new_state: State):
        self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")
        self.state       = new_state
        self.state_start = self.get_clock().now()

    def _state_elapsed(self) -> float:
        """Seconds spent in the current state."""
        if self.state_start is None:
            return 0.0
        delta = self.get_clock().now() - self.state_start
        return delta.nanoseconds / 1e9

    def _publish_state_name(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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


if __name__ == '__main__':
    main()
