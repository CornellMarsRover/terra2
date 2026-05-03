#!/usr/bin/env python3

from typing import List

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from autonomous_navigation.obstacle_guard_core import (
    CostEntry,
    choose_turn_direction,
    summarize_front_obstacles,
)


class ObstacleGuardNode(Node):
    """Emergency obstacle override layered on top of the existing planner."""

    def __init__(self) -> None:
        super().__init__("obstacle_guard")

        self.declare_parameter("lookahead_distance", 1.5)
        self.declare_parameter("hard_stop_distance", 0.8)
        self.declare_parameter("corridor_half_width", 0.55)
        self.declare_parameter("cost_threshold", 8.0)
        self.declare_parameter("turn_speed", 0.45)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.lookahead_distance = float(self.get_parameter("lookahead_distance").value)
        self.hard_stop_distance = float(self.get_parameter("hard_stop_distance").value)
        self.corridor_half_width = float(self.get_parameter("corridor_half_width").value)
        self.cost_threshold = float(self.get_parameter("cost_threshold").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)

        self.cost_entries: List[CostEntry] = []
        self.robot_north = 0.0
        self.robot_west = 0.0
        self.robot_yaw = 0.0

        self.override_pub = self.create_publisher(
            Twist, "/autonomy/obstacle_avoidance/override", 10
        )
        self.active_pub = self.create_publisher(Bool, "/autonomy/obstacle_avoidance/active", 10)

        self.create_subscription(
            Float32MultiArray, "/autonomy/costmap", self.costmap_callback, 10
        )
        self.create_subscription(
            TwistStamped, "/autonomy/pose/robot/global", self.pose_callback, 10
        )

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_override)

        self.get_logger().info("Obstacle guard active")

    def costmap_callback(self, msg: Float32MultiArray) -> None:
        if len(msg.data) % 3 != 0:
            self.get_logger().warn("Received malformed costmap payload", throttle_duration_sec=5.0)
            return
        entries: List[CostEntry] = []
        for i in range(0, len(msg.data), 3):
            entries.append((msg.data[i], msg.data[i + 1], msg.data[i + 2]))
        self.cost_entries = entries

    def pose_callback(self, msg: TwistStamped) -> None:
        self.robot_north = msg.twist.linear.x
        self.robot_west = msg.twist.linear.y
        self.robot_yaw = msg.twist.angular.z

    def publish_override(self) -> None:
        summary = summarize_front_obstacles(
            self.cost_entries,
            self.robot_north,
            self.robot_west,
            self.robot_yaw,
            self.lookahead_distance,
            self.corridor_half_width,
            self.cost_threshold,
        )

        intervention_required = summary.blocked and summary.nearest_distance <= self.hard_stop_distance

        active_msg = Bool()
        active_msg.data = intervention_required
        self.active_pub.publish(active_msg)

        override = Twist()
        if summary.blocked:
            if intervention_required:
                override.angular.z = choose_turn_direction(summary) * self.turn_speed
            self.get_logger().info(
                "obstacle_guard "
                f"blocked={summary.blocked} nearest={summary.nearest_distance:.2f} "
                f"left={summary.left_cost:.1f} center={summary.center_cost:.1f} right={summary.right_cost:.1f} "
                f"intervene={intervention_required} override_turn={override.angular.z:+.3f}",
                throttle_duration_sec=1.0,
            )
        self.override_pub.publish(override)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
