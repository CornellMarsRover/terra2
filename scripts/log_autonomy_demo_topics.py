#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from cmr_msgs.msg import AutonomyDrive
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class DemoTopicLogger(Node):
    def __init__(self, duration_s: float, sample_rate_hz: float) -> None:
        super().__init__("autonomy_demo_topic_logger")
        self.duration_s = duration_s
        self.start_time = time.monotonic()
        self.done = False
        self.samples: List[Dict[str, Any]] = []

        self.pose: Optional[Dict[str, float]] = None
        self.goal: Optional[List[float]] = None
        self.local_target: Optional[List[float]] = None
        self.next_waypoint: Optional[List[float]] = None
        self.costmap: List[float] = []
        self.avoidance_active = False
        self.all_obstacles: List[float] = []
        self.visible_obstacles: List[float] = []
        self.ackermann_cmd: Optional[Dict[str, float]] = None
        self.point_turn_cmd: Optional[Dict[str, float]] = None

        self.create_subscription(
            TwistStamped, "/autonomy/pose/robot/global", self.pose_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/autonomy/target/global", self.goal_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/autonomy/target/local", self.local_target_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/autonomy/path/next_waypoint", self.next_waypoint_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/autonomy/costmap", self.costmap_callback, 10
        )
        self.create_subscription(
            Bool, "/autonomy/obstacle_avoidance/active", self.avoidance_active_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/autonomy/sim_obstacles/all", self.all_obstacles_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/autonomy/sim_obstacles/visible", self.visible_obstacles_callback, 10
        )
        self.create_subscription(
            AutonomyDrive, "/autonomy/move/ackerman", self.ackermann_callback, 10
        )
        self.create_subscription(
            Twist, "/autonomy/move/point_turn", self.point_turn_callback, 10
        )

        self.timer = self.create_timer(1.0 / sample_rate_hz, self.sample_once)

    def elapsed(self) -> float:
        return time.monotonic() - self.start_time

    def pose_callback(self, msg: TwistStamped) -> None:
        self.pose = {
            "north": float(msg.twist.linear.x),
            "west": float(msg.twist.linear.y),
            "yaw": float(msg.twist.angular.z),
        }

    def goal_callback(self, msg: Float32MultiArray) -> None:
        self.goal = [float(value) for value in msg.data]

    def local_target_callback(self, msg: Float32MultiArray) -> None:
        self.local_target = [float(value) for value in msg.data]

    def next_waypoint_callback(self, msg: Float32MultiArray) -> None:
        self.next_waypoint = [float(value) for value in msg.data]

    def costmap_callback(self, msg: Float32MultiArray) -> None:
        self.costmap = [float(value) for value in msg.data]

    def avoidance_active_callback(self, msg: Bool) -> None:
        self.avoidance_active = bool(msg.data)

    def all_obstacles_callback(self, msg: Float32MultiArray) -> None:
        self.all_obstacles = [float(value) for value in msg.data]

    def visible_obstacles_callback(self, msg: Float32MultiArray) -> None:
        self.visible_obstacles = [float(value) for value in msg.data]

    def ackermann_callback(self, msg: AutonomyDrive) -> None:
        self.ackermann_cmd = {
            "vel": float(msg.vel),
            "fl": float(msg.fl_angle),
            "fr": float(msg.fr_angle),
            "bl": float(msg.bl_angle),
            "br": float(msg.br_angle),
        }

    def point_turn_callback(self, msg: Twist) -> None:
        self.point_turn_cmd = {"angular_z": float(msg.angular.z)}

    def sample_once(self) -> None:
        t = self.elapsed()
        sample = {
            "t": t,
            "pose": self.pose,
            "goal": self.goal,
            "local_target": self.local_target,
            "next_waypoint": self.next_waypoint,
            "costmap": self.costmap,
            "avoidance_active": self.avoidance_active,
            "all_obstacles": self.all_obstacles,
            "visible_obstacles": self.visible_obstacles,
            "ackermann_cmd": self.ackermann_cmd,
            "point_turn_cmd": self.point_turn_cmd,
        }
        self.samples.append(sample)
        if t >= self.duration_s:
            self.done = True


def main() -> None:
    parser = argparse.ArgumentParser(description="Record autonomy demo topics to JSON.")
    parser.add_argument("--duration", type=float, default=24.0)
    parser.add_argument("--sample-rate", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = DemoTopicLogger(duration_s=args.duration, sample_rate_hz=args.sample_rate)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        args.output.write_text(json.dumps({"samples": node.samples}, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
