#!/usr/bin/env python3

import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class ArmCoordinator(Node):
    """Run a fixed, open-loop A -> C -> M -> R typing sequence."""

    def __init__(self):
        super().__init__("arm_coordinator")

        self.declare_parameter("servo_twist_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("text", "CMR")
        self.declare_parameter("key_offset_scale", 3.0)
        self.declare_parameter("press_depth_m", 0.0762)  # 3 inches
        self.declare_parameter("command_magnitude", 0.05)
        self.declare_parameter("servo_linear_scale_mps", 0.8)
        self.declare_parameter("start_delay_s", 2.0)
        self.declare_parameter("settle_time_s", 0.3)
        self.declare_parameter("enable_motion", True)

        self.text = str(self.get_parameter("text").value).upper()
        self.key_offset_scale = float(
            self.get_parameter("key_offset_scale").value
        )
        self.press_depth_m = float(self.get_parameter("press_depth_m").value)
        self.command_magnitude = float(
            self.get_parameter("command_magnitude").value
        )
        self.servo_linear_scale_mps = float(
            self.get_parameter("servo_linear_scale_mps").value
        )
        self.start_delay_s = float(self.get_parameter("start_delay_s").value)
        self.settle_time_s = float(self.get_parameter("settle_time_s").value)
        self.enable_motion = bool(self.get_parameter("enable_motion").value)

        key_file = os.path.join(
            get_package_share_directory("autonomous_typing_package"),
            "key_position.json",
        )
        with open(key_file, "r", encoding="utf-8") as stream:
            self.key_positions = json.load(stream)

        missing = [key for key in "A" + self.text if key not in self.key_positions]
        if missing:
            raise ValueError(f"Keys missing from key_position.json: {missing}")
        if self.press_depth_m <= 0.0:
            raise ValueError("press_depth_m must be positive")
        if self.key_offset_scale <= 0.0:
            raise ValueError("key_offset_scale must be positive")
        if not 0.0 < self.command_magnitude <= 1.0:
            raise ValueError("command_magnitude must be in (0, 1]")
        if self.servo_linear_scale_mps <= 0.0:
            raise ValueError("servo_linear_scale_mps must be positive")

        self.segments = self.build_segments()
        self.segment_index = -1
        self.segment_started_at = None
        self.settle_started_at = None
        self.started_at = self.get_clock().now()
        self.done = False

        self.twist_pub = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("servo_twist_topic").value),
            10,
        )
        self.timer = self.create_timer(0.05, self.tick)

        self.get_logger().info(
            f"Starting hardcoded '{self.text}' sequence from A in "
            f"{self.start_delay_s:.1f}s; press depth={self.press_depth_m:.4f}m."
        )

    def build_segments(self):
        segments = []
        previous_key = "A"
        for key in self.text:
            previous = self.key_positions[previous_key]
            target = self.key_positions[key]
            dx = (
                (float(target["x"]) - float(previous["x"]))
                * 0.01
                * self.key_offset_scale
            )
            dy = (
                (float(target["y"]) - float(previous["y"]))
                * 0.01
                * self.key_offset_scale
            )
            segments.append(self.make_segment(f"MOVE_{previous_key}_TO_{key}", dx, dy, 0.0))
            segments.append(self.make_segment(f"PRESS_{key}", 0.0, 0.0, -self.press_depth_m))
            segments.append(self.make_segment(f"RETRACT_{key}", 0.0, 0.0, self.press_depth_m))
            previous_key = key
        return segments

    def make_segment(self, name, dx, dy, dz):
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = self.command_magnitude * self.servo_linear_scale_mps
        duration = distance / speed
        direction = (dx / distance, dy / distance, dz / distance)
        return {
            "name": name,
            "command": tuple(value * self.command_magnitude for value in direction),
            "duration": duration,
            "offset": (dx, dy, dz),
        }

    def tick(self):
        if self.done:
            self.publish_zero()
            return

        if self.elapsed_since(self.started_at) < self.start_delay_s:
            self.publish_zero()
            return

        if self.segment_started_at is None:
            self.start_next_segment()
            return

        segment = self.segments[self.segment_index]
        if self.settle_started_at is None:
            if self.elapsed_since(self.segment_started_at) < segment["duration"]:
                self.publish_command(segment["command"])
            else:
                self.publish_zero()
                self.settle_started_at = self.get_clock().now()
        elif self.elapsed_since(self.settle_started_at) >= self.settle_time_s:
            self.start_next_segment()

    def start_next_segment(self):
        self.segment_index += 1
        if self.segment_index >= len(self.segments):
            self.done = True
            self.publish_zero()
            self.get_logger().info("Hardcoded typing sequence complete.")
            return

        self.segment_started_at = self.get_clock().now()
        self.settle_started_at = None
        segment = self.segments[self.segment_index]
        dx, dy, dz = segment["offset"]
        self.get_logger().info(
            f"{segment['name']}: offset=({dx:.4f}, {dy:.4f}, {dz:.4f})m, "
            f"duration={segment['duration']:.2f}s"
        )

    def publish_command(self, command):
        if not self.enable_motion:
            return
        msg = self.zero_twist()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = command
        self.twist_pub.publish(msg)

    def publish_zero(self):
        if self.enable_motion:
            self.twist_pub.publish(self.zero_twist())

    def zero_twist(self):
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def elapsed_since(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = ArmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
