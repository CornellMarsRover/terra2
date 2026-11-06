#!/usr/bin/env python3
"""ROS adapter that fuses gesture perception and offline voice commands."""

from __future__ import annotations

import copy
import math

import rclpy
from cmr_msgs.msg import AstronautCommand, AstronautVoiceCommand
from rclpy.node import Node
from std_msgs.msg import String

from astronaut_autonomy.command_fusion import (
    CommandFusion,
    CommandObservation,
    FusionConfig,
)
from astronaut_autonomy.mission_core import PerceptionSample


class AstronautCommandFusionNode(Node):
    def __init__(self):
        super().__init__("astronaut_command_fusion")
        self._declare_parameters()
        self.fusion = CommandFusion(
            FusionConfig(
                minimum_gesture_confidence=float(
                    self.get_parameter("minimum_gesture_confidence").value
                ),
                minimum_voice_confidence=float(
                    self.get_parameter("minimum_voice_confidence").value
                ),
                gesture_timeout_s=float(
                    self.get_parameter("gesture_timeout_s").value
                ),
                voice_timeout_s=float(
                    self.get_parameter("voice_timeout_s").value
                ),
                depth_timeout_s=float(
                    self.get_parameter("depth_timeout_s").value
                ),
                duplicate_voice_window_s=float(
                    self.get_parameter("duplicate_voice_window_s").value
                ),
                maximum_future_skew_s=float(
                    self.get_parameter("maximum_future_skew_s").value
                ),
            )
        )
        self.latest_gesture_message = None
        self.last_report = None

        self.command_publisher = self.create_publisher(
            AstronautCommand,
            "/autonomy/astronaut/perception",
            10,
        )
        self.source_publisher = self.create_publisher(
            String,
            "/autonomy/astronaut/command_source",
            10,
        )
        self.detail_publisher = self.create_publisher(
            String,
            "/autonomy/astronaut/fusion_detail",
            10,
        )
        self.create_subscription(
            AstronautCommand,
            "/autonomy/astronaut/gesture_perception",
            self._gesture_callback,
            10,
        )
        self.create_subscription(
            AstronautVoiceCommand,
            "/autonomy/astronaut/voice_command",
            self._voice_callback,
            10,
        )
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish)
        self.get_logger().info(
            "Astronaut command fusion ready; gesture FOLLOW is primary, "
            "conflicts produce a safety hold, and physical drive output "
            "remains in the mission node"
        )

    def _declare_parameters(self):
        defaults = {
            "publish_rate_hz": 10.0,
            "minimum_gesture_confidence": 0.50,
            "minimum_voice_confidence": 0.70,
            "gesture_timeout_s": 0.75,
            "voice_timeout_s": 1.25,
            "depth_timeout_s": 0.75,
            "duplicate_voice_window_s": 1.50,
            "maximum_future_skew_s": 0.10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _gesture_callback(self, message: AstronautCommand) -> None:
        observed_at = self._stamp_seconds(message.header.stamp)
        if observed_at is None:
            self.get_logger().warning(
                "Rejecting gesture perception without a valid timestamp",
                throttle_duration_sec=2.0,
            )
            return
        sample = PerceptionSample(
            command=message.command,
            gesture=message.gesture,
            confidence=float(message.confidence),
            astronaut_detected=bool(message.astronaut_detected),
            distance_valid=bool(message.distance_valid),
            depth_status=message.depth_status,
            distance_m=float(message.distance_m),
            bearing_deg=float(message.horizontal_angle_deg),
        )
        if self.fusion.update_gesture(
            sample,
            observed_at,
            source="GESTURE",
            received_at_s=self._now_s(),
        ):
            self.latest_gesture_message = copy.deepcopy(message)

    def _voice_callback(self, message: AstronautVoiceCommand) -> None:
        observed_at = self._stamp_seconds(message.header.stamp)
        if observed_at is None:
            self.get_logger().warning(
                "Rejecting voice command without a valid timestamp",
                throttle_duration_sec=2.0,
            )
            return
        accepted = self.fusion.update_voice(
            CommandObservation(
                command=message.command,
                confidence=float(message.confidence),
                source=message.source,
                observed_at_s=observed_at,
                transcript=message.transcript,
            ),
            received_at_s=self._now_s(),
        )
        if not accepted:
            self.get_logger().warning(
                "Rejected duplicate, out-of-order, or invalid-source voice command",
                throttle_duration_sec=1.0,
            )

    def _publish(self) -> None:
        now = self._now_s()
        decision = self.fusion.resolve(now)
        if (
            decision.perception is not None
            and self.latest_gesture_message is not None
        ):
            output = copy.deepcopy(self.latest_gesture_message)
        else:
            output = AstronautCommand()
            output.header.stamp = self.get_clock().now().to_msg()
            output.header.frame_id = "astronaut_command_fusion"
            output.gesture = "NONE"
            output.astronaut_detected = False
            output.distance_valid = False
            output.depth_status = "UNAVAILABLE"
            output.distance_m = math.nan
            output.direction = "UNKNOWN"
            output.horizontal_angle_deg = math.nan

        output.command = decision.command
        output.confidence = float(decision.confidence)
        output.source = decision.source
        output.decision_reason = decision.reason
        output.safety_stop = decision.safety_stop
        output.distance_required = decision.command in (
            "FOLLOW",
            "COME",
        )
        self.command_publisher.publish(output)
        self.source_publisher.publish(String(data=decision.source))
        self.detail_publisher.publish(String(data=decision.reason))

        report = (decision.command, decision.source, decision.reason)
        if report != self.last_report:
            self.get_logger().info(
                f"Fused command={decision.command}, source={decision.source}, "
                f"reason={decision.reason}"
            )
            self.last_report = report

    @staticmethod
    def _stamp_seconds(stamp):
        value = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
        return value if math.isfinite(value) and value > 0.0 else None

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def main(args=None):
    rclpy.init(args=args)
    node = AstronautCommandFusionNode()
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
