#!/usr/bin/env python3
"""ROS adapter for the hardware-independent astronaut mission core."""

from __future__ import annotations

import rclpy
from cmr_msgs.msg import AstronautCommand, DriveCommand
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from astronaut_autonomy.mission_core import (
    AstronautMissionStateMachine,
    ManipulationAction,
    ManipulationStatus,
    MissionConfig,
    PerceptionSample,
)
from astronaut_autonomy.tracking_control import TrackingConfig, VelocityCommand


class AstronautMissionNode(Node):
    def __init__(self):
        super().__init__("astronaut_mission")
        self._declare_parameters()

        self.drive_rps = abs(float(self.get_parameter("drive_rps").value))
        self.enable_drive_adapter = bool(
            self.get_parameter("enable_drive_adapter").value
        )
        mission_config = MissionConfig(
            command_debounce_s=float(
                self.get_parameter("command_debounce_s").value
            ),
            perception_timeout_s=float(
                self.get_parameter("perception_timeout_s").value
            ),
            manipulation_timeout_s=float(
                self.get_parameter("manipulation_timeout_s").value
            ),
            minimum_command_confidence=float(
                self.get_parameter("minimum_command_confidence").value
            ),
            enforce_competition_sequence=bool(
                self.get_parameter("enforce_competition_sequence").value
            ),
        )
        tracking_config = TrackingConfig(
            follow_distance_m=float(
                self.get_parameter("follow_distance_m").value
            ),
            come_distance_m=float(
                self.get_parameter("come_distance_m").value
            ),
            distance_deadband_m=float(
                self.get_parameter("distance_deadband_m").value
            ),
            bearing_deadband_deg=float(
                self.get_parameter("bearing_deadband_deg").value
            ),
            rotate_in_place_deg=float(
                self.get_parameter("rotate_in_place_deg").value
            ),
            linear_gain=float(self.get_parameter("linear_gain").value),
            angular_gain=float(self.get_parameter("angular_gain").value),
            maximum_linear=float(self.get_parameter("maximum_linear").value),
            maximum_angular=float(
                self.get_parameter("maximum_angular").value
            ),
        )
        self.mission = AstronautMissionStateMachine(
            mission_config=mission_config,
            tracking_config=tracking_config,
        )
        self.emergency_stop = False
        self.last_state_report = None

        self.proposed_drive_publisher = self.create_publisher(
            DriveCommand, "/autonomy/astronaut/drive_command", 10
        )
        self.drive_adapter_publisher = None
        if self.enable_drive_adapter:
            self.drive_adapter_publisher = self.create_publisher(
                DriveCommand, "/cmd_vel/autonomy", 10
            )
            self.get_logger().warning(
                "Astronaut drive adapter ENABLED on /cmd_vel/autonomy; do not "
                "run the waypoint controller on the same topic"
            )
        else:
            self.get_logger().info(
                "Astronaut drive adapter disabled; commands are proposal-only"
            )

        self.state_publisher = self.create_publisher(
            String, "/autonomy/astronaut/mission_state", 10
        )
        self.detail_publisher = self.create_publisher(
            String, "/autonomy/astronaut/mission_detail", 10
        )
        self.expected_command_publisher = self.create_publisher(
            String, "/autonomy/astronaut/expected_command", 10
        )
        self.mission_complete_publisher = self.create_publisher(
            Bool, "/autonomy/astronaut/mission_complete", 10
        )
        self.manipulation_publisher = self.create_publisher(
            String, "/autonomy/astronaut/manipulation/request", 10
        )
        self.create_subscription(
            AstronautCommand,
            "/autonomy/astronaut/perception",
            self._perception_callback,
            10,
        )
        self.create_subscription(
            Bool, "/cmd_vel/estop", self._estop_callback, 10
        )
        self.create_subscription(
            String,
            "/autonomy/astronaut/manipulation/status",
            self._manipulation_callback,
            10,
        )
        self.create_service(
            Trigger, "/autonomy/astronaut/reset_error", self._reset_error
        )
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        if control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        self.timer = self.create_timer(1.0 / control_rate_hz, self._control_tick)

    def _declare_parameters(self) -> None:
        defaults = {
            "control_rate_hz": 10.0,
            "command_debounce_s": 0.35,
            "perception_timeout_s": 0.75,
            "manipulation_timeout_s": 30.0,
            "minimum_command_confidence": 0.50,
            "enforce_competition_sequence": True,
            "follow_distance_m": 2.0,
            "come_distance_m": 1.25,
            "distance_deadband_m": 0.15,
            "bearing_deadband_deg": 4.0,
            "rotate_in_place_deg": 30.0,
            "linear_gain": 0.55,
            "angular_gain": 1.2,
            "maximum_linear": 0.50,
            "maximum_angular": 0.60,
            "drive_rps": 4.0,
            "enable_drive_adapter": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _perception_callback(self, message: AstronautCommand) -> None:
        if message.source not in {"GESTURE", "VOICE", "FUSED", "FUSION"}:
            self.get_logger().warning(
                f"Ignoring astronaut command with invalid source: {message.source!r}",
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
            source=message.source,
            decision_reason=message.decision_reason,
            safety_stop=bool(message.safety_stop),
        )
        accepted = self.mission.observe_perception(sample, self._now_s())
        if (
            accepted and self.mission.state.value == "STAYING"
        ) or not sample.valid_target:
            # Do not wait for the periodic control callback to stop after a
            # stale range, invalid depth sample, or lost astronaut track.
            self._publish_drive(VelocityCommand(reason="invalid_perception"))

    def _estop_callback(self, message: Bool) -> None:
        self.emergency_stop = bool(message.data)
        if self.emergency_stop:
            # Publish an immediate zero in addition to the drive mux's own
            # independently enforced emergency stop.
            self._publish_drive(VelocityCommand(reason="emergency_stop"))

    def _manipulation_callback(self, message: String) -> None:
        try:
            status = ManipulationStatus(message.data.strip().upper())
        except ValueError:
            self.get_logger().warning(
                f"Ignoring unknown manipulation status: {message.data!r}"
            )
            return
        self.mission.observe_manipulation(status, self._now_s())

    def _control_tick(self) -> None:
        decision = self.mission.tick(
            self._now_s(), emergency_stop=self.emergency_stop
        )
        self._publish_drive(decision.velocity)
        if decision.manipulation_action != ManipulationAction.NONE:
            self.manipulation_publisher.publish(
                String(data=decision.manipulation_action.value)
            )

        report = (decision.state.value, decision.reason)
        self.state_publisher.publish(String(data=decision.state.value))
        self.detail_publisher.publish(String(data=decision.reason))
        self.expected_command_publisher.publish(
            String(data=decision.expected_command)
        )
        self.mission_complete_publisher.publish(
            Bool(data=decision.mission_complete)
        )
        if report != self.last_state_report:
            self.get_logger().info(
                f"Astronaut mission: {decision.state.value} "
                f"({decision.reason})"
            )
            self.last_state_report = report

    def _publish_drive(self, velocity: VelocityCommand) -> None:
        message = DriveCommand(
            vx=float(velocity.linear_x),
            vy=0.0,
            omega=float(velocity.angular_z),
            speed_rps=self.drive_rps,
        )
        self.proposed_drive_publisher.publish(message)
        if self.drive_adapter_publisher is not None:
            self.drive_adapter_publisher.publish(message)

    def _reset_error(self, _request, response):
        if self.emergency_stop:
            response.success = False
            response.message = "cannot reset while emergency stop is active"
            return response
        response.success = self.mission.reset_error()
        response.message = (
            "mission error reset"
            if response.success
            else "mission is not in ERROR"
        )
        return response

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def destroy_node(self):
        self._publish_drive(VelocityCommand(reason="shutdown"))
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AstronautMissionNode()
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
