#!/usr/bin/env python3
"""Closed-loop ROS simulator for the astronaut mission pipeline."""

from __future__ import annotations

import json
import math

import rclpy
from cmr_msgs.msg import AstronautCommand, DriveCommand
from geometry_msgs.msg import Pose2D as Pose2DMessage
from rclpy.node import Node
from std_msgs.msg import Bool, String

from astronaut_autonomy.competition_simulation import (
    CompetitionScenario,
    CompetitionSimulationConfig,
    write_report_json,
    write_trajectory_csv,
)
from astronaut_autonomy.mission_core import ManipulationAction, MissionState
from astronaut_autonomy.tracking_control import VelocityCommand


class AstronautMissionSimulator(Node):
    """Publishes perception derived from simulated world and rover positions."""

    def __init__(self):
        super().__init__("astronaut_mission_simulator")
        self._declare_parameters()
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self.dt_s = 1.0 / rate
        self.scenario = CompetitionScenario(self._simulation_config())
        self.started_s = self._now_s()
        self.mission_state = MissionState.IDLE
        self.mission_reason = "waiting_for_command"
        self.expected_command = "FOLLOW"
        self.mission_complete = False
        self.last_drive = VelocityCommand(reason="simulator_start")
        self.last_observation = None
        self.last_phase = None
        self.report_written = False

        self.perception_publisher = self.create_publisher(
            AstronautCommand, "/autonomy/astronaut/perception", 10
        )
        self.estop_publisher = self.create_publisher(Bool, "/cmd_vel/estop", 10)
        self.pose_publisher = self.create_publisher(
            Pose2DMessage, "/autonomy/astronaut/sim/rover_pose", 10
        )
        self.manipulation_status_publisher = self.create_publisher(
            String, "/autonomy/astronaut/manipulation/status", 10
        )
        self.create_subscription(
            DriveCommand,
            "/autonomy/astronaut/drive_command",
            self._drive_callback,
            10,
        )
        self.create_subscription(
            String,
            "/autonomy/astronaut/mission_state",
            self._state_callback,
            10,
        )
        self.create_subscription(
            String,
            "/autonomy/astronaut/mission_detail",
            self._detail_callback,
            10,
        )
        self.create_subscription(
            String,
            "/autonomy/astronaut/expected_command",
            self._expected_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/autonomy/astronaut/mission_complete",
            self._complete_callback,
            10,
        )
        self.create_subscription(
            String,
            "/autonomy/astronaut/manipulation/request",
            self._manipulation_request,
            10,
        )
        self.timer = self.create_timer(self.dt_s, self._tick)
        self.get_logger().warning(
            "Closed-loop mock rover active. Physical output remains disabled; "
            "this node does not publish /cmd_vel/autonomy."
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "publish_rate_hz": 10.0,
            "maximum_duration_s": 90.0,
            "rover_maximum_speed_mps": 2.0,
            "rover_maximum_turn_rate_rad_s": 1.4,
            "initial_astronaut_x_m": 6.0,
            "initial_astronaut_y_m": 1.5,
            "astronaut_follow_speed_mps": 0.25,
            "follow_walk_duration_s": 8.0,
            "stay_duration_s": 2.0,
            "remote_command_distance_m": 21.0,
            "astronaut_relocation_speed_mps": 4.0,
            "out_of_order_duration_s": 0.8,
            "manipulation_duration_s": 0.75,
            "range_noise_std_m": 0.0,
            "bearing_noise_std_deg": 0.0,
            "random_seed": 2027,
            "inject_failures": True,
            "minimum_safe_distance_m": 0.75,
            "trajectory_csv": "/tmp/astronaut_sim/trajectory.csv",
            "report_json": "/tmp/astronaut_sim/report.json",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _simulation_config(self) -> CompetitionSimulationConfig:
        def value(name):
            return self.get_parameter(name).value

        return CompetitionSimulationConfig(
            time_step_s=1.0 / float(value("publish_rate_hz")),
            maximum_duration_s=float(value("maximum_duration_s")),
            rover_maximum_speed_mps=float(value("rover_maximum_speed_mps")),
            rover_maximum_turn_rate_rad_s=float(
                value("rover_maximum_turn_rate_rad_s")
            ),
            initial_astronaut_x_m=float(value("initial_astronaut_x_m")),
            initial_astronaut_y_m=float(value("initial_astronaut_y_m")),
            astronaut_follow_speed_mps=float(value("astronaut_follow_speed_mps")),
            follow_walk_duration_s=float(value("follow_walk_duration_s")),
            stay_duration_s=float(value("stay_duration_s")),
            remote_command_distance_m=float(value("remote_command_distance_m")),
            astronaut_relocation_speed_mps=float(
                value("astronaut_relocation_speed_mps")
            ),
            out_of_order_duration_s=float(value("out_of_order_duration_s")),
            manipulation_duration_s=float(value("manipulation_duration_s")),
            range_noise_std_m=float(value("range_noise_std_m")),
            bearing_noise_std_deg=float(value("bearing_noise_std_deg")),
            random_seed=int(value("random_seed")),
            inject_failures=bool(value("inject_failures")),
            minimum_safe_distance_m=float(value("minimum_safe_distance_m")),
        )

    def _tick(self) -> None:
        now_s = self._now_s()
        elapsed_s = now_s - self.started_s
        if self.report_written:
            return

        if self.last_observation is not None:
            self.scenario.check_safety(
                self.last_observation, self.last_drive, self.mission_state
            )
            applied_linear, applied_angular = self.scenario.rover.step(
                self.last_drive,
                self.dt_s,
                emergency_stop=self.last_observation.emergency_stop,
            )
            self.scenario.check_stay_position()
            self.scenario.log(
                elapsed_s,
                self.last_observation,
                self.mission_state,
                self.last_drive,
                applied_linear,
                applied_angular,
            )

        self.scenario.advance_world(elapsed_s, self.dt_s)
        self.scenario.update_phase(
            elapsed_s,
            self.mission_state,
            self.expected_command,
            self.mission_complete,
        )
        observation = self.scenario.observation(elapsed_s)
        self._publish_observation(observation)
        self.estop_publisher.publish(Bool(data=observation.emergency_stop))
        self._publish_pose()
        self.last_observation = observation

        manipulation_status = self.scenario.manipulation_status(elapsed_s)
        if manipulation_status is not None:
            self.manipulation_status_publisher.publish(
                String(data=manipulation_status.value)
            )

        if observation.phase != self.last_phase:
            self.get_logger().info(f"Simulator phase: {observation.phase}")
            self.last_phase = observation.phase

        if self.scenario.complete:
            self._finish(elapsed_s)
        elif elapsed_s >= self.scenario.config.maximum_duration_s:
            self._finish(elapsed_s, "simulation_timeout")

    def _publish_observation(self, sample) -> None:
        message = AstronautCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "simulated_zed_camera_frame"
        message.gesture = sample.gesture
        message.command = sample.command
        message.confidence = sample.confidence
        message.astronaut_detected = sample.astronaut_detected
        message.distance_required = sample.command in ("FOLLOW", "FETCH", "COME")
        message.distance_valid = sample.distance_valid
        message.depth_status = sample.depth_status
        message.distance_m = sample.distance_m
        message.direction = self._direction(sample.bearing_deg)
        message.horizontal_angle_deg = sample.bearing_deg
        message.depth_samples = 24 if sample.distance_valid else 0
        if sample.distance_valid:
            angle = math.radians(sample.bearing_deg)
            message.position.x = sample.distance_m * math.cos(angle)
            message.position.y = sample.distance_m * math.sin(angle)
            message.position.z = 0.0
        self.perception_publisher.publish(message)

    def _publish_pose(self) -> None:
        pose = self.scenario.rover.pose
        self.pose_publisher.publish(
            Pose2DMessage(x=pose.x, y=pose.y, theta=pose.yaw_rad)
        )

    def _drive_callback(self, message: DriveCommand) -> None:
        self.last_drive = VelocityCommand(
            linear_x=float(message.vx),
            angular_z=float(message.omega),
            reason="ros_drive_command",
        )

    def _state_callback(self, message: String) -> None:
        try:
            state = MissionState(message.data)
        except ValueError:
            self.get_logger().warning(f"Unknown mission state: {message.data!r}")
            return
        if state != self.mission_state:
            state_to_command = {
                MissionState.FOLLOWING: "FOLLOW",
                MissionState.FETCHING: "FETCH",
                MissionState.COMING: "COME",
                MissionState.GIVING: "GIVE",
            }
            command = state_to_command.get(state)
            if state == MissionState.STAYING and self.scenario.phase == "STAY":
                command = "STAY"
            if command is not None:
                self.scenario.note_command(command)
        self.mission_state = state
        self.scenario.note_state(
            self._now_s() - self.started_s, state, self.mission_reason
        )

    def _detail_callback(self, message: String) -> None:
        self.mission_reason = message.data

    def _expected_callback(self, message: String) -> None:
        self.expected_command = message.data

    def _complete_callback(self, message: Bool) -> None:
        self.mission_complete = bool(message.data)

    def _manipulation_request(self, message: String) -> None:
        request = message.data.strip().upper()
        if request == "CANCEL":
            self.scenario.cancel_manipulation()
            self.manipulation_status_publisher.publish(String(data="CANCELED"))
            return
        try:
            action = ManipulationAction(request)
        except ValueError:
            self.get_logger().warning(
                f"Ignoring unknown manipulation request: {message.data!r}"
            )
            return
        if action not in (
            ManipulationAction.START_FETCH,
            ManipulationAction.START_GIVE,
        ):
            return
        elapsed_s = self._now_s() - self.started_s
        self.scenario.start_manipulation(action, elapsed_s)
        self.manipulation_status_publisher.publish(String(data="RUNNING"))

    def _finish(self, elapsed_s: float, failure_reason: str = "") -> None:
        report = self.scenario.report(elapsed_s, failure_reason)
        write_trajectory_csv(
            str(self.get_parameter("trajectory_csv").value),
            self.scenario.trajectory,
        )
        write_report_json(str(self.get_parameter("report_json").value), report)
        self.get_logger().info(
            "Simulation result: " + json.dumps(report.to_dict(), sort_keys=True)
        )
        self.report_written = True
        self.timer.cancel()

    @staticmethod
    def _direction(bearing_deg: float) -> str:
        if not math.isfinite(bearing_deg):
            return "UNKNOWN"
        if bearing_deg > 10.0:
            return "LEFT"
        if bearing_deg < -10.0:
            return "RIGHT"
        return "CENTER"

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0


def main(args=None):
    rclpy.init(args=args)
    node = AstronautMissionSimulator()
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
