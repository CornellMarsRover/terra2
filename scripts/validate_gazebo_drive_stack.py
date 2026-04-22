#!/usr/bin/env python3

"""Validate the Gazebo rover drive stack end to end.

Run this inside the ROS/Gazebo environment after `gzserver`, the `drives`
entity, and `gazebo_drives.launch.py` are already running.
"""

from __future__ import annotations

import json
import math
import socket
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import rclpy
from cmr_msgs.msg import AutonomyDrive
from geometry_msgs.msg import Twist
from rclpy.node import Node


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


class DriveValidator(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_drive_validator")
        self.cmd_vel_drives_pub = self.create_publisher(Twist, "/cmd_vel_drives", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.point_turn_pub = self.create_publisher(Twist, "/autonomy/move/point_turn", 10)
        self.ackermann_pub = self.create_publisher(AutonomyDrive, "/autonomy/move/ackerman", 10)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_target = ("127.0.0.1", 5010)
        self.rate_hz = 10.0
        self.model_name = "drives"
        self.report_dir = Path("logs/gazebo_validation")
        self.report_dir.mkdir(parents=True, exist_ok=True)

    def _run_cmd(self, command: list[str], timeout_s: float = 5.0, retries: int = 3) -> str:
        last_error: Exception | None = None
        for attempt in range(1, retries + 1):
            try:
                completed = subprocess.run(
                    command,
                    check=True,
                    text=True,
                    capture_output=True,
                    timeout=timeout_s,
                )
                return completed.stdout.strip()
            except subprocess.TimeoutExpired as exc:
                last_error = exc
                self.get_logger().warn(
                    f"Command timed out (attempt {attempt}/{retries}): {' '.join(command)}"
                )
                if command[:2] == ["gz", "model"]:
                    subprocess.run(
                        ["pkill", "-f", f"gz model -m {self.model_name}"],
                        check=False,
                        text=True,
                        capture_output=True,
                    )
                time.sleep(0.5)
            except subprocess.CalledProcessError as exc:
                last_error = exc
                self.get_logger().warn(
                    f"Command failed (attempt {attempt}/{retries}): {' '.join(command)} "
                    f"stderr={exc.stderr.strip()}"
                )
                time.sleep(0.5)
        raise RuntimeError(f"Command failed after {retries} attempts: {' '.join(command)} ({last_error})")

    def get_pose(self) -> Pose:
        output = self._run_cmd(["gz", "model", "-m", self.model_name, "-p"])
        x, y, z, roll, pitch, yaw = [float(value) for value in output.split()]
        return Pose(x, y, z, roll, pitch, yaw)

    def reset_pose(self) -> Pose:
        self._run_cmd(
            [
                "gz",
                "model",
                "-m",
                self.model_name,
                "-x",
                "0",
                "-y",
                "0",
                "-z",
                "0",
                "-R",
                "0",
                "-P",
                "0",
                "-Y",
                "0",
            ]
        )
        time.sleep(0.5)
        return self.get_pose()

    def _publish_twist(self, publisher, linear_x: float, angular_z: float, duration_s: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        end_time = time.time() + duration_s
        period = 1.0 / self.rate_hz
        while time.time() < end_time:
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def _publish_ackermann(self, vel: float, angle_deg: float, duration_s: float) -> None:
        msg = AutonomyDrive()
        msg.vel = vel
        msg.fl_angle = angle_deg
        msg.fr_angle = angle_deg
        msg.bl_angle = angle_deg
        msg.br_angle = angle_deg
        end_time = time.time() + duration_s
        period = 1.0 / self.rate_hz
        while time.time() < end_time:
            self.ackermann_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def _send_udp_packet(self, packet: bytes, duration_s: float) -> None:
        end_time = time.time() + duration_s
        period = 1.0 / self.rate_hz
        while time.time() < end_time:
            self.udp_sock.sendto(packet, self.udp_target)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def _neutral_packet(self) -> bytes:
        return bytes([127, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00"

    def neutralize(self) -> None:
        self._send_udp_packet(self._neutral_packet(), 0.4)
        self._publish_twist(self.cmd_vel_drives_pub, 0.0, 0.0, 0.4)
        self._publish_twist(self.cmd_vel_pub, 0.0, 0.0, 0.4)
        self._publish_twist(self.point_turn_pub, 0.0, 0.0, 0.3)
        self._publish_ackermann(0.0, 0.0, 0.3)
        time.sleep(0.6)

    def _result(self, start: Pose, end: Pose) -> dict[str, float]:
        dx = end.x - start.x
        dy = end.y - start.y
        dyaw = wrap_angle(end.yaw - start.yaw)
        return {
            "start_x": start.x,
            "start_y": start.y,
            "start_yaw": start.yaw,
            "end_x": end.x,
            "end_y": end.y,
            "end_yaw": end.yaw,
            "dx": dx,
            "dy": dy,
            "distance": math.hypot(dx, dy),
            "dyaw": dyaw,
        }

    def _evaluate(self, name: str, result: dict[str, float]) -> tuple[bool, str]:
        distance = result["distance"]
        dyaw = result["dyaw"]
        dx = result["dx"]

        if name in {"cmd_vel_drives_forward", "cmd_vel_forward", "udp_forward"}:
            passed = dx > 0.45 and abs(dyaw) < 0.2
            return passed, f"expected forward translation, got dx={dx:.3f}, dyaw={dyaw:.3f}"
        if name in {"cmd_vel_drives_reverse", "udp_reverse"}:
            passed = dx < -0.45 and abs(dyaw) < 0.2
            return passed, f"expected reverse translation, got dx={dx:.3f}, dyaw={dyaw:.3f}"
        if name in {"cmd_vel_drives_turn_left", "udp_turn_left"}:
            passed = dyaw > 0.35 and distance < 0.2
            return passed, f"expected positive yaw with little translation, got dist={distance:.3f}, dyaw={dyaw:.3f}"
        if name in {"cmd_vel_drives_turn_right", "point_turn_right", "udp_turn_right"}:
            passed = dyaw < -0.35 and distance < 0.2
            return passed, f"expected negative yaw with little translation, got dist={distance:.3f}, dyaw={dyaw:.3f}"
        if name in {"cmd_vel_drives_arc_left", "ackermann_left", "udp_arc_left"}:
            passed = dx > 0.35 and dyaw > 0.2
            return passed, f"expected forward arc-left motion, got dx={dx:.3f}, dyaw={dyaw:.3f}"
        if name in {"udp_neutral", "udp_lateral_only", "udp_right_y_only"}:
            passed = distance < 0.12 and abs(dyaw) < 0.12
            return passed, f"expected no motion, got dist={distance:.3f}, dyaw={dyaw:.3f}"
        raise ValueError(f"No expectation defined for {name}")

    def _exercise(self, name: str, action) -> dict[str, object]:
        self.neutralize()
        reset_pose = self.reset_pose()
        start_pose = reset_pose
        action()
        time.sleep(0.2)
        self.neutralize()
        end_pose = self.get_pose()
        result = self._result(start_pose, end_pose)
        passed, summary = self._evaluate(name, result)
        return {
            "name": name,
            "passed": passed,
            "summary": summary,
            "reset_pose": reset_pose.__dict__,
            "result": result,
        }

    def run_suite(self) -> dict[str, object]:
        tests: list[tuple[str, callable]] = [
            (
                "cmd_vel_drives_forward",
                lambda: self._publish_twist(self.cmd_vel_drives_pub, 0.8, 0.0, 2.0),
            ),
            (
                "cmd_vel_forward",
                lambda: self._publish_twist(self.cmd_vel_pub, 0.8, 0.0, 2.0),
            ),
            (
                "cmd_vel_drives_reverse",
                lambda: self._publish_twist(self.cmd_vel_drives_pub, -0.8, 0.0, 2.0),
            ),
            (
                "cmd_vel_drives_turn_left",
                lambda: self._publish_twist(self.cmd_vel_drives_pub, 0.0, 0.8, 2.0),
            ),
            (
                "cmd_vel_drives_turn_right",
                lambda: self._publish_twist(self.cmd_vel_drives_pub, 0.0, -0.8, 2.0),
            ),
            (
                "cmd_vel_drives_arc_left",
                lambda: self._publish_twist(self.cmd_vel_drives_pub, 0.7, 0.5, 2.0),
            ),
            (
                "point_turn_right",
                lambda: self._publish_twist(self.point_turn_pub, 0.0, -0.8, 2.0),
            ),
            (
                "ackermann_left",
                lambda: self._publish_ackermann(0.7, 18.0, 2.0),
            ),
            (
                "udp_neutral",
                lambda: self._send_udp_packet(self._neutral_packet(), 2.0),
            ),
            (
                "udp_forward",
                lambda: self._send_udp_packet(
                    bytes([127, 0, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00",
                    2.0,
                ),
            ),
            (
                "udp_reverse",
                lambda: self._send_udp_packet(
                    bytes([127, 255, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00",
                    2.0,
                ),
            ),
            (
                "udp_turn_left",
                lambda: self._send_udp_packet(
                    bytes([127, 127, 255, 127, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00",
                    2.0,
                ),
            ),
            (
                "udp_turn_right",
                lambda: self._send_udp_packet(
                    bytes([127, 127, 0, 127, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00",
                    2.0,
                ),
            ),
            (
                "udp_arc_left",
                lambda: self._send_udp_packet(
                    bytes([127, 0, 255, 127, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00",
                    2.0,
                ),
            ),
            (
                "udp_lateral_only",
                lambda: self._send_udp_packet(
                    bytes([255, 127, 127, 127, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00",
                    2.0,
                ),
            ),
            (
                "udp_right_y_only",
                lambda: self._send_udp_packet(
                    bytes([127, 127, 127, 255, 0, 0, 0, 0, 0, 0, 0, 0]) + b"neutral\x00",
                    2.0,
                ),
            ),
        ]

        started_at = datetime.now().isoformat()
        results = []
        for name, action in tests:
            self.get_logger().info(f"Running validation test: {name}")
            results.append(self._exercise(name, action))

        report = {
            "started_at": started_at,
            "finished_at": datetime.now().isoformat(),
            "model_name": self.model_name,
            "tests": results,
            "all_passed": all(test["passed"] for test in results),
        }
        report_path = self.report_dir / f"gazebo_drive_validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2))
        print(f"\nReport written to {report_path}")
        return report


def main() -> int:
    rclpy.init()
    node = DriveValidator()
    try:
        report = node.run_suite()
        return 0 if report["all_passed"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
