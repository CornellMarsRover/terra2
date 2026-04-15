import argparse
import asyncio
import glob
import math
import os
import signal
import sys
import time

import moteus
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from cmr_msgs.msg import AutonomyDrive

from .swerve_test_utils import (
    ALL_MOTOR_IDS,
    DRIVE_MOTOR_IDS,
    LOGICAL_DRIVE_TO_SERVO,
    LOGICAL_SWERVE_TO_SERVO,
    SWERVE_MOTOR_IDS,
    degrees_to_servo_turns,
    format_expected_servo_targets,
    movement_patterns,
    servo_targets_from_logical,
)


DEFAULT_DRIVE_VELOCITY = 12.0
DEFAULT_SWERVE_ANGLE_DEG = 35.0
DEFAULT_DURATION = 1.5
DEFAULT_STOP_GAP = 1.0
DEFAULT_SETTLE = 0.75


class TopicPatternPublisher(Node):
    def __init__(self):
        super().__init__("swerve_test_topic_publisher")
        self.ackermann_pub = self.create_publisher(AutonomyDrive, "/autonomy/move/ackerman", 10)
        self.point_turn_pub = self.create_publisher(Twist, "/autonomy/move/point_turn", 10)

    def publish_stop(self):
        self.ackermann_pub.publish(AutonomyDrive())
        self.point_turn_pub.publish(Twist())

    def publish_pattern(self, pattern):
        if pattern.topic_mode == "ackerman":
            msg = AutonomyDrive()
            msg.vel = pattern.topic_payload["vel"]
            msg.fl_angle = pattern.topic_payload["fl_angle"]
            msg.fr_angle = pattern.topic_payload["fl_angle"]
            msg.bl_angle = 0.0
            msg.br_angle = 0.0
            self.ackermann_pub.publish(msg)
            self.point_turn_pub.publish(Twist())
        else:
            msg = Twist()
            msg.angular.z = pattern.topic_payload["angular_z"]
            self.point_turn_pub.publish(msg)
            self.ackermann_pub.publish(AutonomyDrive())


class SwerveHardwareTester:
    def __init__(self):
        self.drive_max_torque = 2.0
        self.drive_max_velocity = 80.0
        self.drive_max_accel = 60.0
        self.swerve_max_torque = 2.0
        self.swerve_max_velocity = 30.0
        self.swerve_max_accel = 40.0

        self.loop = asyncio.get_event_loop()
        self.transport = None
        self.servos = {}
        self.fdcanusb_path = None
        self.completed_individual_tests = set()

        self._install_signal_handlers()
        self.loop.run_until_complete(self.initialize())

    def _install_signal_handlers(self):
        def _handler(signum, _frame):
            print(f"\nReceived signal {signum}. Stopping all motors before exit.")
            try:
                self.loop.run_until_complete(self.stop_all())
            finally:
                raise SystemExit(1)

        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)

    async def initialize(self):
        self.fdcanusb_path = self.detect_fdcanusb_path()
        print(f"Initializing moteus transport on {self.fdcanusb_path}")
        self.transport = moteus.Fdcanusb(path=self.fdcanusb_path)
        self.servos = {
            servo_id: moteus.Controller(id=servo_id, transport=self.transport)
            for servo_id in sorted(ALL_MOTOR_IDS.values())
        }
        await self.stop_all()
        await self.transport.cycle([servo.make_rezero() for servo in self.servos.values()])
        await self.stop_all()

    def detect_fdcanusb_path(self):
        if os.path.exists("/dev/fdcanusb"):
            return "/dev/fdcanusb"

        matches = glob.glob("/dev/serial/by-id/*fdcanusb*")
        if matches:
            return sorted(matches)[0]

        if sys.platform != "win32":
            ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
            if ports:
                return sorted(ports)[0]

        return moteus.Fdcanusb.detect_fdcanusb()

    async def stop_all(self):
        if not self.servos:
            return
        await self.transport.cycle([servo.make_stop() for servo in self.servos.values()])

    async def send_drive_velocity(self, servo_id, velocity):
        commands = []
        for current_servo_id, servo in self.servos.items():
            if current_servo_id == servo_id:
                commands.append(
                    servo.make_position(
                        position=math.nan,
                        query=False,
                        velocity=velocity,
                        accel_limit=self.drive_max_accel,
                        velocity_limit=self.drive_max_velocity,
                        maximum_torque=self.drive_max_torque,
                    )
                )
            else:
                commands.append(servo.make_stop())
        await self.transport.cycle(commands)

    async def send_swerve_position(self, servo_id, target_position):
        commands = []
        for current_servo_id, servo in self.servos.items():
            if current_servo_id == servo_id:
                commands.append(
                    servo.make_position(
                        position=target_position,
                        query=False,
                        accel_limit=self.swerve_max_accel,
                        velocity_limit=self.swerve_max_velocity,
                        maximum_torque=self.swerve_max_torque,
                    )
                )
            else:
                commands.append(servo.make_stop())
        await self.transport.cycle(commands)

    async def send_logical_pattern(self, drive_targets, swerve_targets):
        servo_targets = servo_targets_from_logical(*drive_targets, *swerve_targets)
        commands = []
        for servo_id in sorted(self.servos):
            servo = self.servos[servo_id]
            target = servo_targets[servo_id]
            if "velocity" in target:
                commands.append(
                    servo.make_position(
                        position=math.nan,
                        query=False,
                        velocity=target["velocity"],
                        accel_limit=self.drive_max_accel,
                        velocity_limit=self.drive_max_velocity,
                        maximum_torque=self.drive_max_torque,
                    )
                )
            else:
                commands.append(
                    servo.make_position(
                        position=target["position"],
                        query=False,
                        accel_limit=self.swerve_max_accel,
                        velocity_limit=self.swerve_max_velocity,
                        maximum_torque=self.swerve_max_torque,
                    )
                )
        await self.transport.cycle(commands)

    async def run_for(self, duration):
        await asyncio.sleep(duration)

    async def safe_pause(self, gap):
        await self.stop_all()
        await asyncio.sleep(gap)


def require_confirmation(prompt):
    answer = input(f"{prompt} [y/N]: ").strip().lower()
    return answer in {"y", "yes"}


def print_checklist(items):
    print("Manual checklist:")
    for item in items:
        print(f"  - {item}")


def prompt_pass_fail(label):
    print(f"Record result for {label}:")
    answer = input("Mark this step as passed? [y/N]: ").strip().lower()
    return answer in {"y", "yes"}


def run_async(coro):
    return asyncio.get_event_loop().run_until_complete(coro)


def run_drive_motor_test(tester, motor_name, duration, stop_gap, speed_scale):
    servo_id = DRIVE_MOTOR_IDS[motor_name]
    velocity = DEFAULT_DRIVE_VELOCITY * max(0.1, min(speed_scale, 1.0))

    print("\n====================================================")
    print(f"Drive motor test: {motor_name} (servo id={servo_id})")
    print("Expected behavior: only this drive motor should spin.")
    print("Step 1: forward spin")
    print_checklist(
        [
            f"{motor_name} spins in the expected forward direction",
            "No other drive motor moves",
            "No swerve motor moves",
            "Motion is smooth and bounded",
        ]
    )

    if not require_confirmation("Wheels are lifted clear and you are ready to run the forward step"):
        print("Skipped forward step.")
        return False
    run_async(tester.send_drive_velocity(servo_id, velocity))
    run_async(tester.run_for(duration))
    run_async(tester.safe_pause(stop_gap))
    forward_ok = prompt_pass_fail(f"{motor_name} forward step")

    print("\nStep 2: reverse spin")
    print_checklist(
        [
            f"{motor_name} spins in the expected reverse direction",
            "No other drive motor moves",
            "No swerve motor moves",
            "Stop command halts the motor reliably",
        ]
    )
    if not require_confirmation("Ready to run the reverse step"):
        print("Skipped reverse step.")
        return False
    run_async(tester.send_drive_velocity(servo_id, -velocity))
    run_async(tester.run_for(duration))
    run_async(tester.safe_pause(stop_gap))
    reverse_ok = prompt_pass_fail(f"{motor_name} reverse step")

    passed = forward_ok and reverse_ok
    if passed:
        tester.completed_individual_tests.add(motor_name)
    return passed


def run_swerve_motor_test(tester, motor_name, duration, stop_gap):
    servo_id = SWERVE_MOTOR_IDS[motor_name]
    angle = degrees_to_servo_turns(DEFAULT_SWERVE_ANGLE_DEG)

    print("\n====================================================")
    print(f"Swerve motor test: {motor_name} (servo id={servo_id})")
    print("Expected behavior: only this steering module should move.")

    sequence = [
        ("positive angle", angle),
        ("return to zero", 0.0),
        ("negative angle", -angle),
        ("return to zero", 0.0),
    ]

    step_results = []
    for label, target in sequence:
        print(f"\nStep: {label}")
        print_checklist(
            [
                f"{motor_name} moves for the requested {label}",
                "Other swerve motors stay still",
                "Drive motors remain stationary",
                "Motion is smooth and bounded",
            ]
        )
        if not require_confirmation(f"Ready to command {motor_name} to {label}"):
            print("Skipped remaining sequence.")
            return False
        run_async(tester.send_swerve_position(servo_id, target))
        run_async(tester.run_for(duration))
        run_async(tester.safe_pause(stop_gap))
        step_results.append(prompt_pass_fail(f"{motor_name} {label}"))

    passed = all(step_results)
    if passed:
        tester.completed_individual_tests.add(motor_name)
    return passed


def run_pattern_test(tester, pattern, duration, stop_gap, settle_time):
    print("\n====================================================")
    print(format_expected_servo_targets(pattern))
    print_checklist(
        [
            "Wheel/module motion matches the printed expectation",
            "No obviously swapped wheel identity is observed",
            "Inside/outside speed relationship looks correct",
            "Stop command halts all motors reliably",
        ]
    )
    if not require_confirmation("Ready to run this movement pattern"):
        print("Skipped pattern.")
        return False

    zero_drives = (0.0, 0.0, 0.0, 0.0)
    run_async(tester.send_logical_pattern(zero_drives, pattern.swerve_targets))
    run_async(tester.run_for(settle_time))
    run_async(tester.send_logical_pattern(pattern.drive_targets, pattern.swerve_targets))
    run_async(tester.run_for(duration))
    run_async(tester.safe_pause(stop_gap))
    return prompt_pass_fail(pattern.label)


def choose_from_menu(title, entries):
    print(f"\n{title}")
    for index, entry in enumerate(entries, start=1):
        print(f"  {index}. {entry}")
    print("  0. Back")

    while True:
        choice = input("Select an option: ").strip()
        if choice.isdigit():
            number = int(choice)
            if number == 0:
                return None
            if 1 <= number <= len(entries):
                return entries[number - 1]
        print("Please enter a valid menu number.")


def interactive_hardware_menu(tester, duration, stop_gap, speed_scale, force):
    drive_names = list(DRIVE_MOTOR_IDS.keys())
    swerve_names = list(SWERVE_MOTOR_IDS.keys())
    pattern_map = {pattern.label: pattern for pattern in movement_patterns(speed_scale)}

    while True:
        choice = choose_from_menu(
            "Swerve Bench Test Menu",
            [
                "Drive motor tests",
                "Swerve motor tests",
                "Movement pattern tests",
                "Stop all",
                "Exit",
            ],
        )

        if choice is None or choice == "Exit":
            run_async(tester.stop_all())
            return

        if choice == "Drive motor tests":
            motor_name = choose_from_menu("Drive motor selection", drive_names)
            if motor_name:
                run_drive_motor_test(tester, motor_name, duration, stop_gap, speed_scale)
            continue

        if choice == "Swerve motor tests":
            motor_name = choose_from_menu("Swerve motor selection", swerve_names)
            if motor_name:
                run_swerve_motor_test(tester, motor_name, duration, stop_gap)
            continue

        if choice == "Movement pattern tests":
            expected = set(DRIVE_MOTOR_IDS) | set(SWERVE_MOTOR_IDS)
            if not force and tester.completed_individual_tests != expected:
                missing = sorted(expected - tester.completed_individual_tests)
                print("Movement patterns are locked until all 8 individual motor checks pass in this session.")
                print(f"Missing checks: {', '.join(missing)}")
                print("Re-run with --force if you need to bypass the lock.")
                continue
            label = choose_from_menu("Movement pattern selection", list(pattern_map))
            if label:
                run_pattern_test(
                    tester,
                    pattern_map[label],
                    duration,
                    stop_gap,
                    DEFAULT_SETTLE,
                )
            continue

        if choice == "Stop all":
            run_async(tester.stop_all())
            print("All motors stopped.")


def run_topic_validation(duration, speed_scale):
    rclpy.init()
    node = TopicPatternPublisher()
    patterns = movement_patterns(speed_scale)

    try:
        for pattern in patterns:
            print("\n====================================================")
            print("ROS topic validation case")
            print(format_expected_servo_targets(pattern))
            print(f"Publishing to existing controller topics using {pattern.topic_mode}.")
            node.publish_stop()
            time.sleep(0.25)
            node.publish_pattern(pattern)
            time.sleep(duration)
            node.publish_stop()
            time.sleep(0.5)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


def build_parser():
    parser = argparse.ArgumentParser(
        description="Bench-first swerve test utility for direct moteus bring-up and controller-path validation.",
    )
    parser.add_argument(
        "--mode",
        choices=["interactive", "drive", "swerve", "pattern", "topic"],
        default="interactive",
        help="Select test mode. Interactive launches the CLI menu.",
    )
    parser.add_argument("--motor", choices=sorted(ALL_MOTOR_IDS), help="Motor name for drive/swerve mode.")
    parser.add_argument(
        "--pattern",
        choices=[pattern.key for pattern in movement_patterns(1.0)],
        help="Pattern key for pattern mode.",
    )
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION, help="Command hold time in seconds.")
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=0.35,
        help="Bench-safe scale factor between 0 and 1 for pattern and drive velocity defaults.",
    )
    parser.add_argument("--force", action="store_true", help="Allow movement-pattern mode without all individual checks.")
    return parser


def main(args=None):
    parser = build_parser()
    parsed = parser.parse_args(args=args)

    if parsed.mode == "topic":
        run_topic_validation(parsed.duration, parsed.speed_scale)
        return

    tester = SwerveHardwareTester()
    print("Bench-first swerve tester ready.")
    print("Safety reminder: keep wheels off the ground unless you are intentionally running a supervised slow-roll check.")

    try:
        if parsed.mode == "interactive":
            interactive_hardware_menu(
                tester,
                parsed.duration,
                DEFAULT_STOP_GAP,
                parsed.speed_scale,
                parsed.force,
            )
        elif parsed.mode == "drive":
            if parsed.motor not in DRIVE_MOTOR_IDS:
                parser.error("--motor must be one of the drive motors in drive mode")
            run_drive_motor_test(tester, parsed.motor, parsed.duration, DEFAULT_STOP_GAP, parsed.speed_scale)
        elif parsed.mode == "swerve":
            if parsed.motor not in SWERVE_MOTOR_IDS:
                parser.error("--motor must be one of the swerve motors in swerve mode")
            run_swerve_motor_test(tester, parsed.motor, parsed.duration, DEFAULT_STOP_GAP)
        elif parsed.mode == "pattern":
            if not parsed.pattern:
                parser.error("--pattern is required in pattern mode")
            patterns = {pattern.key: pattern for pattern in movement_patterns(parsed.speed_scale)}
            pattern = patterns[parsed.pattern]
            expected = set(DRIVE_MOTOR_IDS) | set(SWERVE_MOTOR_IDS)
            if not parsed.force and tester.completed_individual_tests != expected:
                print("Pattern mode requires all individual checks in the current session. Re-run with --force to override.")
                raise SystemExit(2)
            run_pattern_test(tester, pattern, parsed.duration, DEFAULT_STOP_GAP, DEFAULT_SETTLE)
    finally:
        run_async(tester.stop_all())


if __name__ == "__main__":
    main()
