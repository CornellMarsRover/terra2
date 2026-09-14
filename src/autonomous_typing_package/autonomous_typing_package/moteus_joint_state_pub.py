#!/usr/bin/env python3

import argparse
import asyncio
import math
import threading
import time

import moteus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_MOTOR_IDS = {
    "base_joint": 9,
    "shoulder_joint": 10,
    "elbow_joint": 11,
    "wrist_rotate": 12,
    "wrist_twist": 13,
    "wrist_rotate_two": 14,
}


def build_parser():
    parser = argparse.ArgumentParser(
        description="Publish /joint_states for the arm by reading moteus encoder positions."
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=0.5)
    return parser


def make_query_resolution():
    qr = moteus.QueryResolution()
    for field, resolution in [
        ("position", moteus.F32),
        ("velocity", moteus.F32),
        ("fault", moteus.INT8),
    ]:
        if hasattr(qr, field):
            setattr(qr, field, resolution)
    return qr


def motor_rev_to_joint_rad(joint_name, motor_position):
    if joint_name in ("base_joint", "shoulder_joint", "elbow_joint"):
        return motor_position * 2.0 * math.pi / 100.0
    return -motor_position * 2.0 * math.pi / 50.0


class MoteusJointStatePublisher(Node):
    def __init__(self, args):
        super().__init__("moteus_joint_state_pub")
        self.args = args
        self.latest_msg = None
        self.latest_error = None
        self.lock = threading.Lock()
        self.running = True
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.timer = self.create_timer(1.0 / args.rate, self.publish_latest)
        self.thread = threading.Thread(target=self.reader_thread, daemon=True)
        self.thread.start()

    def reader_thread(self):
        asyncio.run(self.reader_loop())

    async def reader_loop(self):
        transport = moteus.Fdcanusb(self.args.port)
        qr = make_query_resolution()
        controllers = {
            joint: moteus.Controller(
                id=motor_id,
                transport=transport,
                query_resolution=qr,
            )
            for joint, motor_id in JOINT_MOTOR_IDS.items()
        }

        while self.running and rclpy.ok():
            msg = JointState()
            msg.name = list(JOINT_MOTOR_IDS.keys())

            positions = []
            velocities = []
            try:
                for joint, controller in controllers.items():
                    result = await asyncio.wait_for(
                        controller.query(),
                        timeout=self.args.timeout,
                    )
                    fault = result.values.get(moteus.Register.FAULT)
                    if fault not in (None, 0):
                        raise RuntimeError(f"{joint} fault={fault}")

                    motor_position = result.values.get(moteus.Register.POSITION)
                    motor_velocity = result.values.get(moteus.Register.VELOCITY, 0.0)
                    if motor_position is None:
                        raise RuntimeError(f"{joint} query did not include position")

                    positions.append(motor_rev_to_joint_rad(joint, motor_position))
                    velocities.append(motor_rev_to_joint_rad(joint, motor_velocity or 0.0))

                msg.position = positions
                msg.velocity = velocities
                with self.lock:
                    self.latest_msg = msg
                    self.latest_error = None
            except Exception as exc:
                with self.lock:
                    self.latest_error = exc

            await asyncio.sleep(max(0.0, 1.0 / self.args.rate))

    def publish_latest(self):
        with self.lock:
            msg = self.latest_msg
            error = self.latest_error

        if error is not None:
            self.get_logger().warn(f"moteus joint state read failed: {error}", throttle_duration_sec=1.0)
            return
        if msg is None:
            return

        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    parsed = build_parser().parse_args(args=args)
    rclpy.init()
    node = MoteusJointStatePublisher(parsed)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
