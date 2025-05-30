#!/usr/bin/env python3
"""
udp_cmd_vel_node.py  (version with acceleration limiting)

* Listens for key‑state strings over UDP.
* Converts them to target velocities using KEY_TO_VEL.
* Ramps current velocity toward the target, respecting
  max linear accel 0.2 m/s² and max angular accel 0.1 rad/s².
* Publishes a Twist at 10 Hz.
"""

import argparse
import select
import socket

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ------------------------------------------------------------- #
KEY_TO_VEL = {
    "w": ( 0.5,  0.0,  0.0),
    "s": (-0.5,  0.0,  0.0),
    "a": ( 0.0,  0.5,  0.0),
    "d": ( 0.0, -0.5,  0.0),
    "z": ( 0.0,  0.0,  0.4),
    "x": ( 0.0,  0.0, -0.4),
    "i": ( 0.1,  0.0,  0.0),
    "k": (-0.1,  0.0,  0.0),
    "j": ( 0.0,  0.1,  0.0),
    "l": ( 0.0, -0.1,  0.0),
    "n": ( 0.0,  0.0,  0.1),
    "m": ( 0.0,  0.0, -0.1),
}

POS_X, NEG_X = {"w", "i"}, {"s", "k"}
POS_Y, NEG_Y = {"a", "j"}, {"d", "l"}
POS_Z, NEG_Z = {"z", "n"}, {"x", "m"}

TICK_HZ = 10.0          # publish rate
DT       = 1.0 / TICK_HZ
MAX_DV   = 0.2 * DT     # 0.02  m/s per tick
MAX_DW   = 0.1 * DT     # 0.01 rad/s per tick


def clamp_delta(delta: float, limit: float) -> float:
    """Limit |delta| ≤ limit."""
    if delta > limit:
        return limit
    if delta < -limit:
        return -limit
    return delta


class UdpTeleopNode(Node):
    def __init__(self, port: int, topic: str):
        super().__init__("udp_teleop_node")
        self.pub = self.create_publisher(Twist, topic, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

        self.latest_keys = set()     # most recent key‑set from UDP
        self.vel_x = 0.0             # current commanded velocities
        self.vel_y = 0.0
        self.ang_z = 0.0

        self.timer = self.create_timer(DT, self.tick)
        self.get_logger().info(
            f"UDP tele‑op: listening on :{port}, publishing '{topic}' @ {TICK_HZ} Hz "
            "(accel‑limited)"
        )

    # --------------------------------------------------------- #
    def tick(self):
        # --- read all available UDP packets, keep only last
        while True:
            ready, _, _ = select.select([self.sock], [], [], 0.0)
            if not ready:
                break
            data, _ = self.sock.recvfrom(1024)
            self.latest_keys = set(data.decode("utf-8").strip().lower())

        # --- determine target velocity from keys ----------------
        tgt_x = tgt_y = tgt_z = 0.0

        # linear x
        if self.latest_keys & POS_X and self.latest_keys & NEG_X:
            tgt_x = 0.0
        else:
            tgt_x = sum(KEY_TO_VEL[k][0] for k in self.latest_keys if k in POS_X | NEG_X)

        # linear y
        if self.latest_keys & POS_Y and self.latest_keys & NEG_Y:
            tgt_y = 0.0
        else:
            tgt_y = sum(KEY_TO_VEL[k][1] for k in self.latest_keys if k in POS_Y | NEG_Y)

        # angular z
        if self.latest_keys & POS_Z and self.latest_keys & NEG_Z:
            tgt_z = 0.0
        else:
            tgt_z = sum(KEY_TO_VEL[k][2] for k in self.latest_keys if k in POS_Z | NEG_Z)

        # --- apply acceleration limits --------------------------
        self.vel_x += clamp_delta(tgt_x - self.vel_x, MAX_DV)
        self.vel_y += clamp_delta(tgt_y - self.vel_y, MAX_DV)
        self.ang_z += clamp_delta(tgt_z - self.ang_z, MAX_DW)

        # --- publish --------------------------------------------
        twist = Twist()
        twist.linear.x  = self.vel_x
        twist.linear.y  = self.vel_y
        twist.angular.z = self.ang_z
        self.pub.publish(twist)


# ---------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description="UDP → /cmd_vel_drives with accel limiting")
    parser.add_argument("--port",  type=int, default=5005, help="UDP port to listen on")
    parser.add_argument("--topic", default="/cmd_vel_drives", help="Twist topic to publish")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = UdpTeleopNode(port=args.port, topic=args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
