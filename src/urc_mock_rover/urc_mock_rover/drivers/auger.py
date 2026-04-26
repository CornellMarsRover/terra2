"""Mock auger driver.

Integrates commanded velocities into a simulated position/velocity state.
Interface is driven by the ``auger:`` block of
``astrotech_interfaces.yaml``; nothing here hardcodes topic names.

TODO(astrotech-q-1): command message type is a single ``geometry_msgs/Twist``
(linear.z = up/down vel, angular.z = spin vel). If the real moteus driver
splits these into two topics or changes units, edit this file and the YAML
together.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Header

from cmr_msgs.msg import AugerState


@dataclass
class _AugerSim:
    """Minimal kinematic state carried across timer ticks."""

    position_rev: float = 0.0
    spin_position_rev: float = 0.0
    linear_cmd: float = 0.0
    spin_cmd: float = 0.0
    last_cmd_time: float = 0.0


class MockAugerDriver:
    """Owns the auger command subscription + state publisher.

    Parameters
    ----------
    node : rclpy.node.Node
        Parent coordinator node. This driver creates timers/subs/pubs on it
        rather than owning its own rclpy node, so lifecycle is shared.
    cfg : dict
        The ``auger`` slice of ``astrotech_interfaces.yaml``.
    """

    _CMD_TIMEOUT_S = 0.5  # if no new cmd within this window, assume stop.

    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._cfg = cfg
        self._sim = _AugerSim(last_cmd_time=time.monotonic())

        # TODO(astrotech-q-1): if cmd_type in YAML changes, change the import
        # and the msg class used here. The rest of this file stays.
        if cfg["cmd_type"] != "geometry_msgs/Twist":
            raise NotImplementedError(
                f"MockAugerDriver only implements geometry_msgs/Twist; "
                f"got {cfg['cmd_type']} in astrotech_interfaces.yaml"
            )

        self._cmd_sub = node.create_subscription(
            Twist, cfg["cmd_topic"], self._on_cmd, 10
        )
        self._state_pub = node.create_publisher(
            AugerState, cfg["state_topic"], 10
        )

        rate_hz = float(cfg["state_rate_hz"])
        self._state_timer = node.create_timer(1.0 / rate_hz, self._tick)

        node.get_logger().info(
            f"MockAugerDriver up: cmd={cfg['cmd_topic']} "
            f"state={cfg['state_topic']} @ {rate_hz} Hz"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._sim.linear_cmd = float(msg.linear.z)
        self._sim.spin_cmd = float(msg.angular.z)
        self._sim.last_cmd_time = time.monotonic()

    def _tick(self) -> None:
        # Drop to zero velocity after a quiet period so the state reflects a
        # stopped auger (matches how a real driver with a watchdog behaves).
        now = time.monotonic()
        if now - self._sim.last_cmd_time > self._CMD_TIMEOUT_S:
            self._sim.linear_cmd = 0.0
            self._sim.spin_cmd = 0.0

        dt = 1.0 / float(self._cfg["state_rate_hz"])
        self._sim.position_rev += self._sim.linear_cmd * dt
        self._sim.spin_position_rev += self._sim.spin_cmd * dt

        msg = AugerState()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "auger"
        msg.position_rev = float(self._sim.position_rev)
        msg.velocity_rev_s = float(self._sim.linear_cmd)
        # Torque is not simulated — report zero until a real driver lands.
        msg.torque_nm = 0.0
        msg.is_moving = (
            abs(self._sim.linear_cmd) > 1e-6 or abs(self._sim.spin_cmd) > 1e-6
        )
        self._state_pub.publish(msg)
