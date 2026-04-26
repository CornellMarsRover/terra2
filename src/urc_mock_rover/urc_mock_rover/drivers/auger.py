"""Mock auger driver.

Honest, commanded-only telemetry: the message echoes the *most recent
command we sent* and when. There is no synthetic encoder integration —
the real Astrotech CAN-FD library exposes no feedback, so any "current
position" or "is_moving" field would be made up.

TODO(astrotech-q-1): command message type is a single ``geometry_msgs/Twist``
(linear.z = up/down vel, angular.z = spin vel). If the real driver
splits these into two topics or changes units, edit this file and the
YAML together.
"""

from __future__ import annotations

from geometry_msgs.msg import Twist
from rclpy.node import Node

from cmr_msgs.msg import AugerState


_CMD_KIND_NONE = AugerState.COMMAND_KIND_NONE
_CMD_KIND_UP = AugerState.COMMAND_KIND_UP
_CMD_KIND_DOWN = AugerState.COMMAND_KIND_DOWN
_CMD_KIND_SPIN_FWD = AugerState.COMMAND_KIND_SPIN_FORWARD
_CMD_KIND_SPIN_BACK = AugerState.COMMAND_KIND_SPIN_BACKWARD


def _classify_twist(msg: Twist) -> int:
    """Reduce a 6-DoF Twist to one of the COMMAND_KIND_* enum values.

    Spin (angular.z) takes priority over linear if both are nonzero;
    panels send one or the other in practice.
    """
    if abs(msg.angular.z) > 1e-6:
        return _CMD_KIND_SPIN_FWD if msg.angular.z > 0 else _CMD_KIND_SPIN_BACK
    if abs(msg.linear.z) > 1e-6:
        return _CMD_KIND_UP if msg.linear.z > 0 else _CMD_KIND_DOWN
    return _CMD_KIND_NONE


class MockAugerDriver:
    """Owns the auger command subscription + state publisher."""

    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node

        # TODO(astrotech-q-1): if cmd_type in YAML changes, change the import
        # and the msg class used here. The rest of this file stays.
        if cfg["cmd_type"] != "geometry_msgs/Twist":
            raise NotImplementedError(
                f"MockAugerDriver only implements geometry_msgs/Twist; "
                f"got {cfg['cmd_type']} in astrotech_interfaces.yaml"
            )

        self._last_kind = _CMD_KIND_NONE
        # commanded_duration is unknown in this stub (the panel sends a Twist,
        # not a "run for N seconds" command); leave 0 until a real driver
        # adopts the BDC-board duration semantics.
        self._last_duration = 0.0
        self._last_stamp = node.get_clock().now().to_msg()

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
            f"state={cfg['state_topic']} @ {rate_hz} Hz (commanded-only)"
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_kind = _classify_twist(msg)
        self._last_stamp = self._node.get_clock().now().to_msg()

    def _tick(self) -> None:
        out = AugerState()
        out.last_command_kind = self._last_kind
        out.commanded_duration = float(self._last_duration)
        out.commanded_at = self._last_stamp
        self._state_pub.publish(out)
