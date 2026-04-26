"""Mock mixing-servo driver.

Advertises the preset-set service, keeps last preset as state, and
re-publishes it on a low-rate timer so the GCS always sees a current value.

TODO(astrotech-q-2): controller family is not yet known. Mock holds state
in a Python string and does nothing else; swap this class when the real
driver is chosen.
"""

from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import String

from cmr_msgs.srv import SetMixingServoPreset


class MockMixingServoDriver:
    """Service server + state publisher for the mixing servo."""

    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._presets = list(cfg["presets"])
        self._current = "RETRACT" if "RETRACT" in self._presets else self._presets[0]

        self._srv = node.create_service(
            SetMixingServoPreset,
            cfg["set_preset_service"],
            self._on_set_preset,
        )
        self._state_pub = node.create_publisher(
            String, cfg["state_topic"], 10
        )
        rate_hz = float(cfg["state_rate_hz"])
        self._state_timer = node.create_timer(1.0 / rate_hz, self._tick)

        node.get_logger().info(
            f"MockMixingServoDriver up: service={cfg['set_preset_service']} "
            f"presets={self._presets}"
        )

    def _on_set_preset(
        self,
        request: SetMixingServoPreset.Request,
        response: SetMixingServoPreset.Response,
    ) -> SetMixingServoPreset.Response:
        name = request.preset_name
        if name not in self._presets:
            response.success = False
            response.message = (
                f"unknown preset {name!r}; valid: {self._presets}"
            )
            self._node.get_logger().warn(response.message)
            return response

        self._current = name
        response.success = True
        response.message = f"set to {name}"
        self._node.get_logger().info(response.message)
        return response

    def _tick(self) -> None:
        msg = String()
        msg.data = self._current
        self._state_pub.publish(msg)
