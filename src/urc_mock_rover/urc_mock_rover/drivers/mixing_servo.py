"""Mock mixing-servo driver.

Same service surface as the real driver (`mixing_servo_real.py`):
  * `/astrotech/mixing_servo/set_angle` -- drive to an absolute angle in
    degrees (offset from the most recently set home).
  * `/astrotech/mixing_servo/set_home` -- declare the current position to
    be 0 deg. (`std_srvs/Trigger`; no fields.)
  * `/astrotech/mixing_servo/state` -- last commanded angle, deg, 5 Hz.

Mock just keeps the last commanded angle in memory and republishes it.
Useful for panel/wiring development without hardware.
"""

from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from cmr_msgs.srv import SetMixingServoAngle


class MockMixingServoDriver:
    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._current_deg = 0

        self._set_angle_srv = node.create_service(
            SetMixingServoAngle,
            cfg["set_angle_service"],
            self._on_set_angle,
        )
        self._set_home_srv = node.create_service(
            Trigger,
            cfg["set_home_service"],
            self._on_set_home,
        )
        self._state_pub = node.create_publisher(
            Int32, cfg["state_topic"], 10
        )
        rate_hz = float(cfg["state_rate_hz"])
        self._state_timer = node.create_timer(1.0 / rate_hz, self._tick)

        node.get_logger().info(
            f"MockMixingServoDriver up: "
            f"set_angle={cfg['set_angle_service']} "
            f"set_home={cfg['set_home_service']}"
        )

    def _on_set_angle(
        self,
        request: SetMixingServoAngle.Request,
        response: SetMixingServoAngle.Response,
    ) -> SetMixingServoAngle.Response:
        deg = int(request.angle_deg)
        if not 0 <= deg <= 4095:
            response.success = False
            response.message = (
                f"angle_deg out of 12-bit wire range 0..4095: {deg}"
            )
            self._node.get_logger().warn(response.message)
            return response
        self._current_deg = deg
        response.success = True
        response.message = f"mock: angle = {deg} deg"
        self._node.get_logger().info(response.message)
        return response

    def _on_set_home(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request  # std_srvs/Trigger has no fields
        self._current_deg = 0
        response.success = True
        response.message = "mock: set_home(0) -- current is now 0 deg"
        self._node.get_logger().info(response.message)
        return response

    def _tick(self) -> None:
        msg = Int32()
        msg.data = int(self._current_deg)
        self._state_pub.publish(msg)
