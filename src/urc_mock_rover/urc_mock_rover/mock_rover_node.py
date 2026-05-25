"""Coordinator node for the URC Astrotech mock rover.

Reads ``config/astrotech_interfaces.yaml`` from the installed share dir and
instantiates one driver per feature area. The node itself owns no business
logic — every driver is free to publish/subscribe/advertise independently
under this node's clock and logger.

Running::

    ros2 launch urc_mock_rover mock.launch.py

starts this node alongside the foxglove_bridge.
"""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from urc_mock_rover.drivers.analysis_sequencer import MockAnalysisSequencer
from urc_mock_rover.drivers.auger import MockAugerDriver
from urc_mock_rover.drivers.env import MockEnvPublisher
from urc_mock_rover.drivers.mixing_servo import MockMixingServoDriver
from urc_mock_rover.drivers.raman import MockRamanPublisher

# Camera feeds intentionally absent: that work lives on a separate
# branch and will merge into astrotech-gui later.


_PACKAGE = "urc_mock_rover"
_DEFAULT_CONFIG_NAME = "astrotech_interfaces.yaml"

# Set URC_AUGER_REAL=1 (or true / yes) to swap the mock auger driver for
# RealAugerDriver, which talks to the actual moteus stack on the fdcanusb.
# Same pattern for URC_MIXING_SERVO_REAL=1, which swaps in a driver that
# speaks the CMR servo wire format on the fdcanusb directly.
#
# Mock remains the default so devs without hardware can still launch the
# rover. The real drivers are imported lazily inside the switch so a
# missing moteus / pyserial-asyncio install does not break mock launches.
#
# Bus contention warning: the two real drivers cannot coexist on the
# same fdcanusb today -- auger_real holds the moteus singleton transport
# while mixing_servo_real opens the same /dev/ttyACM* directly via
# pyserial-asyncio. Run only one at a time until the URC team's unified
# driver-node lands (post-URC backlog). The user's plan is to move the
# servo bus onto a separate fdcanusb, which makes the contention moot.
_REAL_AUGER_ENV_VAR = "URC_AUGER_REAL"
_REAL_MIXING_SERVO_ENV_VAR = "URC_MIXING_SERVO_REAL"
_TRUTHY = {"1", "true", "yes", "on"}


def _locate_config() -> Path:
    """Resolve the interface YAML from the installed share dir or overlay."""
    override = os.environ.get("URC_MOCK_ROVER_CONFIG")
    if override:
        return Path(override)
    share_dir = Path(get_package_share_directory(_PACKAGE))
    return share_dir / "config" / _DEFAULT_CONFIG_NAME


class MockRoverNode(Node):
    """Thin container for all mock drivers."""

    def __init__(self) -> None:
        super().__init__("mock_rover_node")

        config_path = _locate_config()
        if not config_path.is_file():
            raise FileNotFoundError(
                f"astrotech_interfaces.yaml not found at {config_path}. "
                "Did you colcon build --symlink-install and source install/setup.bash?"
            )
        self.get_logger().info(f"loading config {config_path}")
        with config_path.open("r") as fh:
            cfg = yaml.safe_load(fh)

        # Order matters only for logger readability; no inter-driver deps.
        if os.environ.get(_REAL_AUGER_ENV_VAR, "").strip().lower() in _TRUTHY:
            from urc_mock_rover.drivers.auger_real import RealAugerDriver
            self.get_logger().info(
                f"{_REAL_AUGER_ENV_VAR} is set; using RealAugerDriver "
                "(talks to the moteus stack on fdcanusb)."
            )
            self._auger = RealAugerDriver(self, cfg["auger"])
        else:
            self._auger = MockAugerDriver(self, cfg["auger"])
        if os.environ.get(_REAL_MIXING_SERVO_ENV_VAR, "").strip().lower() in _TRUTHY:
            from urc_mock_rover.drivers.mixing_servo_real import (
                RealMixingServoDriver,
            )
            self.get_logger().info(
                f"{_REAL_MIXING_SERVO_ENV_VAR} is set; using "
                "RealMixingServoDriver (talks to the CMR servo board "
                "on fdcanusb directly via pyserial-asyncio)."
            )
            self._mixing = RealMixingServoDriver(self, cfg["mixing_servo"])
        else:
            self._mixing = MockMixingServoDriver(self, cfg["mixing_servo"])
        self._analysis = MockAnalysisSequencer(self, cfg["analysis"])
        self._raman = MockRamanPublisher(self, cfg["raman"])
        self._env = MockEnvPublisher(self, cfg["env"])

        self.get_logger().info("mock rover ready")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockRoverNode()
    # MultiThreadedExecutor so the analysis action server's blocking
    # execute_callback doesn't stall the other drivers' timers.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
