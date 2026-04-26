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
        self._auger = MockAugerDriver(self, cfg["auger"])
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
