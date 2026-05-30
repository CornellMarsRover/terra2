"""Coordinator node for the URC Astrotech rover.

Reads ``config/astrotech_interfaces.yaml`` from the installed share dir and
instantiates one driver per feature area. The node itself owns no business
logic — every driver is free to publish/subscribe/advertise independently
under this node's clock and logger.

Running::

    ros2 launch astrotech_rover astrotech.launch.py

starts this node alongside the foxglove_bridge.

Real-by-default
---------------
Each feature runs its **real hardware driver by default**. For GUI
development / demos / CI without hardware, swap any feature to its
simulation driver with a per-feature env var:

    URC_AUGER_MOCK=1          URC_MIXING_SERVO_MOCK=1
    URC_RAMAN_MOCK=1          URC_ENV_MOCK=1          URC_ANALYSIS_MOCK=1

(The older ``URC_*_REAL`` vars are not read by anything — real is already
the default, so an old command that sets one still gets the real driver.)

Real drivers are imported lazily inside the switch so that a missing
moteus / pyserial / adafruit-blinka install only affects the feature
that needs it. Every real driver also degrades gracefully on its own:
if its hardware/deps are absent it logs an error and idles (publishes
nothing) rather than crashing the node.

The analysis sequencer now has a real driver (drivers/analysis_real.py,
CMR_CANFD pumps/servos); URC_ANALYSIS_MOCK=1 swaps in the no-hardware sim.
It shares the fdcanusb with the auger / mixing servo, so run it in lab mode
(URC_AUGER_MOCK=1 URC_MIXING_SERVO_MOCK=1).

fdcanusb bus contention
-----------------------
The auger (moteus singleton transport) and the mixing servo
(pyserial-asyncio direct open) cannot share one fdcanusb today — the
second open errors with EBUSY. On a single dongle, set
``URC_MIXING_SERVO_MOCK=1`` (or ``URC_AUGER_MOCK=1``) so only one holds
the bus, or use a second fdcanusb. Raman (USB serial) and env (I2C) are
on other buses and are unaffected.
"""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# Real drivers are the default; imported lazily in __init__ so a missing
# hardware dep only bites the feature that uses it. Mock drivers live in
# the drivers/mock/ subpackage (see its README) and are opt-in.

# Camera feeds are launched rover-side via src/cmr_cams/ (cmr_cv camera
# nodes + stereolabs zed_wrapper); see gui/operator_guide.md.


_PACKAGE = "astrotech_rover"
_DEFAULT_CONFIG_NAME = "astrotech_interfaces.yaml"

# Per-feature MOCK opt-in. Default (no env var) is the real hardware driver.
_MOCK_AUGER_ENV_VAR = "URC_AUGER_MOCK"
_MOCK_MIXING_SERVO_ENV_VAR = "URC_MIXING_SERVO_MOCK"
_MOCK_RAMAN_ENV_VAR = "URC_RAMAN_MOCK"
_MOCK_ENV_ENV_VAR = "URC_ENV_MOCK"
_MOCK_ANALYSIS_ENV_VAR = "URC_ANALYSIS_MOCK"
_TRUTHY = {"1", "true", "yes", "on"}


def _env_truthy(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in _TRUTHY


def _cfg_enabled(cfg: dict, key: str) -> bool:
    value = cfg.get(key, {}).get("enabled", True)
    if isinstance(value, str):
        return value.strip().lower() not in {"0", "false", "no", "off"}
    return bool(value)


def _locate_config() -> Path:
    """Resolve the interface YAML from the installed share dir or overlay."""
    override = os.environ.get("ASTROTECH_CONFIG")
    if override:
        return Path(override)
    share_dir = Path(get_package_share_directory(_PACKAGE))
    return share_dir / "config" / _DEFAULT_CONFIG_NAME


class AstrotechRoverNode(Node):
    """Thin container for all Astrotech feature drivers (real by default)."""

    def __init__(self) -> None:
        super().__init__("astrotech_node")

        config_path = _locate_config()
        if not config_path.is_file():
            raise FileNotFoundError(
                f"astrotech_interfaces.yaml not found at {config_path}. "
                "Did you colcon build --symlink-install and source install/setup.bash?"
            )
        self.get_logger().info(f"loading config {config_path}")
        with config_path.open("r") as fh:
            cfg = yaml.safe_load(fh)

        # Each feature starts independently. If a real driver's hardware or
        # python library is missing, _start logs the error and leaves that
        # one feature unavailable -- it must NOT take down the others (the
        # real drivers import their hardware libs at module load, so a bare
        # default launch on a box missing e.g. moteus or pyserial would
        # otherwise crash the whole node).
        self._auger = self._start("auger", lambda: self._build_auger(cfg))
        self._mixing = self._start("mixing_servo", lambda: self._build_mixing(cfg))
        self._analysis = self._start("analysis", lambda: self._build_analysis(cfg))
        self._raman = self._start("raman", lambda: self._build_raman(cfg))
        self._env = self._start("env", lambda: self._build_env(cfg))
        # Always-on utility: save-plot-to-disk services for the GUI.
        self._snapshot = self._start("snapshot", lambda: self._build_snapshot(cfg))

        self.get_logger().info("astrotech rover ready")

    def _start(self, label, factory):
        """Build one feature driver in isolation. Catch + log any startup
        failure (missing library, no hardware, bad config) so one feature
        can't crash the rest of the node. Returns the driver or None."""
        try:
            return factory()
        except Exception as exc:  # noqa: BLE001 - isolate per-feature startup
            self.get_logger().error(
                f"{label}: driver failed to start -- {exc}. This feature is "
                "unavailable; the rest of the node continues."
            )
            return None

    def _build_auger(self, cfg):
        if _env_truthy(_MOCK_AUGER_ENV_VAR):
            from astrotech_rover.drivers.mock.auger import MockAugerDriver
            self.get_logger().info(f"{_MOCK_AUGER_ENV_VAR} set; auger = sim.")
            return MockAugerDriver(self, cfg["auger"])
        from astrotech_rover.drivers.auger_real import RealAugerDriver
        return RealAugerDriver(self, cfg["auger"])

    def _build_mixing(self, cfg):
        if not _cfg_enabled(cfg, "mixing_servo"):
            self.get_logger().info("mixing_servo disabled in config.")
            return None
        if _env_truthy(_MOCK_MIXING_SERVO_ENV_VAR):
            from astrotech_rover.drivers.mock.mixing_servo import MockMixingServoDriver
            self.get_logger().info(f"{_MOCK_MIXING_SERVO_ENV_VAR} set; mixing servo = sim.")
            return MockMixingServoDriver(self, cfg["mixing_servo"])
        from astrotech_rover.drivers.mixing_servo_real import RealMixingServoDriver
        return RealMixingServoDriver(self, cfg["mixing_servo"])

    def _build_analysis(self, cfg):
        if _env_truthy(_MOCK_ANALYSIS_ENV_VAR):
            from astrotech_rover.drivers.mock.analysis import build_mock_analysis
            self.get_logger().info(f"{_MOCK_ANALYSIS_ENV_VAR} set; analysis = sim.")
            return build_mock_analysis(self, cfg["analysis"])
        from astrotech_rover.drivers.analysis_real import build_real_analysis
        return build_real_analysis(self, cfg["analysis"])

    def _build_raman(self, cfg):
        if not _cfg_enabled(cfg, "raman"):
            self.get_logger().info("raman disabled in config.")
            return None
        if _env_truthy(_MOCK_RAMAN_ENV_VAR):
            from astrotech_rover.drivers.mock.raman import MockRamanPublisher
            self.get_logger().info(f"{_MOCK_RAMAN_ENV_VAR} set; raman = sim.")
            return MockRamanPublisher(self, cfg["raman"])
        from astrotech_rover.drivers.raman_real import RealRamanDriver
        return RealRamanDriver(self, cfg["raman"])

    def _build_env(self, cfg):
        if _env_truthy(_MOCK_ENV_ENV_VAR):
            from astrotech_rover.drivers.mock.env import MockEnvPublisher
            self.get_logger().info(f"{_MOCK_ENV_ENV_VAR} set; env = sim.")
            return MockEnvPublisher(self, cfg["env"])
        from astrotech_rover.drivers.env_real import RealEnvDriver
        return RealEnvDriver(self, cfg["env"])

    def _build_snapshot(self, cfg):
        from astrotech_rover.drivers.snapshot import SnapshotDriver
        return SnapshotDriver(self, cfg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstrotechRoverNode()
    # MultiThreadedExecutor so a blocking service callback (e.g. the analysis
    # cancel dispatching a stop to the bus) doesn't stall the other drivers'
    # timers.
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
