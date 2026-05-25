"""Mock auger driver — closed-loop telemetry simulation.

The real auger lives on two moteus controllers (id=15 lead_screw,
id=16 auger) per ``docs/reference/auger_keys_test_harness.py``. moteus
exposes per-cycle position / velocity / torque / temperature / mode /
fault, so the GCS gets honest telemetry — unlike the BDC and servo
boards on Caitlin's library which stay open-loop.

This mock keeps a small kinematic model: integrate commanded velocity
(through a first-order lag) into position, random-walk torque, and
let temperature drift up while motors are active. Watchdog: zero
both velocities if no command for 200 ms.

The real-hardware path lives in :mod:`urc_mock_rover.drivers.auger_real`
and is selected via the ``URC_AUGER_REAL`` env var in
:mod:`urc_mock_rover.mock_rover_node`. Mock remains the default so devs
without an fdcanusb / rover hardware can still launch.

TODO(astrotech-q-fault-codes): the real moteus mode/fault values are
already plumbed through ``RealAugerDriver``; the *mock* still hardcodes
both fields to 0. If we want the mock to exercise GCS fault rendering
we should add a debug knob here later.
"""

from __future__ import annotations

import random
import time

from rclpy.node import Node

from cmr_msgs.msg import AugerCommand, AugerState


_LAG_TIME_CONSTANT_S = 0.05  # ~50 ms first-order lag on velocity tracking.
_DEFAULT_WATCHDOG_MS = 200


class _MotorSim:
    """Per-motor kinematic + thermal state."""

    def __init__(self, rng: random.Random) -> None:
        self._rng = rng
        self.position_rev = 0.0
        self.velocity_rev_s = 0.0   # filtered; what the publisher reports
        self.cmd_velocity_rev_s = 0.0  # raw target from the last command
        self.cmd_max_torque = 0.0
        self.torque_nm = 0.0
        self.temperature_c = 25.0

    def step(self, dt: float) -> None:
        # First-order velocity tracking.
        alpha = min(1.0, dt / _LAG_TIME_CONSTANT_S)
        self.velocity_rev_s += (
            (self.cmd_velocity_rev_s - self.velocity_rev_s) * alpha
        )
        self.position_rev += self.velocity_rev_s * dt

        # Torque random walk: small ±0.3 Nm idle, larger swings under load.
        is_active = abs(self.cmd_velocity_rev_s) > 1e-6
        scale = 0.4 if is_active else 0.05
        cap = max(0.05, self.cmd_max_torque) if is_active else 0.3
        self.torque_nm += self._rng.uniform(-scale, scale) * dt
        self.torque_nm = max(-cap, min(cap, self.torque_nm))

        # Temperature: drift up when active, slowly relax otherwise.
        if is_active:
            self.temperature_c += 0.5 * dt  # ~0.5 °C/s under load (mock).
        else:
            self.temperature_c += (25.0 - self.temperature_c) * 0.02 * dt
        # Soft cap so it doesn't run away during long-held commands.
        self.temperature_c = min(self.temperature_c, 90.0)


class MockAugerDriver:
    """Owns the AugerCommand subscription and AugerState publisher."""

    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        if cfg["cmd_type"] != "cmr_msgs/AugerCommand":
            raise NotImplementedError(
                f"MockAugerDriver expects cmr_msgs/AugerCommand; "
                f"got {cfg['cmd_type']} in astrotech_interfaces.yaml"
            )

        self._watchdog_s = float(
            cfg.get("cmd_watchdog_ms", _DEFAULT_WATCHDOG_MS)
        ) / 1000.0
        self._last_cmd_time = time.monotonic()

        self._rng = random.Random(31337)
        self._lead_screw = _MotorSim(self._rng)
        self._auger = _MotorSim(self._rng)
        self._last_step_time = time.monotonic()

        self._cmd_sub = node.create_subscription(
            AugerCommand, cfg["cmd_topic"], self._on_cmd, 10
        )
        self._state_pub = node.create_publisher(
            AugerState, cfg["state_topic"], 10
        )
        rate_hz = float(cfg["state_rate_hz"])
        self._state_timer = node.create_timer(1.0 / rate_hz, self._tick)

        node.get_logger().info(
            f"MockAugerDriver up: cmd={cfg['cmd_topic']} "
            f"state={cfg['state_topic']} @ {rate_hz} Hz "
            f"(closed-loop sim, watchdog={self._watchdog_s * 1000:.0f} ms)"
        )

    def _on_cmd(self, msg: AugerCommand) -> None:
        self._lead_screw.cmd_velocity_rev_s = float(msg.lead_screw_velocity_rev_s)
        self._lead_screw.cmd_max_torque = float(msg.lead_screw_max_torque_nm)
        self._auger.cmd_velocity_rev_s = float(msg.auger_velocity_rev_s)
        self._auger.cmd_max_torque = float(msg.auger_max_torque_nm)
        self._last_cmd_time = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._last_step_time)
        self._last_step_time = now

        # Watchdog: zero out velocities if no command for >watchdog_s.
        if now - self._last_cmd_time > self._watchdog_s:
            self._lead_screw.cmd_velocity_rev_s = 0.0
            self._auger.cmd_velocity_rev_s = 0.0

        self._lead_screw.step(dt)
        self._auger.step(dt)

        out = AugerState()
        out.stamp = self._node.get_clock().now().to_msg()

        out.lead_screw_position_rev = float(self._lead_screw.position_rev)
        out.lead_screw_velocity_rev_s = float(self._lead_screw.velocity_rev_s)
        out.lead_screw_torque_nm = float(self._lead_screw.torque_nm)
        out.lead_screw_temperature_c = float(self._lead_screw.temperature_c)
        out.lead_screw_mode = 0
        out.lead_screw_fault = 0

        out.auger_position_rev = float(self._auger.position_rev)
        out.auger_velocity_rev_s = float(self._auger.velocity_rev_s)
        out.auger_torque_nm = float(self._auger.torque_nm)
        out.auger_temperature_c = float(self._auger.temperature_c)
        out.auger_mode = 0
        out.auger_fault = 0

        self._state_pub.publish(out)
