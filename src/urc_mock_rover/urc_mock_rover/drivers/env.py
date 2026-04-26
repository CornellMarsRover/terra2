"""Mock CO2/humidity/temperature publisher.

CO2 gets a slow sinusoidal drift plus a periodic transient spike so the GCS
plot has visible events. Humidity and temperature drift slowly inside their
ranges.

TODO(astrotech-q-6): swap to external driver type if one exists.
"""

from __future__ import annotations

import math
import time

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header

from cmr_msgs.msg import EnvSample


_CO2_BASELINE_PPM = 420.0
_CO2_DRIFT_AMPLITUDE_PPM = 20.0
_CO2_DRIFT_PERIOD_S = 120.0

_CO2_TRANSIENT_PEAK_PPM = 1200.0
_CO2_TRANSIENT_PERIOD_S = 30.0
_CO2_TRANSIENT_RAMP_S = 3.0
_CO2_TRANSIENT_HOLD_S = 5.0
_CO2_TRANSIENT_DECAY_S = 10.0


def _co2_transient(phase_s: float) -> float:
    """Value of the CO2 spike at position ``phase_s`` within the event."""
    ramp = _CO2_TRANSIENT_RAMP_S
    hold = ramp + _CO2_TRANSIENT_HOLD_S
    decay = hold + _CO2_TRANSIENT_DECAY_S
    peak_delta = _CO2_TRANSIENT_PEAK_PPM - _CO2_BASELINE_PPM
    if phase_s < 0.0:
        return 0.0
    if phase_s < ramp:
        return peak_delta * (phase_s / ramp)
    if phase_s < hold:
        return peak_delta
    if phase_s < decay:
        fall = (phase_s - hold) / _CO2_TRANSIENT_DECAY_S
        return peak_delta * (1.0 - fall)
    return 0.0


class MockEnvPublisher:
    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._t0 = time.monotonic()
        self._rng = np.random.default_rng(seed=7)

        self._pub = node.create_publisher(EnvSample, cfg["topic"], 10)
        rate_hz = float(cfg["rate_hz"])
        self._timer = node.create_timer(1.0 / rate_hz, self._tick)

        node.get_logger().info(
            f"MockEnvPublisher up: topic={cfg['topic']} @ {rate_hz} Hz"
        )

    def _tick(self) -> None:
        t = time.monotonic() - self._t0

        co2 = (
            _CO2_BASELINE_PPM
            + _CO2_DRIFT_AMPLITUDE_PPM
            * math.sin(2.0 * math.pi * t / _CO2_DRIFT_PERIOD_S)
            + float(self._rng.normal(0.0, 2.0))
        )
        transient_phase = t % _CO2_TRANSIENT_PERIOD_S
        co2 += _co2_transient(transient_phase)

        humidity = (
            50.0
            + 10.0 * math.sin(2.0 * math.pi * t / 90.0)
            + float(self._rng.normal(0.0, 0.3))
        )
        temperature = (
            22.5
            + 2.5 * math.sin(2.0 * math.pi * t / 150.0 + 1.3)
            + float(self._rng.normal(0.0, 0.05))
        )

        msg = EnvSample()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "env"
        msg.co2_ppm = float(co2)
        msg.humidity_pct = float(humidity)
        msg.temperature_c = float(temperature)
        self._pub.publish(msg)
