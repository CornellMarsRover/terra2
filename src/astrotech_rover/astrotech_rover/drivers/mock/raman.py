"""Mock Raman publisher.

Synthesizes a believable-looking spectrum with three Gaussian peaks whose
amplitudes drift slowly over time, plus additive noise.

TODO: if a real driver already exists in another repo with
a different message type, swap the import + publisher type here and update
the YAML.
"""

from __future__ import annotations

import math
import time

import numpy as np
from rclpy.node import Node
from std_msgs.msg import Header

from cmr_msgs.msg import RamanSpectrum


class MockRamanPublisher:
    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._t0 = time.monotonic()

        self._wavenumbers = np.linspace(
            float(cfg["wavenumber_min"]),
            float(cfg["wavenumber_max"]),
            int(cfg["n_points"]),
            dtype=np.float32,
        )
        self._peaks = [float(x) for x in cfg["mock_peaks_cm_inv"]]
        self._rng = np.random.default_rng(seed=42)

        self._pub = node.create_publisher(RamanSpectrum, cfg["topic"], 10)
        rate_hz = float(cfg["rate_hz"])
        self._timer = node.create_timer(1.0 / rate_hz, self._tick)

        node.get_logger().info(
            f"MockRamanPublisher up: topic={cfg['topic']} "
            f"n={len(self._wavenumbers)} @ {rate_hz} Hz"
        )

    def _tick(self) -> None:
        t = time.monotonic() - self._t0
        intensities = np.zeros_like(self._wavenumbers)

        # Three Gaussian peaks, widths fixed, amplitudes oscillating at
        # different periods so the GCS plot looks alive.
        widths_cm_inv = [12.0, 8.0, 15.0]
        base_amplitudes = [1.0, 0.8, 0.6]
        periods_s = [23.0, 37.0, 53.0]
        for peak_cm_inv, width, base_amp, period in zip(
            self._peaks, widths_cm_inv, base_amplitudes, periods_s
        ):
            drift = 0.7 + 0.3 * math.sin(2.0 * math.pi * t / period)
            amp = base_amp * drift
            intensities += amp * np.exp(
                -((self._wavenumbers - peak_cm_inv) ** 2) / (2.0 * width * width)
            ).astype(np.float32)

        # Baseline + noise.
        baseline = 0.05 + 0.02 * np.sin(self._wavenumbers / 400.0).astype(np.float32)
        noise = self._rng.normal(
            loc=0.0, scale=0.01, size=self._wavenumbers.shape
        ).astype(np.float32)
        intensities = intensities + baseline + noise

        msg = RamanSpectrum()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "raman"
        msg.wavenumbers_cm_inv = self._wavenumbers.tolist()
        msg.intensities = intensities.tolist()
        self._pub.publish(msg)
