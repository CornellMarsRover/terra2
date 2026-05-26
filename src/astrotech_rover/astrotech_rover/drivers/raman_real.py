"""Real Raman driver -- reads a TCD1340-class linear-CCD spectrometer over
USB serial and republishes it as ``cmr_msgs/RamanSpectrum``.

Default Raman driver in :mod:`astrotech_rover.astrotech_node`. Drop-in
counterpart of :class:`MockRamanPublisher` (same ``__init__(node, cfg)``
signature, same ``/astrotech/raman/spectrum`` topic + type); set
``URC_RAMAN_MOCK=1`` to swap in the synthetic-spectrum driver.

Hardware / data path
--------------------
A bare TCD1340 CCD cannot talk USB on its own -- a microcontroller
(Arduino / Teensy / STM32) clocks the sensor, runs the ADC, and streams
each scan over a USB serial port. **The exact framing is not yet
confirmed with Astrotech**, so this driver implements one clean,
documented contract and flags it loudly. When the firmware is known,
adjust :meth:`_parse_line` (and the ``raman.real_frame`` config) to match.

ASSUMED WIRE PROTOCOL (``raman.real_frame: "csv_line"``)
    One spectrum per newline-terminated line: ``n_points`` intensity
    values separated by commas or whitespace, e.g.::

        103,98,110, ... (3648 values) ... ,95\n

    Lines that don't parse to ``n_points`` numbers (boot banners, partial
    reads) are skipped. ASCII keeps the firmware trivial; if bandwidth
    matters, switch to a binary frame and extend ``_parse_line``.

Calibration (optional)
---------------------
The CCD reports intensity per *pixel*; it knows nothing about
wavenumbers. To put the x-axis in real cm⁻¹ you need a pixel->wavenumber
calibration, supplied in config under ``raman.real_calibration``::

    real_calibration:
      laser_nm: 785.0
      pixel_to_wavelength_poly: [c0, c1, c2]   # nm = c0 + c1*p + c2*p^2

We then compute, per pixel p:
    wavelength_nm = c0 + c1*p + c2*p^2 (+ ...)
    shift_cm_inv  = (1/laser_nm - 1/wavelength_nm) * 1e7

If ``real_calibration`` is absent, the x-axis is the **pixel index**
(0..n_points-1) -- the plot still works, peaks just aren't in cm⁻¹ until
Astrotech provides the polynomial + laser wavelength. See
gui/operator_guide.md ("Raman calibration").

Dependency: ``pyserial`` (``/usr/bin/python3 -m pip install --user pyserial``).
"""

from __future__ import annotations

import re
import threading

from rclpy.node import Node
from std_msgs.msg import Header

from cmr_msgs.msg import RamanSpectrum

try:
    import serial  # pyserial
except ImportError as exc:  # pragma: no cover - hardware path
    raise ImportError(
        "RealRamanDriver: missing pyserial. install with: "
        "/usr/bin/python3 -m pip install --user pyserial "
        f"({exc})"
    ) from exc


# Split a frame line on commas and/or any whitespace.
_TOKEN_SPLIT = re.compile(r"[,\s]+")


class RealRamanDriver:
    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._n_points = int(cfg["n_points"])
        self._port = str(cfg.get("real_port", "/dev/ttyACM1"))
        self._baud = int(cfg.get("real_baud", 115200))
        self._frame_id = str(cfg.get("frame_id", "raman"))
        # Accept a frame whose length is within this many samples of
        # n_points (CCDs often emit a few dark/dummy pixels).
        self._len_tol = int(cfg.get("real_length_tolerance", 8))

        self._wavenumbers = self._build_wavenumbers(cfg.get("real_calibration"))

        self._pub = node.create_publisher(RamanSpectrum, cfg["topic"], 10)

        self._ser: serial.Serial | None = None
        self._stop = threading.Event()
        node.context.on_shutdown(self._stop.set)

        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=1.0)
        except Exception as exc:  # noqa: BLE001 - hardware path
            node.get_logger().error(
                f"RealRamanDriver: could not open {self._port} @ {self._baud} "
                f"({exc}). Driver will idle and publish nothing -- check "
                "raman.real_port / wiring (see gui/operator_guide.md)."
            )
            return

        self._thread = threading.Thread(
            target=self._read_loop, name="raman_real_read", daemon=True
        )
        self._thread.start()

        calib = "cm⁻¹ (calibrated)" if cfg.get("real_calibration") else "pixel index"
        node.get_logger().info(
            f"RealRamanDriver up: topic={cfg['topic']} port={self._port} "
            f"baud={self._baud} n_points={self._n_points} x-axis={calib}"
        )

    def _build_wavenumbers(self, calib: dict | None) -> list[float]:
        """Precompute the fixed x-axis: cm⁻¹ if calibrated, else pixel index."""
        n = self._n_points
        if not calib:
            return [float(p) for p in range(n)]
        try:
            laser_nm = float(calib["laser_nm"])
            poly = [float(c) for c in calib["pixel_to_wavelength_poly"]]
        except (KeyError, TypeError, ValueError) as exc:
            self._node.get_logger().error(
                f"raman.real_calibration malformed ({exc}); falling back to "
                "pixel-index x-axis. Expected laser_nm + pixel_to_wavelength_poly."
            )
            return [float(p) for p in range(n)]

        wn: list[float] = []
        for p in range(n):
            # wavelength_nm = c0 + c1*p + c2*p^2 + ...  (ascending coeffs)
            wl = 0.0
            for power, c in enumerate(poly):
                wl += c * (p ** power)
            if wl <= 0.0:
                wn.append(0.0)
                continue
            shift = (1.0 / laser_nm - 1.0 / wl) * 1.0e7
            wn.append(shift)
        return wn

    def _read_loop(self) -> None:
        assert self._ser is not None
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except Exception as exc:  # noqa: BLE001 - transient serial error
                self._node.get_logger().warning(f"Raman serial read failed: {exc}")
                continue
            if not raw:
                continue  # timeout, no full line yet
            intensities = self._parse_line(raw)
            if intensities is None:
                continue
            self._publish(intensities)

        try:
            self._ser.close()
        except Exception:  # noqa: BLE001
            pass

    def _parse_line(self, raw: bytes) -> list[float] | None:
        """Parse one frame line into intensities, or None if it's not a frame.

        Adjust this method when the real firmware framing is confirmed.
        """
        line = raw.decode("ascii", errors="ignore").strip()
        if not line:
            return None
        tokens = [t for t in _TOKEN_SPLIT.split(line) if t]
        if abs(len(tokens) - self._n_points) > self._len_tol:
            return None  # banner / partial line / wrong frame
        try:
            return [float(t) for t in tokens]
        except ValueError:
            return None  # contained non-numeric tokens

    def _publish(self, intensities: list[float]) -> None:
        # Align lengths: trust the x-axis we precomputed for n_points.
        n = min(len(intensities), len(self._wavenumbers))
        msg = RamanSpectrum()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.wavenumbers_cm_inv = [float(x) for x in self._wavenumbers[:n]]
        msg.intensities = [float(x) for x in intensities[:n]]
        self._pub.publish(msg)
