"""Real environmental-sensor driver -- Adafruit SCD-30 (NDIR CO2 +
temperature + humidity) over I2C.

Default environment driver in :mod:`astrotech_rover.astrotech_node`.
Drop-in counterpart of :class:`MockEnvPublisher` (same ``__init__(node,
cfg)`` signature, same ``/astrotech/env/sample`` topic +
``cmr_msgs/EnvSample`` type); set ``URC_ENV_MOCK=1`` to swap in the
synthetic-sample driver for hardware-free dev.

Hardware
--------
Adafruit SCD-30 (https://www.adafruit.com/product/4867). I2C device,
fixed address **0x61**. It is *not* a native-USB device, so it reaches
the rover computer one of three ways -- set ``env.real_connection`` in
``astrotech_interfaces.yaml`` to match the wiring:

* ``ft232h``      (default) -- an FT232H USB->I2C bridge. We set
                  ``BLINKA_FT232H=1`` *before* importing ``board`` so
                  Blinka talks to the bridge instead of host GPIO.
* ``jetson_i2c``  -- SCD-30 wired straight to the Jetson's I2C pins;
                  Blinka uses the default ``board.I2C()`` bus.
* ``busio``       -- explicit ``busio.I2C(board.SCL, board.SDA)`` for
                  boards where ``board.I2C()`` isn't defined.

**If the sensor doesn't show up, that connection setting is the first
thing to change.** See gui/operator_guide.md ("Wiring the SCD-30").

Sampling
--------
The SCD-30's measurement interval is 2 s minimum (≈0.5 Hz), set via
``env.real_measurement_interval_s``. We poll ``data_available`` on a fast
ROS timer and publish only when a fresh sample lands, so the publish rate
is sensor-driven, not ``rate_hz`` (which only the mock honors).

Dependencies
------------
``adafruit-circuitpython-scd30`` + ``adafruit-blinka`` (pip). On the
FT232H path you also need ``pyftdi`` and the libusb backend. Install::

    /usr/bin/python3 -m pip install --user \
        adafruit-circuitpython-scd30 adafruit-blinka pyftdi
"""

from __future__ import annotations

import os

from rclpy.node import Node
from std_msgs.msg import Header

from cmr_msgs.msg import EnvSample


# SCD-30 fixed I2C address; included for documentation / sanity logging.
_SCD30_I2C_ADDR = 0x61


class RealEnvDriver:
    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._connection = str(cfg.get("real_connection", "ft232h")).lower()
        self._interval_s = int(cfg.get("real_measurement_interval_s", 2))
        self._frame_id = str(cfg.get("frame_id", "env"))

        self._pub = node.create_publisher(EnvSample, cfg["topic"], 10)

        self._scd = None
        self._init_error: str | None = None
        self._sensor_ok = self._init_sensor()

        if not self._sensor_ok:
            node.get_logger().error(
                f"RealEnvDriver: sensor init failed ({self._init_error}). "
                "Driver will idle and publish nothing -- check the "
                "env.real_connection setting and wiring "
                "(see gui/operator_guide.md)."
            )
            return

        # Poll well above the sensor's update rate; publish only on fresh data.
        self._timer = node.create_timer(0.25, self._tick)
        node.get_logger().info(
            f"RealEnvDriver up: topic={cfg['topic']} "
            f"connection={self._connection} addr=0x{_SCD30_I2C_ADDR:02x} "
            f"interval={self._interval_s}s"
        )

    def _init_sensor(self) -> bool:
        """Open the I2C bus + SCD-30. Returns True on success."""
        try:
            # FT232H must be selected BEFORE `import board`, hence the env
            # var dance rather than a constructor arg.
            if self._connection == "ft232h":
                os.environ["BLINKA_FT232H"] = "1"

            import board  # noqa: E402  (deferred: depends on env var above)
            import adafruit_scd30  # noqa: E402

            if self._connection == "busio":
                import busio  # noqa: E402
                i2c = busio.I2C(board.SCL, board.SDA)
            else:
                # ft232h and jetson_i2c both use the default bus; the env
                # var set above routes ft232h to the bridge.
                i2c = board.I2C()

            scd = adafruit_scd30.SCD30(i2c)
            # measurement_interval is in seconds (2..1800).
            try:
                scd.measurement_interval = self._interval_s
            except Exception as exc:  # noqa: BLE001 - non-fatal tuning step
                self._node.get_logger().warning(
                    f"could not set measurement_interval={self._interval_s}: "
                    f"{exc} (continuing with sensor default)"
                )
            self._scd = scd
            return True
        except ImportError as exc:
            self._init_error = (
                "missing python deps. install with: /usr/bin/python3 -m pip "
                "install --user adafruit-circuitpython-scd30 adafruit-blinka "
                f"pyftdi ({exc})"
            )
            return False
        except Exception as exc:  # noqa: BLE001 - hardware path, report it
            self._init_error = f"{type(exc).__name__}: {exc}"
            return False

    def _tick(self) -> None:
        scd = self._scd
        if scd is None:
            return
        try:
            if not scd.data_available:
                return
            co2 = scd.CO2
            temperature = scd.temperature
            humidity = scd.relative_humidity
        except Exception as exc:  # noqa: BLE001 - transient I2C hiccup
            self._node.get_logger().warning(f"SCD-30 read failed: {exc}")
            return

        # SCD-30 returns None for a field that isn't ready; skip the frame.
        if co2 is None or temperature is None or humidity is None:
            return

        msg = EnvSample()
        msg.header = Header()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.co2_ppm = float(co2)
        msg.humidity_pct = float(humidity)
        msg.temperature_c = float(temperature)
        self._pub.publish(msg)
