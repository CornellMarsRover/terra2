"""Real mixing-servo driver -- bridges set_angle / set_home services to
the CMR servo board over the fdcanusb.

Drop-in replacement for `MockMixingServoDriver` (same `__init__(node, cfg)`
signature, same advertised topics/services). Selected via the
`URC_MIXING_SERVO_REAL` env var in :mod:`urc_mock_rover.mock_rover_node`;
mock remains the default so devs without hardware can still launch.

Code path
---------
Mirrors `scripts/mixing_servo_min.py` byte-for-byte:

* Open the fdcanusb via `CMR_CANFD.FdCanInterface` (pyserial-asyncio).
* Manual configure_bus at 1 Mbit/s -- the vendored library's
  `configure_bus()` hardcodes 500 kbps, so we issue the six bring-up
  commands ourselves with the right bitrate.
* Every goto frame is built with
  `servo._make_control_frame(control_mode=0, control_data=N, clear_faults=1)`
  and written via `fd.write_frame(flags="FB")`. `clear_faults=1` is
  load-bearing on this firmware; without it the bus ACKs frames but the
  chamber stops moving once any other traffic has touched the bus.
* Set-home uses the library's `servo.set_home(0)` (control_mode=3,
  reset_home=1).

See `docs/mixing_servo_bringup_handoff.md` for the bench history.

Bus contention
--------------
This driver opens `/dev/ttyACM*` directly via pyserial-asyncio. It does
NOT share the moteus singleton transport that `auger_real.py` uses, so
the two cannot coexist on the same fdcanusb -- the second open errors
with EBUSY. Until the URC team's unified driver-node lands (post-URC
backlog), only one of `URC_AUGER_REAL` / `URC_MIXING_SERVO_REAL` should
be set per process. The user's plan is to physically separate the two
buses; once that lands the contention disappears.
"""

from __future__ import annotations

import asyncio
import os
import sys
import threading

from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

from cmr_msgs.srv import SetMixingServoAngle

# Vendored CMR_CANFD library lives in <repo>/third_party/astrotech_canfd/
# (NOT pip-installed). Resolving the path needs `realpath` on __file__
# because `colcon build --symlink-install` makes the installed driver a
# symlink to the source -- realpath gives us the actual on-disk source
# path, from which we can walk up to the repo root.
#
# If the install was done WITHOUT --symlink-install this resolution
# fails, since the file is copied into install/.. and the third_party
# tree isn't installed alongside it. The driver raises a clear error in
# that case.
_THIS_FILE = os.path.realpath(__file__)
_REPO_ROOT_GUESS = os.path.abspath(
    os.path.join(os.path.dirname(_THIS_FILE), "..", "..", "..", "..")
)
_VENDOR_DIR = os.path.join(
    _REPO_ROOT_GUESS, "third_party", "astrotech_canfd",
    "servo_bdc_control_ms", "test",
)
if os.path.isdir(_VENDOR_DIR) and _VENDOR_DIR not in sys.path:
    sys.path.insert(0, _VENDOR_DIR)

try:
    from CMR_CANFD import FdCanInterface, ServoController  # type: ignore[import-not-found]  # noqa: E402
except ImportError as exc:
    # Distinguish the two failure modes the user can hit here:
    #   1. pyserial-asyncio missing  (CMR_CANFD/FdCanInterface.py top-level
    #      `import serial_asyncio`). Apt doesn't ship it under Humble's
    #      python3.10; install via pip.
    #   2. CMR_CANFD path missing (colcon installed without --symlink-install,
    #      or the third_party tree was moved).
    msg = str(exc)
    if "serial_asyncio" in msg:
        hint = (
            "missing pyserial-asyncio. install with: "
            "/usr/bin/python3 -m pip install --user pyserial-asyncio"
        )
    else:
        hint = (
            f"could not find CMR_CANFD at {_VENDOR_DIR}. If you ran "
            "colcon build WITHOUT --symlink-install, the vendored library "
            "isn't reachable from the installed driver -- re-run "
            "`colcon build --symlink-install`, or ensure CMR_CANFD is on "
            "PYTHONPATH."
        )
    raise ImportError(f"RealMixingServoDriver: {hint} ({exc})") from exc

# Service-callback dispatch timeout. The goto path is one CAN frame +
# move_settle_s (so the chamber gets to the target before we return); we
# add a generous 5 s buffer on top of move_settle_s for the timeout.
_DISPATCH_TIMEOUT_BUFFER_S = 5.0


class RealMixingServoDriver:
    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._port = cfg.get("real_port",
                             "/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00")
        self._can_id = int(cfg.get("real_can_id", 26))
        self._servo_id = int(cfg.get("real_servo_id", 15))
        self._bitrate = int(cfg.get("real_bitrate", 1_000_000))
        self._settle_s = float(cfg.get("move_settle_s", 5.0))

        self._loop: asyncio.AbstractEventLoop | None = None
        self._fd: FdCanInterface | None = None
        self._servo: ServoController | None = None
        self._ready = threading.Event()
        self._init_error: str | None = None
        self._stop = threading.Event()
        self._current_deg = 0
        # Serializes goto/set_home so two rapid panel clicks issue frames
        # back-to-back rather than overlapping mid-settle. The lock is
        # owned by the asyncio loop; service callbacks dispatch onto it.
        self._cmd_lock: asyncio.Lock | None = None

        self._set_angle_srv = node.create_service(
            SetMixingServoAngle, cfg["set_angle_service"], self._on_set_angle
        )
        self._set_home_srv = node.create_service(
            Trigger, cfg["set_home_service"], self._on_set_home
        )
        self._state_pub = node.create_publisher(
            Int32, cfg["state_topic"], 10
        )
        rate_hz = float(cfg["state_rate_hz"])
        self._state_timer = node.create_timer(1.0 / rate_hz, self._tick)

        node.context.on_shutdown(self._on_shutdown)

        self._thread = threading.Thread(
            target=self._thread_main,
            name="mixing_servo_real_loop",
            daemon=True,
        )
        self._thread.start()

        # Wait for the bus to come up before logging "ready"; otherwise
        # the very first service call after launch races configure_bus
        # and looks like a flake.
        if not self._ready.wait(timeout=10.0):
            node.get_logger().error(
                f"RealMixingServoDriver: open/configure_bus did not "
                f"complete within 10 s. last error: {self._init_error}"
            )
            return

        node.get_logger().info(
            f"RealMixingServoDriver up: "
            f"set_angle={cfg['set_angle_service']} "
            f"set_home={cfg['set_home_service']} "
            f"(port={self._port}, can_id={self._can_id}, "
            f"servo_id={self._servo_id}, bitrate={self._bitrate}, "
            f"move_settle={self._settle_s} s)"
        )

    def _on_shutdown(self) -> None:
        self._stop.set()

    def _thread_main(self) -> None:
        try:
            asyncio.run(self._async_main())
        except Exception as exc:  # pragma: no cover - hardware path
            self._init_error = str(exc)
            self._ready.set()  # unblock __init__ so it can log the error
            self._node.get_logger().error(
                f"mixing servo loop crashed: {exc}"
            )

    async def _async_main(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._cmd_lock = asyncio.Lock()
        try:
            self._fd = FdCanInterface(port=self._port, baud=115200)
            await self._fd.open()
            # Manual bring-up: vendored configure_bus() hardcodes 500 kbps.
            # Same six commands as scripts/mixing_servo_min.py.
            for cmd in (
                "can off",
                f"conf set can.bitrate {self._bitrate}",
                "conf set can.fdcan_frame off",
                "conf set can.bitrate_switch on",
                "conf set can.termination on",
                "can on",
            ):
                await self._fd._send_command(cmd)
            self._servo = ServoController(
                can=self._fd,
                servo_id=self._servo_id,
                can_id=self._can_id,
            )
        except Exception as exc:  # pragma: no cover - hardware path
            self._init_error = f"open/configure_bus failed: {exc}"
            self._ready.set()
            return

        self._ready.set()

        try:
            # Park: just hold the loop alive so service callbacks can
            # dispatch coroutines onto it via run_coroutine_threadsafe.
            while not self._stop.is_set():
                await asyncio.sleep(0.2)
        finally:
            try:
                if self._fd is not None:
                    await self._fd.close()
            except Exception as exc:  # pragma: no cover - hardware path
                self._node.get_logger().warning(
                    f"fd.close on shutdown raised {exc}"
                )

    async def _do_goto(self, angle_deg: int) -> None:
        assert self._fd is not None and self._servo is not None
        assert self._cmd_lock is not None
        async with self._cmd_lock:
            # Same frame as mixing_servo_min.py: control_mode=0,
            # control_data=N, clear_faults=1, flags="FB".
            frame = self._servo._make_control_frame(
                control_mode=0, control_data=angle_deg, clear_faults=1,
            )
            await self._fd.write_frame(
                std_id=self._servo.can_id,
                data_hex=frame.hex(),
                flags="FB",
            )
            # Hold for move_settle_s before returning -- the firmware
            # races mechanical motion against the next bus event, and the
            # user-side workaround in mixing_servo_min.py is the same
            # idea (--hold-s). With the lock held, two rapid panel
            # clicks queue rather than collide.
            if self._settle_s > 0:
                await asyncio.sleep(self._settle_s)

    async def _do_set_home(self) -> None:
        assert self._servo is not None
        assert self._cmd_lock is not None
        async with self._cmd_lock:
            await self._servo.set_home(0)
            # Match mixing_servo_min.py's 50 ms settle between set_home
            # and any subsequent frame.
            await asyncio.sleep(0.05)

    def _dispatch(self, coro):
        if self._loop is None or not self._ready.is_set():
            raise RuntimeError(
                "driver not ready (open/configure_bus didn't complete)"
            )
        if self._init_error is not None:
            raise RuntimeError(self._init_error)
        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return fut.result(timeout=self._settle_s + _DISPATCH_TIMEOUT_BUFFER_S)

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
        try:
            self._dispatch(self._do_goto(deg))
        except Exception as exc:
            response.success = False
            response.message = f"goto failed: {exc}"
            self._node.get_logger().error(response.message)
            return response
        self._current_deg = deg
        response.success = True
        response.message = f"goto {deg} deg"
        return response

    def _on_set_home(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        try:
            self._dispatch(self._do_set_home())
        except Exception as exc:
            response.success = False
            response.message = f"set_home failed: {exc}"
            self._node.get_logger().error(response.message)
            return response
        self._current_deg = 0
        response.success = True
        response.message = "set_home(0) ok"
        return response

    def _tick(self) -> None:
        msg = Int32()
        msg.data = int(self._current_deg)
        self._state_pub.publish(msg)
