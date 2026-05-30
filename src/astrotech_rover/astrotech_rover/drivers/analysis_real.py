"""Real analysis executor -- drives the fluidic pumps / servos over CMR_CANFD.

Default analysis backend in :mod:`astrotech_rover.astrotech_node`
(``URC_ANALYSIS_MOCK=1`` swaps in the no-hardware sim). Owns the fdcanusb
directly. On the two-CAN payload stack, this bus is for BDC pump boards only;
the chamber servo is moving to Arduino serial, so servo actions are skipped
unless explicitly re-enabled in config.

Mirrors the proven CMR_CANFD bring-up in :mod:`mixing_servo_real`: open the
port, manual ``configure_bus`` at 1 Mbit/s (the vendored lib hardcodes a wrong
500 kbit/s), and build one controller per BDC device. BDC pump moves
fire-and-return; the sequencer's per-step wait covers the run time. Cancel ->
``stop_all`` sends ``stop_motor`` to every pump (cancelling the coroutine does
NOT stop a commanded move -- see astrotech_canfd/API_NOTES.md).
"""

from __future__ import annotations

import asyncio
import os
import sys
import threading

from rclpy.node import Node

from astrotech_rover.drivers import analysis_protocol as proto
from astrotech_rover.drivers.analysis_sequencer import (
    AnalysisExecutor,
    AnalysisSequencer,
)

# Vendored CMR_CANFD library (same resolution as mixing_servo_real.py).
_THIS_FILE = os.path.realpath(__file__)
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(_THIS_FILE), "..", ".."))
_VENDOR_DIR = os.path.join(
    _PKG_ROOT, "third_party", "astrotech_canfd", "servo_bdc_control_ms", "test",
)
if os.path.isdir(_VENDOR_DIR) and _VENDOR_DIR not in sys.path:
    sys.path.insert(0, _VENDOR_DIR)

try:
    from CMR_CANFD import (  # type: ignore[import-not-found]  # noqa: E402
        BDCController,
        FdCanInterface,
        ServoController,
    )
except ImportError as exc:
    msg = str(exc)
    if "serial_asyncio" in msg:
        hint = (
            "missing pyserial-asyncio. install with: "
            "/usr/bin/python3 -m pip install --user pyserial-asyncio"
        )
    else:
        hint = (
            f"could not find CMR_CANFD at {_VENDOR_DIR}. re-run "
            "`colcon build --symlink-install`, or ensure CMR_CANFD is on PYTHONPATH."
        )
    raise ImportError(f"RealAnalysisExecutor: {hint} ({exc})") from exc

_DISPATCH_TIMEOUT_S = 10.0


class RealAnalysisExecutor(AnalysisExecutor):
    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._port = os.environ.get(
            "ASTROTECH_BDC_CAN_PORT",
            cfg.get(
                "real_port", "/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00"
            ),
        )
        self._bitrate = int(cfg.get("real_bitrate", 1_000_000))
        self._enable_chamber_servo = bool(cfg.get("enable_chamber_servo", False))
        self._warned_servo_skip = False

        self._loop: asyncio.AbstractEventLoop | None = None
        self._fd: FdCanInterface | None = None
        self._controllers: dict = {}  # device name -> controller
        self._ready_ev = threading.Event()
        self._error: str | None = None
        self._stop = threading.Event()
        self._cmd_lock: asyncio.Lock | None = None
        self._thread: threading.Thread | None = None

    @property
    def ready(self) -> bool:
        return self._ready_ev.is_set() and self._error is None

    @property
    def error(self) -> str | None:
        return self._error

    def startup(self) -> None:
        self._thread = threading.Thread(
            target=self._thread_main, name="analysis_real_loop", daemon=True
        )
        self._thread.start()
        if not self._ready_ev.wait(timeout=12.0):
            self._error = self._error or "open/configure_bus did not finish in 12 s"
        if self._error:
            self._node.get_logger().error(
                f"RealAnalysisExecutor unavailable: {self._error}. The analysis "
                "panel will report 'not ready'; check the fdcanusb / lab mode."
            )
        else:
            self._node.get_logger().info(
                f"RealAnalysisExecutor up: port={self._port} "
                f"bitrate={self._bitrate} ({len(self._controllers)} devices, "
                f"chamber_servo={'enabled' if self._enable_chamber_servo else 'skipped'})"
            )

    def shutdown(self) -> None:
        self._stop.set()

    def _thread_main(self) -> None:
        try:
            asyncio.run(self._async_main())
        except Exception as exc:  # pragma: no cover - hardware path
            self._error = str(exc)
            self._ready_ev.set()

    async def _async_main(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._cmd_lock = asyncio.Lock()
        try:
            self._fd = FdCanInterface(port=self._port, baud=115200)
            await self._fd.open()
            for cmd in (
                "can off",
                f"conf set can.bitrate {self._bitrate}",
                "conf set can.fdcan_frame off",
                "conf set can.bitrate_switch on",
                "conf set can.termination on",
                "can on",
            ):
                await self._fd._send_command(cmd)
            for dev in proto.referenced_devices():
                if dev.kind == proto.SERVO:
                    if not self._enable_chamber_servo:
                        continue
                    self._controllers[dev.name] = ServoController(
                        can=self._fd, servo_id=dev.dev_id, can_id=dev.can_id
                    )
                else:
                    bdc = BDCController(
                        can=self._fd, motor_id=dev.dev_id, can_id=dev.can_id
                    )
                    try:
                        await bdc.clear_faults()
                    except Exception:  # noqa: BLE001
                        pass
                    self._controllers[dev.name] = bdc
        except Exception as exc:  # pragma: no cover - hardware path
            self._error = f"open/configure_bus failed: {exc}"
            self._ready_ev.set()
            return

        self._ready_ev.set()
        try:
            while not self._stop.is_set():
                await asyncio.sleep(0.2)
        finally:
            try:
                if self._fd is not None:
                    await self._fd.close()
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().warning(f"fd.close on shutdown: {exc}")

    def _dispatch(self, coro):
        if self._loop is None or not self.ready:
            raise RuntimeError(self._error or "analysis bus not ready")
        fut = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return fut.result(timeout=_DISPATCH_TIMEOUT_S)

    def perform(self, action: proto.Action) -> None:
        self._dispatch(self._perform_async(action))

    async def _perform_async(self, action: proto.Action) -> None:
        if action.kind == proto.SERVO and not self._enable_chamber_servo:
            if not self._warned_servo_skip:
                self._node.get_logger().warning(
                    "analysis chamber servo actions are skipped because "
                    "analysis.enable_chamber_servo is false; move the "
                    "chamber manually/through the Arduino path for now."
                )
                self._warned_servo_skip = True
            return
        ctrl = self._controllers.get(action.device)
        if ctrl is None:
            raise RuntimeError(f"no controller for device '{action.device}'")
        assert self._cmd_lock is not None and self._fd is not None
        async with self._cmd_lock:
            if action.kind == proto.SERVO:
                # clear_faults=1 is load-bearing (see API_NOTES.md); build the
                # frame the same way mixing_servo_real does.
                frame = ctrl._make_control_frame(
                    control_mode=0, control_data=int(action.magnitude),
                    clear_faults=1,
                )
                await self._fd.write_frame(
                    std_id=ctrl.can_id, data_hex=frame.hex(), flags="FB"
                )
            else:
                secs = max(0, min(proto.PUMP_MAX_SECONDS, int(round(action.magnitude))))
                if action.direction == "rev":
                    await ctrl.move_motor_reverse(secs)
                else:
                    await ctrl.move_motor_forward(secs)

    def stop_all(self) -> None:
        self._dispatch(self._stop_all_async())

    async def _stop_all_async(self) -> None:
        assert self._cmd_lock is not None
        async with self._cmd_lock:
            for ctrl in self._controllers.values():
                try:
                    if isinstance(ctrl, BDCController):
                        await ctrl.stop_motor()
                    else:
                        await ctrl.stop()
                except Exception:  # noqa: BLE001
                    pass


def build_real_analysis(node: Node, cfg: dict) -> AnalysisSequencer:
    return AnalysisSequencer(node, cfg, RealAnalysisExecutor(node, cfg))
