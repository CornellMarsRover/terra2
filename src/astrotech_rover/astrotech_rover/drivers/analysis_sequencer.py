"""Generic analysis sequencer + the Foxglove-native ROS interface (Option B).

Runs a ``Sequence`` (from :mod:`analysis_protocol`) step by step through an
*executor* that does the actual device I/O. Two executors exist, both driven by
this identical sequencer:

* :class:`~astrotech_rover.drivers.analysis_real.RealAnalysisExecutor` — CMR_CANFD
  hardware (default).
* :class:`~astrotech_rover.drivers.mock.analysis.MockAnalysisExecutor` — no
  hardware, logs each action (``URC_ANALYSIS_MOCK=1``).

ROS interface (Foxglove panels have no ROS action client):

* start service   ``StartAnalysisSequence(sequence_id) -> success, message``
* cancel service  ``std_srvs/Trigger``  — hard abort: stops every motor
* status topic    ``AnalysisStatus``    — published at ``status_rate_hz``

There is no board feedback, so progress is commanded-only and a cancel is a
hard abort (the rig is left as-is; recovery is the operator's problem).
"""

from __future__ import annotations

import threading
import time

from rclpy.node import Node
from std_srvs.srv import Trigger

from cmr_msgs.msg import AnalysisStatus
from cmr_msgs.srv import StartAnalysisSequence

from astrotech_rover.drivers import analysis_protocol as proto


class AnalysisExecutor:
    """Performs device actions for the sequencer. Subclassed real / mock."""

    @property
    def ready(self) -> bool:
        return True

    @property
    def error(self) -> str | None:
        return None

    def startup(self) -> None:
        """Bring up hardware (may block). Raise / set error on failure."""

    def shutdown(self) -> None:
        """Release hardware."""

    def perform(self, action: proto.Action) -> None:
        """Issue one action and return (the step's wait covers run time)."""
        raise NotImplementedError

    def stop_all(self) -> None:
        """Hard abort: stop every motor. Cancelling a task does NOT stop a
        commanded move, so this must send explicit stop frames."""


class AnalysisSequencer:
    def __init__(self, node: Node, cfg: dict, executor: AnalysisExecutor) -> None:
        self._node = node
        self._ex = executor
        self._status_rate_hz = float(cfg.get("status_rate_hz", 5.0))

        self._lock = threading.Lock()
        self._running = False
        self._active_id = 0
        self._current_step = 0
        self._total_steps = 0
        self._step_desc = ""
        self._elapsed = 0.0
        self._last_success = False
        self._last_message = ""
        self._last_completed_step = 0
        self._cancel = threading.Event()
        self._worker: threading.Thread | None = None

        self._status_pub = node.create_publisher(
            AnalysisStatus, cfg["status_topic"], 10
        )
        self._start_srv = node.create_service(
            StartAnalysisSequence, cfg["start_service"], self._on_start
        )
        self._cancel_srv = node.create_service(
            Trigger, cfg["cancel_service"], self._on_cancel
        )
        period = 1.0 / max(self._status_rate_hz, 0.1)
        self._status_timer = node.create_timer(period, self._publish_status)
        node.context.on_shutdown(self._on_shutdown)

        # Bring the executor up. A hardware failure must NOT crash the node --
        # log it and advertise anyway, so the panel sees the services and a
        # start returns a clean "not ready" error.
        try:
            self._ex.startup()
        except Exception as exc:  # noqa: BLE001 - isolate hardware startup
            node.get_logger().error(f"analysis executor startup failed: {exc}")

        node.get_logger().info(
            f"AnalysisSequencer up: start={cfg['start_service']} "
            f"cancel={cfg['cancel_service']} status={cfg['status_topic']} "
            f"({len(proto.SEQUENCES)} sequences)"
        )

    # --- shutdown -------------------------------------------------------
    def _on_shutdown(self) -> None:
        self._cancel.set()
        try:
            self._ex.shutdown()
        except Exception:  # noqa: BLE001
            pass

    # --- services -------------------------------------------------------
    def _on_start(
        self,
        req: StartAnalysisSequence.Request,
        resp: StartAnalysisSequence.Response,
    ) -> StartAnalysisSequence.Response:
        seq = proto.get_sequence(req.sequence_id)
        if seq is None:
            resp.success = False
            resp.message = (
                f"unknown sequence_id {req.sequence_id}; "
                f"valid: {sorted(proto.SEQUENCES)}"
            )
            return resp
        if not self._ex.ready:
            resp.success = False
            resp.message = (
                f"analysis hardware not ready: {self._ex.error or 'unavailable'}"
            )
            return resp
        with self._lock:
            if self._running:
                resp.success = False
                resp.message = f"busy: sequence {self._active_id} already running"
                return resp
            self._running = True
            self._active_id = seq.sequence_id
            self._total_steps = len(seq.steps)
            self._current_step = 0
            self._step_desc = "starting"
            self._elapsed = 0.0
        self._cancel.clear()
        self._worker = threading.Thread(
            target=self._run, args=(seq,), name="analysis_seq", daemon=True
        )
        self._worker.start()
        resp.success = True
        resp.message = f"started sequence {seq.sequence_id}: {seq.name}"
        self._node.get_logger().info(resp.message)
        return resp

    def _on_cancel(self, _req, resp: Trigger.Response) -> Trigger.Response:
        with self._lock:
            running = self._running
            active = self._active_id
        if not running:
            resp.success = False
            resp.message = "no sequence running"
            return resp
        self._cancel.set()
        try:
            self._ex.stop_all()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f"stop_all on cancel raised: {exc}")
        resp.success = True
        resp.message = f"cancel requested for sequence {active}"
        self._node.get_logger().info(resp.message)
        return resp

    # --- worker ---------------------------------------------------------
    def _run(self, seq: proto.Sequence) -> None:
        start = time.monotonic()
        completed = 0
        cancelled = False
        try:
            for idx, step in enumerate(seq.steps, start=1):
                if self._cancel.is_set():
                    cancelled = True
                    break
                with self._lock:
                    self._current_step = idx
                    self._step_desc = step.description
                    self._elapsed = time.monotonic() - start
                for action in step.actions:
                    try:
                        self._ex.perform(action)
                    except Exception as exc:  # noqa: BLE001
                        self._node.get_logger().error(
                            f"step {idx} action {action.device} failed: {exc}"
                        )
                # Wait wait_s, polling for cancel (wakes early on cancel).
                step_start = time.monotonic()
                while time.monotonic() - step_start < step.wait_s:
                    if self._cancel.is_set():
                        cancelled = True
                        break
                    with self._lock:
                        self._elapsed = time.monotonic() - start
                    self._cancel.wait(0.05)
                if cancelled:
                    break
                completed = idx
        finally:
            if cancelled:
                try:
                    self._ex.stop_all()
                except Exception:  # noqa: BLE001
                    pass
            with self._lock:
                self._running = False
                self._active_id = 0
                self._current_step = 0
                self._step_desc = ""
                self._elapsed = time.monotonic() - start
                self._last_completed_step = completed
                if cancelled:
                    self._last_success = False
                    self._last_message = (
                        f"cancelled at step {completed + 1}/{len(seq.steps)}"
                    )
                else:
                    self._last_success = True
                    self._last_message = f"{seq.name} complete"
            self._node.get_logger().info(self._last_message)

    # --- status ---------------------------------------------------------
    def _publish_status(self) -> None:
        msg = AnalysisStatus()
        with self._lock:
            msg.running = self._running
            msg.active_sequence_id = int(self._active_id)
            msg.current_step = int(self._current_step)
            msg.total_steps = int(self._total_steps)
            msg.current_step_description = self._step_desc
            msg.elapsed_seconds = float(self._elapsed)
            msg.last_success = self._last_success
            msg.last_message = self._last_message
            msg.last_completed_step = int(self._last_completed_step)
        self._status_pub.publish(msg)
