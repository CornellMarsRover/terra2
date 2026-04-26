"""Mock analysis-sequence action server(s).

One action server per sequence id from the YAML. Each goal is served by
linearly ramping ``progress_pct`` from 0 to 100 over the configured mock
duration, then returning success. Cancelable.

TODO(astrotech-q-3): duration + step names are placeholders. Real BDC-driven
sequences have discrete phases (e.g. "arm pre-position", "lower auger",
"drill", "retract", "mix", "scan"); the panel design assumes whatever
strings we put in ``current_step`` show up in the GCS. Replace here when
real sequence steps are known.

TODO(astrotech-q-4): the goal field is named ``sequence_id``. If the team
decides ``site_num`` / "site id" is the correct semantic, rename the field
in ``action/RunAnalysisSequence.action`` and the references in this file.
"""

from __future__ import annotations

import threading
import time
from typing import Dict

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from cmr_msgs.action import RunAnalysisSequence


# Mock phases. Not real; see TODO(astrotech-q-3).
_PHASES = [
    (0.0, "starting"),
    (25.0, "running"),
    (75.0, "finalizing"),
    (100.0, "done"),
]

_TICK_HZ = 5.0  # feedback rate while a sequence is running


def _phase_for(progress_pct: float) -> str:
    """Return the current-step label for a progress value."""
    label = _PHASES[0][1]
    for threshold, name in _PHASES:
        if progress_pct >= threshold:
            label = name
    return label


class MockAnalysisSequencer:
    """One action server per sequence id."""

    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._duration_s = float(cfg["mock_duration_sec"])
        self._servers: Dict[int, ActionServer] = {}
        # Guard against concurrent goals across the (currently two) servers:
        # the mock is single-threaded at heart, so reject a new goal if any
        # sequence is already running.
        self._busy_lock = threading.Lock()
        self._busy = False

        for seq_cfg in cfg["sequences"]:
            seq_id = int(seq_cfg["id"])
            action_name = seq_cfg["action"]
            self._servers[seq_id] = ActionServer(
                node,
                RunAnalysisSequence,
                action_name,
                execute_callback=self._make_execute_callback(seq_id),
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
            )
            node.get_logger().info(
                f"MockAnalysisSequencer up: action={action_name} id={seq_id}"
            )

    def _goal_callback(self, _goal_request) -> GoalResponse:
        with self._busy_lock:
            if self._busy:
                self._node.get_logger().warn(
                    "rejecting analysis goal; another sequence is running"
                )
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _make_execute_callback(self, seq_id: int):
        def execute_callback(goal_handle: ServerGoalHandle):
            return self._execute(seq_id, goal_handle)
        return execute_callback

    def _execute(
        self, seq_id: int, goal_handle: ServerGoalHandle
    ) -> RunAnalysisSequence.Result:
        request = goal_handle.request
        # TODO(astrotech-q-4): validate sequence_id against accepted values.
        if int(request.sequence_id) != seq_id:
            result = RunAnalysisSequence.Result()
            result.success = False
            result.result_message = (
                f"sequence_id {request.sequence_id} does not match "
                f"server id {seq_id}"
            )
            goal_handle.abort()
            return result

        with self._busy_lock:
            self._busy = True
        try:
            return self._run(seq_id, goal_handle)
        finally:
            with self._busy_lock:
                self._busy = False

    def _run(
        self, seq_id: int, goal_handle: ServerGoalHandle
    ) -> RunAnalysisSequence.Result:
        self._node.get_logger().info(
            f"analysis seq {seq_id}: starting ({self._duration_s} s)"
        )
        feedback = RunAnalysisSequence.Feedback()
        result = RunAnalysisSequence.Result()

        start = time.monotonic()
        tick = 1.0 / _TICK_HZ
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.result_message = f"sequence {seq_id} canceled"
                self._node.get_logger().info(result.result_message)
                return result

            elapsed = time.monotonic() - start
            pct = min(100.0, 100.0 * elapsed / self._duration_s)
            feedback.progress_pct = float(pct)
            feedback.current_step = _phase_for(pct)
            goal_handle.publish_feedback(feedback)

            if pct >= 100.0:
                break
            time.sleep(tick)

        goal_handle.succeed()
        result.success = True
        result.result_message = f"sequence {seq_id} complete"
        self._node.get_logger().info(result.result_message)
        return result
