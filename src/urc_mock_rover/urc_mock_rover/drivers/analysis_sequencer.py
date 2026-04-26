"""Mock analysis-sequence action server(s).

One action server per sequence id from the YAML. Each goal runs N fake
steps of fixed duration; feedback is published once per second with the
current step index, total steps, a step description, and elapsed seconds.

Cancel = hard abort. The action returns ``success=false`` with a
"cancelled at step N" message. The rover is left in whatever state it
was in. There is no pause / resume; that is deferred to post-URC
(see docs/post_urc_backlog.md).

The sequencer is **stateless across goals**. No resume token, no
persistent step index. Each new goal starts at step 1.

TODO(astrotech-q-3): real fluidic-protocol step set is unknown; mock
uses generic placeholder names.

TODO(astrotech-q-4): `sequence_id` semantics still unconfirmed.
"""

from __future__ import annotations

import threading
import time
from typing import Dict

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from cmr_msgs.action import RunAnalysisSequence


_FEEDBACK_HZ = 1.0  # one feedback message per second while running


class MockAnalysisSequencer:
    """One action server per sequence id."""

    def __init__(self, node: Node, cfg: dict) -> None:
        self._node = node
        self._num_steps = int(cfg.get("num_steps", 5))
        self._step_duration_s = float(cfg.get("step_duration_sec", 2.0))

        self._busy_lock = threading.Lock()
        self._busy = False

        self._servers: Dict[int, ActionServer] = {}
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
                f"MockAnalysisSequencer up: action={action_name} id={seq_id} "
                f"(steps={self._num_steps}, "
                f"step_duration={self._step_duration_s} s)"
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
        if int(request.sequence_id) != seq_id:
            result = RunAnalysisSequence.Result()
            result.success = False
            result.message = (
                f"sequence_id {request.sequence_id} does not match "
                f"server id {seq_id}"
            )
            result.last_completed_step = 0
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
            f"analysis seq {seq_id}: starting "
            f"({self._num_steps} steps × {self._step_duration_s} s)"
        )

        feedback = RunAnalysisSequence.Feedback()
        feedback.total_steps = self._num_steps

        result = RunAnalysisSequence.Result()
        start_time = time.monotonic()
        feedback_period = 1.0 / _FEEDBACK_HZ

        for step in range(1, self._num_steps + 1):
            step_start = time.monotonic()
            feedback.current_step = step
            feedback.current_step_description = (
                f"step {step}/{self._num_steps}: mock step {step}"
            )

            while True:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = (
                        f"cancelled at step {step}/{self._num_steps}"
                    )
                    result.last_completed_step = step - 1
                    self._node.get_logger().info(result.message)
                    return result

                feedback.elapsed_seconds = float(
                    time.monotonic() - start_time
                )
                goal_handle.publish_feedback(feedback)

                step_elapsed = time.monotonic() - step_start
                if step_elapsed >= self._step_duration_s:
                    break
                time.sleep(min(feedback_period, self._step_duration_s - step_elapsed))

        goal_handle.succeed()
        result.success = True
        result.message = f"sequence {seq_id} complete"
        result.last_completed_step = self._num_steps
        self._node.get_logger().info(result.message)
        return result
