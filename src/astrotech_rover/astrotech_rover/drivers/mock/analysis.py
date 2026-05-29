"""Mock analysis executor -- no hardware. ``perform`` logs each action and
``stop_all`` is a no-op. The sequencer still runs the real step list with the
real wait times, so the panel shows realistic progress for GUI dev / demos.
Enable with ``URC_ANALYSIS_MOCK=1``.
"""

from __future__ import annotations

from rclpy.node import Node

from astrotech_rover.drivers import analysis_protocol as proto
from astrotech_rover.drivers.analysis_sequencer import (
    AnalysisExecutor,
    AnalysisSequencer,
)


class MockAnalysisExecutor(AnalysisExecutor):
    def __init__(self, node: Node) -> None:
        self._node = node

    def startup(self) -> None:
        self._node.get_logger().info(
            "MockAnalysisExecutor: no hardware; sequence steps are simulated."
        )

    def perform(self, action: proto.Action) -> None:
        if action.kind == proto.SERVO:
            self._node.get_logger().info(
                f"[mock analysis] servo {action.device} -> {action.magnitude:.0f} deg"
            )
        else:
            self._node.get_logger().info(
                f"[mock analysis] pump {action.device} {action.direction} "
                f"{action.magnitude:.0f}s"
            )

    def stop_all(self) -> None:
        self._node.get_logger().info("[mock analysis] stop_all")


def build_mock_analysis(node: Node, cfg: dict) -> AnalysisSequencer:
    return AnalysisSequencer(node, cfg, MockAnalysisExecutor(node))
