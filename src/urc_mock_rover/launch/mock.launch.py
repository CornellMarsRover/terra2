"""Launch the mock rover alongside the Phase 1 Foxglove bridge.

``ros2 launch urc_mock_rover mock.launch.py`` — one command, one terminal,
fake rover plus ``ws://localhost:8765`` up and serving Foxglove.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# Phase 1 bridge launch lives at the repo root ``launch/`` folder, not
# inside any package's share dir. Locate it relative to this file.
#   src/urc_mock_rover/launch/mock.launch.py
#     -> ../../../launch/gcs_bridge.launch.py
_GCS_BRIDGE_LAUNCH = (
    Path(__file__).resolve().parents[3] / "launch" / "gcs_bridge.launch.py"
)


def generate_launch_description() -> LaunchDescription:
    actions = [
        Node(
            package="urc_mock_rover",
            executable="mock_rover_node",
            name="mock_rover_node",
            output="screen",
        ),
    ]

    if _GCS_BRIDGE_LAUNCH.is_file():
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(_GCS_BRIDGE_LAUNCH))
            )
        )
    else:
        # Fall back to starting foxglove_bridge directly if the repo-root
        # launch file isn't visible (e.g. installed binary distribution).
        actions.append(
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[{"port": 8765, "address": "0.0.0.0"}],
            )
        )

    return LaunchDescription(actions)
