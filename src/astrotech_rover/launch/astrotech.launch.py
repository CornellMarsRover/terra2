"""Launch the Astrotech rover node alongside the Foxglove bridge.

``ros2 launch astrotech_rover astrotech.launch.py`` — one command, one
terminal: the astrotech_node (real drivers by default; URC_*_MOCK=1 to
sim a feature) plus ``ws://localhost:8765`` serving Foxglove.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# The bridge launch lives at the repo root ``launch/`` folder, not
# inside any package's share dir. Locate it relative to this file.
#   src/astrotech_rover/launch/astrotech.launch.py
#     -> ../../../launch/gcs_bridge.launch.py
_GCS_BRIDGE_LAUNCH = (
    Path(__file__).resolve().parents[3] / "launch" / "gcs_bridge.launch.py"
)


def generate_launch_description() -> LaunchDescription:
    actions = [
        Node(
            package="astrotech_rover",
            executable="astrotech_node",
            name="astrotech_node",
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
