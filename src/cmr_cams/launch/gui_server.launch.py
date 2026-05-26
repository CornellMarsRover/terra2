"""Deprecated — superseded by ``launch/gcs_bridge.launch.py`` at the repo root.

This file used to start ``foxglove_bridge`` on port 8765 with no QoS overrides.
Running it alongside the new GCS bridge launch would double-bind port 8765 and
cause one of the two bridges to fail with "address already in use".

Keep the file (empty launch) so any script that still references
``gui_server.launch.py`` doesn't crash on a missing file, but don't start the
bridge here. Use::

    ros2 launch launch/gcs_bridge.launch.py

from the repo root instead.
"""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "[cmr_cams/gui_server.launch.py] This launch is deprecated. "
                    "Foxglove bridge is now started from launch/gcs_bridge.launch.py "
                    "at the repo root. No bridge was started by this file."
                )
            ),
        ]
    )
