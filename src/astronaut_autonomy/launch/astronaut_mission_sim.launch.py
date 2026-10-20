from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("astronaut_autonomy"))
    config = str(share / "config" / "astronaut_mission.yaml")
    return LaunchDescription(
        [
            Node(
                package="astronaut_autonomy",
                executable="astronaut_mission",
                name="astronaut_mission",
                output="screen",
                parameters=[config, {"enable_drive_adapter": False}],
            ),
            Node(
                package="astronaut_autonomy",
                executable="astronaut_mission_simulator",
                name="astronaut_mission_simulator",
                output="screen",
            ),
        ]
    )
