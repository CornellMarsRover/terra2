from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("astronaut_autonomy"))
    mission_config = str(share / "config" / "astronaut_mission.yaml")
    fusion_config = str(share / "config" / "command_fusion.yaml")
    return LaunchDescription(
        [
            Node(
                package="astronaut_autonomy",
                executable="astronaut_command_fusion",
                name="astronaut_command_fusion",
                output="screen",
                parameters=[fusion_config],
            ),
            Node(
                package="astronaut_autonomy",
                executable="astronaut_mission",
                name="astronaut_mission",
                output="screen",
                parameters=[
                    mission_config,
                    {"enable_drive_adapter": False},
                ],
            ),
            Node(
                package="astronaut_autonomy",
                executable="astronaut_mission_simulator",
                name="astronaut_mission_simulator",
                output="screen",
            ),
        ]
    )
