from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("astronaut_autonomy"))
    config = str(share / "config" / "astronaut_mission.yaml")
    enable_drive_adapter = LaunchConfiguration("enable_drive_adapter")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_drive_adapter",
                default_value="false",
                description=(
                    "Publish astronaut commands to /cmd_vel/autonomy; leave "
                    "false until rover integration is reviewed"
                ),
            ),
            Node(
                package="astronaut_autonomy",
                executable="astronaut_mission",
                name="astronaut_mission",
                output="screen",
                parameters=[
                    config,
                    {
                        "enable_drive_adapter": ParameterValue(
                            enable_drive_adapter, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
