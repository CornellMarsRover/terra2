# rover_gazebo/launch/main_launch.py
import os
from typing import Final, List, Optional

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ROS Package Share
PKG: Final[str] = "rover_gazebo"

# File Names
SPAWN_LAUNCH_FILE: Final[str] = "spawn_rover.launch.py"
WORLD_LAUNCH_FILE: Final[str] = "launch_world.launch.py"
RVIZ_LAUNCH_FILE: Final[str] = "rover_rviz.launch.py"

# Directory Names
LAUNCH_DIR: Final[str] = "launch"


# File Paths
spawn_launch_path = os.path.join(
    get_package_share_directory(PKG), LAUNCH_DIR, SPAWN_LAUNCH_FILE
)

world_launch_path = os.path.join(
    get_package_share_directory(PKG), LAUNCH_DIR, WORLD_LAUNCH_FILE
)

rviz_launch_path = os.path.join(
    get_package_share_directory(PKG), LAUNCH_DIR, RVIZ_LAUNCH_FILE
)

# Launch Arguments
ARGUMENTS: Optional[List[DeclareLaunchArgument]] = [
    DeclareLaunchArgument(
        "log_level",
        default_value="debug",
        description="Logging level",
    ),
]

def generate_launch_description() -> LaunchDescription:
    """
    Generate Launch Description for Gazebo Sim with RViz Visualization.

    Returns
    -------
        LaunchDescription: Description of the launch-able ROS system
    """
    # Include World Launch File
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([world_launch_path]),
    )

    # Include Robot Spawn Launch File
    swerve_robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawn_launch_path]),
    )

    # Include RViz Launch File
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch_path]),
    )

    # Create LaunchDescription and populate
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(world_launch)
    ld.add_action(swerve_robot_spawn)
    ld.add_action(rviz_launch)

    return ld
