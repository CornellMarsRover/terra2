# launch/main_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    gazebo_pkg = 'rover_gazebo'
    navigation_pkg = 'autonomous_navigation'
    launch_dir = 'launch'

    # Paths to launch files
    gazebo_launch_file = os.path.join(
        get_package_share_directory(gazebo_pkg),
        launch_dir,
        'sim.launch.py'
    )
    navigation_launch_file = os.path.join(
        get_package_share_directory(navigation_pkg),
        launch_dir,
        'navigation.launch.py'
    )

    # Include Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
    )

    # Include Navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(navigation_launch)

    return ld
