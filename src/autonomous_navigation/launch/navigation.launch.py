# launch/navigation_launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    # Changed default_value to false for test
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the launch directory
    package_share = get_package_share_directory('autonomous_navigation')

    control_loop = Node(
        package='autonomous_navigation',
        executable='nav_control_loop',
        name='nav_control_loop',
        output='screen',
        parameters=[
            {'real': False},
            {'max_linear_vel': 0.5},
            {'waypoint_tolerance': 2.0},
        ]
    )

    path_planner = Node(
        package='autonomous_navigation',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[
            {'visualize': True},
            {'real': False},
        ]
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)

    # Add the control loop node
    ld.add_action(control_loop)

    # Add the path planner node
    ld.add_action(path_planner)

    return ld
