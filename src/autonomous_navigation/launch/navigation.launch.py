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

    nav_ackerman_real = Node(
        package='autonomous_navigation',
        executable='ackerman_real',
        name='nav_ackerman_real',
        output='screen',
        parameters=[
            {'real': False},
            {'max_linear_vel': 0.5},
            {'waypoint_tolerance': 10.0},
        ]
    )

    # Assign desired control node to be launched
    #waypoint_controller_node = nav_ackerman
    #waypoint_controller_node = nav_control_methods_demo
    waypoint_controller_node = nav_ackerman_real

    ld = LaunchDescription()

    # Add the declared launch arguments
    ld.add_action(declare_use_sim_time)

    # Add the waypoint controller node
    ld.add_action(waypoint_controller_node)

    return ld
