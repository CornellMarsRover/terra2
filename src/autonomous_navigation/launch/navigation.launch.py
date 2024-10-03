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
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_waypoints_file = DeclareLaunchArgument(
        'waypoints_file',
        default_value='config/waypoints.yaml',
        description='Path to the waypoints YAML file'
    )

    # Get the launch directory
    package_share = get_package_share_directory('autonomous_navigation')

    # Node: Waypoint Controller
    waypoint_controller_node = Node(
        package='autonomous_navigation',
        executable='waypoint_controller',
        name='waypoint_controller',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'waypoints_file': LaunchConfiguration('waypoints_file')},
            {'max_linear_vel': 0.3},
            {'max_angular_vel': 0.3},
            {'waypoint_tolerance': 3.0},
            {'angle_threshold_deg': 13.0},
        ]
    )

    ld = LaunchDescription()

    # Add the declared launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_waypoints_file)

    # Add the waypoint controller node
    ld.add_action(waypoint_controller_node)

    return ld
