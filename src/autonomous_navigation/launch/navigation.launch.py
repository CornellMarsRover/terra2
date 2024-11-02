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

    declare_waypoints_file = DeclareLaunchArgument(
        'waypoints_file',
        default_value='config/waypoints.yaml',
        description='Path to the waypoints YAML file'
    )

    declare_waypoints_file_real = DeclareLaunchArgument(
        'waypoints_file_real',
        default_value='config/waypoints_real.yaml',
        description='Path to the waypoints YAML file'
    )


    # Get the launch directory
    package_share = get_package_share_directory('autonomous_navigation')

    # Different control nodes
    nav_control_methods_demo = Node(
        package='autonomous_navigation',
        executable='control_methods_demo',
        name='nav_control_methods_demo',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'waypoints_file': LaunchConfiguration('waypoints_file')},
            {'max_angular_vel': 0.55},
            {'max_linear_vel': 0.5},
            {'angle_threshold_deg': 10.0},
            {'waypoint_tolerance': 2.0},
            {'proportional_gain': 0.5},
        ]
    )

    nav_ackerman = Node(
        package='autonomous_navigation',
        executable='ackerman',
        name='nav_ackerman',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'waypoints_file': LaunchConfiguration('waypoints_file')},
            {'max_linear_vel': 0.5},
            {'waypoint_tolerance': 2.0},
        ]
    )

    nav_ackerman_real = Node(
        package='autonomous_navigation',
        executable='ackerman_real',
        name='nav_ackerman_real',
        output='screen',
        parameters=[
            {'waypoints_file': LaunchConfiguration('waypoints_file_real')},
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
    ld.add_action(declare_waypoints_file_real)

    # Add the waypoint controller node
    ld.add_action(waypoint_controller_node)

    return ld
