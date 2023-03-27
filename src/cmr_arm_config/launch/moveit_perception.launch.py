

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import UnlessCondition, IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():
    pkg_share = get_package_share_directory('cmr_arm_config')
    world_path = os.path.join(pkg_share, 'gazebo', 'arm_world.sdf')
    launch_moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'demo.launch.py')
        )
    )

    spawn_into_gazebo = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'cmr_arm', '-topic', 'robot_description', 
            '-x', '0',
            '-y', '0',
            '-z', '0.5'],
        output='screen'
    )

    launch_gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 
                                        'libgazebo_ros_factory.so', 
                                        world_path], output='screen')

    return LaunchDescription([
        launch_moveit_demo,
        launch_gazebo,
        spawn_into_gazebo
    ])
