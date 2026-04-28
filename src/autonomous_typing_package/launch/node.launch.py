from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Start the camera/ArUco autonomous typing detector',
        ),
        Node(
            package='autonomous_typing_package',
            executable='autonomous_typing',
            name='autonomous_typing_publisher',
            output='screen', #This option directs logs to the console
            condition=IfCondition(use_camera),
        ),
        Node(
            package='autonomous_typing_package',
            executable='arm_coordinator',
            name='arm_coordinator',
            output='screen',
        ),
    ])
