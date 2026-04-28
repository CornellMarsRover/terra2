from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')
    camera_index = LaunchConfiguration('camera_index')
    camera_device = LaunchConfiguration('camera_device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Start the camera/ArUco autonomous typing detector',
        ),
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='OpenCV camera index for autonomous typing, e.g. 2 for /dev/video2',
        ),
        DeclareLaunchArgument(
            'camera_device',
            default_value='',
            description='Explicit V4L2 camera device path, e.g. /dev/video2',
        ),
        Node(
            package='autonomous_typing_package',
            executable='autonomous_typing',
            name='autonomous_typing_publisher',
            output='screen', #This option directs logs to the console
            condition=IfCondition(use_camera),
            parameters=[{
                'camera_index': camera_index,
                'camera_device': camera_device,
            }],
        ),
        Node(
            package='autonomous_typing_package',
            executable='arm_coordinator',
            name='arm_coordinator',
            output='screen',
        ),
    ])
