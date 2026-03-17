from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_typing_package',
            executable='typing_mission_ros',
            name='autonomous_typing_publisher',
            output='screen', #This option directs logs to the console
        ),
        Node(
            package='autonomous_typing_package',
            executable='arm_coordinator',
            name='arm_coordinator',
            output='screen',
        ),
    ])