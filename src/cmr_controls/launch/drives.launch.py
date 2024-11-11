from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmr_controls',
            executable='swerve_controller_node',
            name='swerve_controller_node',
            output='screen',
        ),
    ])
