from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmr_controls',
            executable='new_swerve_node',
            name='new_swerve_node',
            output='screen',
        ),
    ])
