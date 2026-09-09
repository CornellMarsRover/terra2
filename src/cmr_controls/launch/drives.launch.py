from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('active_source', default_value='teleop'),
        Node(
            package='cmr_rovernet',
            executable='drive_command_mux',
            name='drive_command_mux',
            parameters=[{'active_source': LaunchConfiguration('active_source')}],
        ),
        Node(
            package='cmr_controls',
            executable='swerve_controller_node',
            name='swerve_controller_node',
            output='screen',
        ),
    ])
