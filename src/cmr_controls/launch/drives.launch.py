from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    drive_config = path.join(
        get_package_share_directory('cmr_rovernet'), 'config', 'drivesnet.toml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('active_source', default_value='teleop'),
        Node(
            package='cmr_rovernet',
            executable='drive_command_mux',
            name='drive_command_mux',
            parameters=[{'active_source': LaunchConfiguration('active_source')}],
        ),
        Node(
            package='cmr_rovernet',
            executable='usama_control_testing_node',
            name='drivesnet',
            output='screen',
            parameters=[{'config_path': drive_config}],
        ),
    ])
