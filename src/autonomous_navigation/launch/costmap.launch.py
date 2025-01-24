from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get the launch directory
    package_share = get_package_share_directory('autonomous_navigation')
    config_folder = os.path.join(package_share, 'config')
    costmap_file = os.path.join(config_folder, 'nav2_costmap_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[
                costmap_file
            ],
        )
    ])
