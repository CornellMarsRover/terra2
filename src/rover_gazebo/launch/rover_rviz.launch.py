# rover_gazebo/launch/visualize_robot.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ***********************
    # Launch Configuration Arguments
    # ***********************
    
    # Argument for specifying the RViz config file
    rviz_config_file = LaunchConfiguration('rviz_config')
    
    # Declare Launch Arguments
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('rover_description'),
            'rviz',
            'description.rviz'
        ),
        description='RViz Config File'
    )
    
    # ***********************
    # Define RViz Node
    # ***********************
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # ***********************
    # Create LaunchDescription and populate
    # ***********************
    
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_rviz_config_cmd)
    
    # Add the RViz node
    ld.add_action(rviz_node)
    
    return ld
