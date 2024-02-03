from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('cmr_arm_sim')
    
    # Declare the path to the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf/arm_2024.urdf.xacro')
    
    # Declare robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )
    
    # Declare RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', os.path.join(pkg_share, 'config/my_robot.rviz')],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz_node
    ])
