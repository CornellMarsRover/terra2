

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    pkg_share = FindPackageShare(package='cmr_drives_description').find('cmr_drives_description')
    default_rviz_config_path = os.path.join(pkg_share, 'config/drives_view.rviz')
    world_path= os.path.join(pkg_share, 'world/my_world.sdf')
    # Initialize arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rvizconfig")

    # Generate URDF file via Xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content, 
                         "use_sim_time": True}

    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
        # condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'cmr_drives', '-topic', 'robot_description'],
        output='screen'
    )
    start_sync_slam_toolbox_node = Node(
        parameters=[
          os.path.join(get_package_share_directory("cmr_test"), 'config/mapper_params_online_sync.yaml'),
          {'use_sim_time': sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            "description_package",
            default_value="cmr_drives_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="drives.urdf.xacro",
        ),
        DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        start_sync_slam_toolbox_node,
        rviz_node,
    ])
