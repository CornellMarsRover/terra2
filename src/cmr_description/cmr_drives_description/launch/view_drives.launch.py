

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
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
from launch.launch_description_sources import PythonLaunchDescriptionSource




def generate_launch_description():
    pkg_share = FindPackageShare(package='cmr_drives_description').find('cmr_drives_description')
    bringup_dir = get_package_share_directory('nav2_bringup')
    slam_dir = get_package_share_directory('slam_toolbox')
    default_rviz_config_path = os.path.join(pkg_share, 'config/drives_view.rviz')
    default_world_path= os.path.join(pkg_share, 'world/my_world.sdf')
    slam_launch_file = os.path.join(slam_dir, 'launch', 'online_async_launch.py')
    # Initialize arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rvizconfig")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    world = LaunchConfiguration("world")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_desc_pkg_cmd = DeclareLaunchArgument(
            "description_package",
            default_value="cmr_drives_description",
    )
    declare_desc_file_cmd = DeclareLaunchArgument(
            "description_file",
            default_value="drives.urdf.xacro",
    )
    declare_rvizconfig_cmd = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file')
    declare_sim_time_cmd = DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time')
    declare_params_file_cmd = DeclareLaunchArgument(name='params_file', 
                                            default_value=os.path.join(pkg_share, 'config', 'nav_params.yaml'),
                                            description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_use_respawn_cmd = DeclareLaunchArgument(name='use_respawn', default_value='True',
                                            description='Flag to enable respawn of nav2 nodes if they die')
    declare_world_cmd = DeclareLaunchArgument(name='world', default_value=default_world_path,
                                            description='Full path to world file to load')
    declare_slam_params_cmd = DeclareLaunchArgument(name='slam_params_file',
                                            default_value=os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
                                            description='Full path to the ROS2 parameters file to use for all launched nodes')

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
                         "use_sim_time": sim_time}

    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': sim_time}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': sim_time}]
    )
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'cmr_drives', '-topic', 'robot_description'],
        output='screen'
    )
    # start_sync_slam_toolbox_node = Node(
    #     parameters=[
    #       os.path.join(pkg_share, 'config/mapper_params_online_sync.yaml'),
    #       {'use_sim_time': sim_time}
    #     ],
    #     package='slam_toolbox',
    #     executable='sync_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen')

    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'use_sim_time': sim_time,
                          'slam_params_file': slam_params_file}.items(),
    )
    
    start_gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 
                'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world], output='screen')

    # nav_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
    #     launch_arguments={#'namespace': namespace,
    #                       #'use_namespace': use_namespace,
    #                       'map': "",
    #                       'use_sim_time': sim_time,
    #                       'params_file': params_file,
    #                       'slam': 'True',
    #                       #'autostart': autostart,
    #                       'use_composition': 'True',
    #                       'use_respawn': use_respawn
    #                       }.items()
    # )
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': sim_time,
                          'params_file': params_file,
                          'use_respawn': use_respawn,
                          'use_composition': 'True',
                         }.items()
    )

    return LaunchDescription([
        declare_desc_pkg_cmd,
        declare_desc_file_cmd,
        declare_rvizconfig_cmd,
        declare_sim_time_cmd,
        declare_params_file_cmd,
        declare_use_respawn_cmd,
        declare_world_cmd,
        declare_slam_params_cmd,
        start_gazebo,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        start_slam_toolbox,
        rviz_node,
        nav_bringup
    ])
