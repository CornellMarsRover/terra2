from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    autonomy_share = get_package_share_directory("autonomous_navigation")
    gazebo_share = get_package_share_directory("gazebo_ros")

    robot_file = f"{autonomy_share}/urdf/autonomy_obstacle_bot.urdf"
    obstacle_file = f"{autonomy_share}/models/obstacle_box.sdf"

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    goal_north = LaunchConfiguration("goal_north")
    goal_west = LaunchConfiguration("goal_west")
    obstacle_x = LaunchConfiguration("obstacle_x")
    obstacle_y = LaunchConfiguration("obstacle_y")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gazebo_share}/launch/gazebo.launch.py"),
        launch_arguments={"gui": gui}.items(),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "autonomy_bot", "-file", robot_file, "-x", "0.0", "-y", "0.0", "-z", "0.05"],
        output="screen",
    )

    spawn_obstacle = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "obstacle_box",
            "-file",
            obstacle_file,
            "-x",
            obstacle_x,
            "-y",
            obstacle_y,
            "-z",
            "0.5",
        ],
        output="screen",
    )

    autonomy_nodes = [
        Node(
            package="autonomous_navigation",
            executable="sim_drive_bridge",
            name="sim_drive_bridge",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="autonomous_navigation",
            executable="odom_to_autonomy_pose",
            name="odom_to_autonomy_pose",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="autonomous_navigation",
            executable="sim_goal_publisher",
            name="sim_goal_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "goal_north": goal_north,
                    "goal_west": goal_west,
                }
            ],
        ),
        Node(
            package="autonomous_navigation",
            executable="sim_vision_obstacle_detector",
            name="sim_vision_obstacle_detector",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "mode": "auto",
                    "obstacle_x": obstacle_x,
                    "obstacle_y": obstacle_y,
                }
            ],
        ),
        Node(
            package="autonomous_navigation",
            executable="global_planner",
            name="global_planner",
            output="screen",
            parameters=[{"real": False, "use_sim_time": use_sim_time}],
        ),
        Node(
            package="autonomous_navigation",
            executable="local_planner",
            name="local_planner",
            output="screen",
            parameters=[{"real": False, "visualize": False, "use_sim_time": use_sim_time}],
        ),
        Node(
            package="autonomous_navigation",
            executable="obstacle_guard",
            name="obstacle_guard",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="autonomous_navigation",
            executable="controller",
            name="controller",
            output="screen",
            parameters=[{"real": False, "use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("goal_north", default_value="6.0"),
            DeclareLaunchArgument("goal_west", default_value="0.0"),
            DeclareLaunchArgument("obstacle_x", default_value="2.5"),
            DeclareLaunchArgument("obstacle_y", default_value="0.0"),
            gazebo,
            TimerAction(period=2.0, actions=[spawn_robot]),
            TimerAction(period=3.0, actions=[spawn_obstacle]),
            TimerAction(period=4.0, actions=autonomy_nodes),
        ]
    )
