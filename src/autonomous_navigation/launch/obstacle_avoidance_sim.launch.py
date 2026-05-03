from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    autonomy_share = get_package_share_directory("autonomous_navigation")
    gazebo_share = get_package_share_directory("gazebo_ros")
    workspace_root = Path(autonomy_share).resolve().parents[3]

    robot_file = str(workspace_root / "drives.urdf")
    obstacle_file = f"{autonomy_share}/models/obstacle_box.sdf"
    world_file = f"{autonomy_share}/worlds/obstacle_avoidance_demo.world"

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    goal_north = LaunchConfiguration("goal_north")
    goal_west = LaunchConfiguration("goal_west")
    obstacle_x = LaunchConfiguration("obstacle_x")
    obstacle_y = LaunchConfiguration("obstacle_y")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gazebo_share}/launch/gazebo.launch.py"),
        launch_arguments={"gui": gui, "world": world_file}.items(),
    )

    spawn_entities = Node(
        package="autonomous_navigation",
        executable="spawn_demo_entities",
        name="spawn_demo_entities",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_entity": "drives",
                "robot_file": robot_file,
                "robot_x": 0.0,
                "robot_y": 0.0,
                "robot_z": 0.05,
                "obstacle_file": obstacle_file,
                "obstacle_entities": ["obstacle_box"],
                "obstacle_xs": [obstacle_x],
                "obstacle_ys": [obstacle_y],
                "obstacle_zs": [0.5],
            }
        ],
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
            executable="sim_obstacle_pointcloud",
            name="sim_obstacle_pointcloud",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "obstacle_norths": [obstacle_x],
                    "obstacle_wests": [obstacle_y],
                    "obstacle_size_norths": [0.8],
                    "obstacle_size_wests": [0.8],
                }
            ],
        ),
        Node(
            package="autonomous_navigation",
            executable="costmap",
            name="costmap",
            output="screen",
            parameters=[{"real": False, "use_sim_time": use_sim_time}],
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
            parameters=[
                {
                    "real": False,
                    "visualize": False,
                    "use_sim_time": use_sim_time,
                    "replan_confirmation_cycles": 1,
                    "validation_horizon_segments": 1,
                    "replan_cooldown_s": 1.5,
                    "goal_tolerance": 0.7,
                }
            ],
        ),
        Node(
            package="autonomous_navigation",
            executable="obstacle_guard",
            name="obstacle_guard",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "lookahead_distance": 1.0,
                    "hard_stop_distance": 0.35,
                    "corridor_half_width": 0.45,
                }
            ],
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
            spawn_entities,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entities,
                    on_exit=autonomy_nodes,
                )
            ),
        ]
    )
