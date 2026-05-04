from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    autonomy_share = get_package_share_directory("autonomous_navigation")
    gazebo_share = get_package_share_directory("gazebo_ros")
    workspace_root = Path(autonomy_share).resolve().parents[3]

    robot_file = str(workspace_root / "drives.urdf")
    obstacle_file = f"{autonomy_share}/models/obstacle_box.sdf"
    world_file = f"{autonomy_share}/worlds/obstacle_avoidance_demo.world"

    obstacle_specs = [
        {"entity": "obstacle_center", "north": 2.5, "west": 0.0, "size_north": 0.8, "size_west": 0.8},
        {"entity": "obstacle_right", "north": 3.6, "west": -1.2, "size_north": 0.8, "size_west": 0.8},
        {"entity": "obstacle_left", "north": 4.8, "west": 1.8, "size_north": 0.8, "size_west": 0.8},
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gazebo_share}/launch/gazebo.launch.py"),
        launch_arguments={"gui": "false", "world": world_file}.items(),
    )

    spawn_entities = Node(
        package="autonomous_navigation",
        executable="spawn_demo_entities",
        name="spawn_demo_entities",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_entity": "drives",
                "robot_file": robot_file,
                "robot_x": 0.0,
                "robot_y": 0.0,
                "robot_z": 0.05,
                "obstacle_file": obstacle_file,
                "obstacle_entities": [spec["entity"] for spec in obstacle_specs],
                "obstacle_xs": [spec["north"] for spec in obstacle_specs],
                "obstacle_ys": [spec["west"] for spec in obstacle_specs],
                "obstacle_zs": [0.5 for _ in obstacle_specs],
            }
        ],
    )

    autonomy_nodes = [
        Node(
            package="autonomous_navigation",
            executable="sim_drive_bridge",
            name="sim_drive_bridge",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="autonomous_navigation",
            executable="odom_to_autonomy_pose",
            name="odom_to_autonomy_pose",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="autonomous_navigation",
            executable="sim_goal_publisher",
            name="sim_goal_publisher",
            output="screen",
            parameters=[{"use_sim_time": True, "goal_north": 6.0, "goal_west": 0.0}],
        ),
        Node(
            package="autonomous_navigation",
            executable="sim_obstacle_pointcloud",
            name="sim_obstacle_pointcloud",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "fov_deg": 40.0,
                    "synthetic_occlusion_overlap_deg": 10.0,
                    "obstacle_norths": [spec["north"] for spec in obstacle_specs],
                    "obstacle_wests": [spec["west"] for spec in obstacle_specs],
                    "obstacle_size_norths": [spec["size_north"] for spec in obstacle_specs],
                    "obstacle_size_wests": [spec["size_west"] for spec in obstacle_specs],
                }
            ],
        ),
        Node(
            package="autonomous_navigation",
            executable="costmap",
            name="costmap",
            output="screen",
            parameters=[{"real": False, "use_sim_time": True, "obstacle_persistence_s": 0.8}],
        ),
        Node(
            package="autonomous_navigation",
            executable="global_planner",
            name="global_planner",
            output="screen",
            parameters=[{"real": False, "use_sim_time": True}],
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
                    "use_sim_time": True,
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
                    "use_sim_time": True,
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
            parameters=[{"real": False, "use_sim_time": True}],
        ),
    ]

    return LaunchDescription(
        [
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
