from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "drives", "-file", robot_file, "-x", "0.0", "-y", "0.0", "-z", "0.05"],
        output="screen",
    )

    obstacle_spawners = []
    for spec in obstacle_specs:
        obstacle_spawners.append(
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    spec["entity"],
                    "-file",
                    obstacle_file,
                    "-x",
                    str(spec["north"]),
                    "-y",
                    str(spec["west"]),
                    "-z",
                    "0.5",
                ],
                output="screen",
            )
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
            executable="sim_vision_obstacle_detector",
            name="sim_vision_obstacle_detector",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "mode": "synthetic",
                    "image_topic": "/camera1/image_raw",
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
            parameters=[{"real": False, "visualize": False, "use_sim_time": True}],
        ),
        Node(
            package="autonomous_navigation",
            executable="obstacle_guard",
            name="obstacle_guard",
            output="screen",
            parameters=[{"use_sim_time": True}],
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
            TimerAction(period=2.0, actions=[spawn_robot]),
            TimerAction(period=3.0, actions=obstacle_spawners),
            TimerAction(period=4.0, actions=autonomy_nodes),
        ]
    )
