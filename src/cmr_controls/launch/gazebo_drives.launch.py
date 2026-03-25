from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    with_remote = LaunchConfiguration("with_remote")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "with_remote",
                default_value="false",
                description="Also start the existing cmr_controller_remote UDP listener.",
            ),
            Node(
                package="cmr_controller_remote",
                executable="connect_node",
                name="connect_node",
                output="screen",
                condition=IfCondition(with_remote),
            ),
            Node(
                package="cmr_controls",
                executable="gazebo_drive_bridge",
                name="gazebo_drive_bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
