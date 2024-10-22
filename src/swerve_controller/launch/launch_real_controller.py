import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("swerve_controller"),
        "config",
        "robot_geometry.yaml",
    )

    node_commander = Node(
        package="swerve_controller",
        name="controller_real",
        executable="real_rover_controller",
        parameters=[config],
    )

    ld.add_action(node_commander)

    return ld
