from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cmr_fabric",
                executable="fault_handler",
            ),
            Node(
                package="cmr_fabric",
                executable="lifecycle_manager",
            ),
            Node(
                package="cmr_demo",
                executable="demo_node",
                parameters=[
                    {"config_path": "/cmr/terra/src/cmr_demo/config/demo.toml"}
                ],
            ),
        ]
    )
