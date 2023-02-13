import os
from ament_index_python.packages import get_package_share_directory 
from typing import List
from os import listdir, path
from toml import load
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource


composition_ns = "rover"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cmr_fabric",
                executable="fault_handler",
                namespace=composition_ns,
                # arguments=["--ros-args", "--log-level", "debug"],
            ),
            Node(
                package="cmr_fabric",
                executable="lifecycle_manager",
                namespace=composition_ns,
            ),
            fabric_node("/cmr/terra/src/cmr_bs/config/drivescontroller.toml"), 
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('cmr_control'), "launch", 'demo_drives.launch.py')
                    )
            )

        ]
    )




def fabric_node(conf_path: str) -> Node:
    result = load(conf_path)
    pkg = result["package"]
    executable = result["executable"]
    name = result["name"]

    return Node(
        package=pkg,
        executable=executable,
        name=name,
        exec_name=name,
        parameters=[
            {
                "config_path": conf_path,
                "composition_ns": composition_ns,
            }
        ],
        # arguments=["--ros-args", "--log-level", "debug"],
    )
