from typing import List
from os import listdir, path
from toml import load

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

composition_ns = "rover"


def generate_launch_description():
    # Declare launch argument
    enable_ccb = LaunchConfiguration("enable_ccb")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_ccb",
                default_value="false",
                description="Enable CCB serial hardware nodes (cmr_read)",
            ),
            Node(
                package="cmr_fabric",
                executable="fault_handler",
                namespace=composition_ns,
            ),
            Node(
                package="cmr_fabric",
                executable="lifecycle_manager",
                namespace=composition_ns,
            ),
            *fabric_composition(
                "/home/cmr/cmr/terra/src/cmr_rovernet/config",
                enable_ccb,
            ),
        ]
    )


def fabric_composition(conf_dir: str, enable_ccb) -> List[Node]:
    return [
        fabric_node(path.join(conf_dir, x), enable_ccb)
        for x in listdir(conf_dir)
    ]


def fabric_node(conf_path: str, enable_ccb) -> Node:
    result = load(conf_path)
    pkg = result["package"]
    executable = result["executable"]
    name = result["name"]

    # Gate the hardware serial reader
    if name == "cmr_read":
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
            condition=IfCondition(enable_ccb),
        )

    # Default: always launch
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
    )
