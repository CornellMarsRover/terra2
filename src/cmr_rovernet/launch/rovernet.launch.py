from typing import List
from os import listdir, path
from toml import load
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

composition_ns = "rover"


def generate_launch_description():
    enable_arm = LaunchConfiguration("enable_arm")
    enable_drives = LaunchConfiguration("enable_drives")
    enable_debug = LaunchConfiguration("enable_debug")
    enable_controller = LaunchConfiguration("enable_controller")

    launch_switches = {
        "armnet": IfCondition(enable_arm),
        "drivesnet": IfCondition(enable_drives),
        "debug": IfCondition(enable_debug),
        "controller_connect": IfCondition(enable_controller),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_arm",
                default_value="true",
                description="Launch the armnet hardware bridge",
            ),
            DeclareLaunchArgument(
                "enable_drives",
                default_value="true",
                description="Launch the drives hardware bridge",
            ),
            DeclareLaunchArgument(
                "enable_debug",
                default_value="true",
                description="Launch rovernet debug node",
            ),
            DeclareLaunchArgument(
                "enable_controller",
                default_value="true",
                description="Launch controller UDP bridge",
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
                path.join(get_package_share_directory("cmr_rovernet"), "config"),
                launch_switches,
            ),
        ]
    )


def fabric_composition(conf_dir: str, launch_switches) -> List[Node]:
    return [
        fabric_node(path.join(conf_dir, x), launch_switches)
        for x in listdir(conf_dir)
        if x.endswith(".toml") and path.isfile(path.join(conf_dir, x))
    ]


def fabric_node(conf_path: str, launch_switches) -> Node:
    result = load(conf_path)
    pkg = result["package"]
    executable = result["executable"]
    name = result["name"]
    kwargs = {}
    if name in launch_switches:
        kwargs["condition"] = launch_switches[name]

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
        **kwargs,
    )
