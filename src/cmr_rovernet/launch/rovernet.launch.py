from typing import List
from os import listdir, path
from toml import load
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

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
            *fabric_composition("/home/cmr/cmr/terra/src/cmr_rovernet/config"),
        ]
    )


def fabric_composition(conf_dir: str) -> List[Node]:
    return [fabric_node(path.join(conf_dir, x)) for x in listdir(conf_dir)]


def fabric_node(conf_path: str) -> Node:
    result = load(conf_path)
    pkg = result["package"]
    executable = result["executable"]
    name = result["name"]

    #makes it so cmr_read does not launch
    #to make it launch try: ros2 launch cmr_rovernet rovernet.launch.py enable_ccb:=true
    from launch.conditions import IfCondition 
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
