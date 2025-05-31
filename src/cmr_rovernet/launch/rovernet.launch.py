from typing import List
from os import path
from glob import glob
from toml import load

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

composition_ns = "rover"


def generate_launch_description() -> LaunchDescription:
    # Resolve the package share dir at runtime instead of hardcoding an absolute path
    conf_dir = path.join(get_package_share_directory("cmr_rovernet"), "config")

    nodes: List[Node] = [
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
        *fabric_composition(conf_dir),
    ]

    return LaunchDescription(nodes)


def fabric_composition(conf_dir: str) -> List[Node]:
    if not path.isdir(conf_dir):
        raise RuntimeError(
            f"Config directory not found: {conf_dir}. "
            "Make sure cmr_rovernet/config exists in your workspace."
        )

    # Only load .toml files, sorted for deterministic startup order
    files = sorted(glob(path.join(conf_dir, "*.toml")))
    if not files:
        raise RuntimeError(
            f"No .toml config files found in {conf_dir}. "
            "Add at least one config file."
        )

    return [fabric_node(f) for f in files]


def fabric_node(conf_path: str) -> Node:
    result = load(conf_path)
    try:
        pkg = result["package"]
        executable = result["executable"]
        name = result["name"]
    except KeyError as e:
        raise RuntimeError(f"Missing required key {e} in {conf_path}")

    return Node(
        package=pkg,
        executable=executable,
        name=name,
        exec_name=name,
        namespace=composition_ns,
        parameters=[
            {
                "config_path": conf_path,
                "composition_ns": composition_ns,
            }
        ],
        # arguments=["--ros-args", "--log-level", "debug"],
    )


# from typing import List
# from os import listdir, path
# from toml import load
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# composition_ns = "rover"


# def generate_launch_description():
#     return LaunchDescription(
#         [
#             Node(
#                 package="cmr_fabric",
#                 executable="fault_handler",
#                 namespace=composition_ns,
#                 # arguments=["--ros-args", "--log-level", "debug"],
#             ),
#             Node(
#                 package="cmr_fabric",
#                 executable="lifecycle_manager",
#                 namespace=composition_ns,
#             ),
#             *fabric_composition("/cmr/terra/src/cmr_rovernet/config"),
#         ]
#     )


# def fabric_composition(conf_dir: str) -> List[Node]:
#     return [fabric_node(path.join(conf_dir, x)) for x in listdir(conf_dir)]


# def fabric_node(conf_path: str) -> Node:
#     result = load(conf_path)
#     pkg = result["package"]
#     executable = result["executable"]
#     name = result["name"]

#     return Node(
#         package=pkg,
#         executable=executable,
#         name=name,
#         exec_name=name,
#         parameters=[
#             {
#                 "config_path": conf_path,
#                 "composition_ns": composition_ns,
#             }
#         ],
#         # arguments=["--ros-args", "--log-level", "debug"],
#     )
