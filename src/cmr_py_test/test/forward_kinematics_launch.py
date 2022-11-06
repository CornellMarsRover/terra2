from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from toml import load
from os import listdir, path


def gen_forward_kinematics_launch_list() -> list:
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("cmr_arm_description"),
    #                 "urdf",
    #                 "arm.urdf.xacro",
    #             ]
    #         ),
    #     ]
    # )

    # robot_description = {"robot_description": robot_description_content}

    # robot_controllers = PathJoinSubstitution(
    #     [FindPackageShare("cmr_control"), "config", "arm_controllers.yaml"]
    # )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, robot_controllers],
    #     output="both",
    # )

    # robot_state_pub_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    #     remappings=[
    #         ("/arm_controller/cmd_vel_unstamped", "/cmd_vel"),
    #     ],
    # )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["arm_controller", "-c", "/controller_manager"],
    # )

    # # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
    #     RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=joint_state_broadcaster_spawner,
    #             on_exit=[robot_controller_spawner],
    #         )
    #     )
    # )

    # nodes = [
    #     control_node,
    #     robot_state_pub_node,
    #     joint_state_broadcaster_spawner,
    #     delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    # ]
    nodes = fabric_composition("/cmr/terra/src/cmr_demo/config")

    return nodes

def fabric_composition(conf_dir: str) -> list:
    return [fabric_node(path.join(conf_dir, x)) for x in listdir(conf_dir)]


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
                "composition_ns": "rover",
            }
        ],
    )