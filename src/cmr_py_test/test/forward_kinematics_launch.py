from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from toml import load
from os import listdir, path
from ament_index_python.packages import get_package_share_directory
import subprocess


def gen_forward_kinematics_launch_list() -> list:
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cmr_arm_description"),
                    "urdf",
                    "arm.urdf.xacro",
                ]
            ),
        ]
    )
    urdf_content = subprocess.check_output(["xacro", 
                    path.join(get_package_share_directory("cmr_arm_description"), "urdf", "arm.urdf.xacro")]).decode("utf-8")

    robot_description = {"robot_description": urdf_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("cmr_control"), "config", "arm_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

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

    nodes = [
        robot_description_content
        # control_node,
        # robot_state_pub_node,
        # joint_state_broadcaster_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]
    return nodes