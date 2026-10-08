"""MoveIt Servo launch for the physical arm.

Unlike servo_example.launch.py, this launch does not start FakeSystem or any
ros2_control controllers.  Armnet is the hardware driver and is the sole
publisher of the physical arm's joint states.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, file_path), "r") as config_file:
        return yaml.safe_load(config_file)


def generate_launch_description():
    enable_rviz = LaunchConfiguration("enable_rviz")
    moveit_config = (
        MoveItConfigsBuilder("Arm", package_name="cmr_arm_simulator_moveit_config")
        .robot_description(file_path="config/Arm.urdf.xacro")
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": load_yaml(
            "moveit_servo", "config/panda_simulated_config.yaml"
        )
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    support_nodes = ComposableNodeContainer(
        name="moveit_servo_hardware_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "base_link", "frame_id": "world"}],
            ),
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("moveit_servo"),
                "config",
                "demo_rviz_config.rviz",
            ),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(enable_rviz),
        output="log",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="false",
                description="Start RViz with the physical arm.",
            ),
            support_nodes,
            servo_node,
            rviz_node,
        ]
    )
