from os import path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_aruco",
                executable="aruco_node",
                parameters=[
                    {
                        "image_topic": "/zed/zed_node/left/image_rect_color",
                        "camera_info_topic": "/zed/zed_node/left/camera_info",
                        "camera_frame": "zed_left_camera_frame",
                        "aruco_dictionary_id": "DICT_4X4_50",
                        "marker_size": 0.2,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "zed_left_camera_frame",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "-1.58",
                    "0",
                    "-1.58",
                    "base_link",
                    "markers",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "world"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
                output="screen",
            ),
        ]
    )
