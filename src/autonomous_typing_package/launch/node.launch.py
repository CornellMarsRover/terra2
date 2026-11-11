from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("text", default_value="CMR"),
        DeclareLaunchArgument("key_offset_scale", default_value="3.0"),
        DeclareLaunchArgument("press_depth_m", default_value="0.0762"),
        DeclareLaunchArgument("start_delay_s", default_value="2.0"),
        DeclareLaunchArgument("enable_motion", default_value="true"),
        Node(
            package="autonomous_typing_package",
            executable="arm_coordinator",
            name="arm_coordinator",
            output="screen",
            parameters=[{
                "text": LaunchConfiguration("text"),
                "key_offset_scale": LaunchConfiguration("key_offset_scale"),
                "press_depth_m": LaunchConfiguration("press_depth_m"),
                "start_delay_s": LaunchConfiguration("start_delay_s"),
                "enable_motion": LaunchConfiguration("enable_motion"),
            }],
        ),
    ])
