from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller_config = "/home/cmr/cmr/terra2/src/cmr_rovernet/config/connect.toml"

    return LaunchDescription(
        [
            Node(
                package="cmr_controller_remote",
                executable="connect_node",
                name="controller_connect",
                parameters=[
                    {
                        "config_path": controller_config,
                        "composition_ns": "rover_drive_only",
                    }
                ],
            ),
            Node(
                package="cmr_rovernet",
                executable="drive_only_control_node",
                name="drive_only_control",
                parameters=[
                    {
                        "composition_ns": "rover_drive_only",
                        "can_port": "/dev/ttyACM0",
                        "timeout_s": 0.5,
                        "drive_rps": 1.0,
                        "half_speed_multiplier": 0.5,
                        "double_speed_multiplier": 2.0,
                        "triple_speed_multiplier": 3.0,
                        "trigger_pressed_threshold": 32,
                        "drive_max_torque": 0.5,
                        "drive_acceleration_limit": 5.0,
                        "watchdog_timeout_s": 0.5,
                        "controller_deadzone": 0.1,
                        "command_timeout_s": 0.5,
                        "refresh_rate_hz": 10.0,
                    }
                ],
            ),
        ]
    )
