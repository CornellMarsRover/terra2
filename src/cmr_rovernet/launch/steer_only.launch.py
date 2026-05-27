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
                        "composition_ns": "rover_steer_only",
                    }
                ],
            ),
            Node(
                package="cmr_rovernet",
                executable="steer_only_control_node",
                name="steer_only_control",
                parameters=[
                    {
                        "composition_ns": "rover_steer_only",
                        "can_port": "/dev/ttyACM0",
                        "timeout_s": 0.5,
                        "steer_velocity_limit": 1.5,
                        "steer_max_torque": 0.2,
                        "watchdog_timeout_s": 0.5,
                        "controller_deadzone": 0.1,
                        "command_timeout_s": 0.5,
                        "refresh_rate_hz": 10.0,
                        "capture_steer_zero_on_start": True,
                    }
                ],
            ),
        ]
    )
