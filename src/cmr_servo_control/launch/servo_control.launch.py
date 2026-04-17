from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmr_servo_control',
            executable='servo_hw_node',
            output='screen',
            parameters=[
                {'can_port': '/dev/ttyTHS0'},
                {'baud': 115200},
                {'servo_can_id': 24},
            ],
        ),
        Node(
            package='cmr_servo_control',
            executable='keyboard_input_node',
            output='screen',
        ),
    ])
