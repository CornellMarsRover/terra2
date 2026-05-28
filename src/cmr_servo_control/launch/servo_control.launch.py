from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmr_servo_control',
            executable='arm_servo_control_node',
            output='screen',
            parameters=[
                {'can_port': '/dev/serial/by-id/usb-mjbots_fdcanusb_062C45D7-if00'},
                {'baud': 115200},
                {'servo_can_id': 25},
                {'command_topic': '/arm_controller/cmd_buttons'},
            ],
        ),
    ])
