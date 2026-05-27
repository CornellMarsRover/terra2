"""Launch the base-station YOLO detection node (URC 2026 autonomy).

Runs on the BASE STATION LAPTOP. Subscribes to the ZED feed crossing the radio
link, runs urc_objects_v9.pt at a reduced sample rate, and publishes the single
best detection plus the latched STOP command back to the rover.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument('conf_threshold', default_value='0.50',
                              description='Confidence to latch STOP on the armed object'),
        DeclareLaunchArgument('display_conf_threshold', default_value='0.25',
                              description='Confidence to draw a box on the panel'),
        DeclareLaunchArgument('process_every_n_frames', default_value='5',
                              description='Sample 1 in every N ZED frames (~10 Hz feed)'),
        DeclareLaunchArgument('model_file', default_value='urc_objects_v9.pt',
                              description='YOLO model filename in cmr_cams/config'),
        DeclareLaunchArgument('image_topic', default_value='/zed/image_left',
                              description='ZED feed topic to run inference on'),
        DeclareLaunchArgument('use_compressed', default_value='false',
                              description='Set true if image_topic is a CompressedImage'),
    ]

    base_detection_node = Node(
        package='cmr_cams',
        executable='base_detection_node',
        name='base_detection_node',
        output='screen',
        parameters=[{
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'display_conf_threshold': LaunchConfiguration('display_conf_threshold'),
            'process_every_n_frames': LaunchConfiguration('process_every_n_frames'),
            'model_file': LaunchConfiguration('model_file'),
            'image_topic': LaunchConfiguration('image_topic'),
            'use_compressed': LaunchConfiguration('use_compressed'),
        }],
    )

    return LaunchDescription(args + [base_detection_node])
