"""Launch file for YOLOv8 object detection node."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.25',
        description='YOLO confidence threshold'
    )

    max_det_arg = DeclareLaunchArgument(
        'max_det',
        default_value='100',
        description='Maximum number of detections per frame'
    )

    model_file_arg = DeclareLaunchArgument(
        'model_file',
        default_value='urc_objects_v7.pt',
        description='YOLO model filename in config directory'
    )

    # Object detection node
    object_detection_node = Node(
        package='cmr_cams',
        executable='object_detection_node',
        name='yolov8_detection_node',
        output='screen',
        parameters=[{
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'max_det': LaunchConfiguration('max_det'),
            'model_file': LaunchConfiguration('model_file'),
        }],
    )

    return LaunchDescription([
        conf_threshold_arg,
        max_det_arg,
        model_file_arg,
        object_detection_node,
    ])
