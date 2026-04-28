"""
Launch file for aruco detection with a simple OpenCV camera.
No ZED SDK required -- uses cv2.VideoCapture for the camera feed.

Usage:
    ros2 launch cmr_arm_arucos aruco_detection.launch.py
    ros2 launch cmr_arm_arucos aruco_detection.launch.py camera_index:=1 fps:=15
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_index_arg = DeclareLaunchArgument(
        'camera_index', default_value='0',
        description='Camera device index (0 for default USB camera)')

    fps_arg = DeclareLaunchArgument(
        'fps', default_value='10.0',
        description='Camera capture FPS')

    camera_node = Node(
        package='cmr_arm_arucos',
        executable='simple_camera_node',
        name='simple_camera_node',
        output='screen',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'fps': LaunchConfiguration('fps'),
            'image_topic': '/zed/image_left',
        }],
    )

    aruco_node = Node(
        package='cmr_arm_arucos',
        executable='arm_aruco_detection',
        name='aruco_detection_node',
        output='screen',
    )

    return LaunchDescription([
        camera_index_arg,
        fps_arg,
        camera_node,
        aruco_node,
    ])
