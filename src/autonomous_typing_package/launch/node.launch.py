from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_camera    = LaunchConfiguration('use_camera')
    camera_index  = LaunchConfiguration('camera_index')
    camera_device = LaunchConfiguration('camera_device')
    marker_size   = LaunchConfiguration('aruco_marker_size_m')

    # Static TF: where the camera sits relative to the end effector.
    # The camera moves WITH the arm, so this parent is end_effector (not base_link).
    # x/y/z: offset in metres from end_effector origin to camera lens.
    # rpy: rotation so that camera +Z points the same way the end effector approaches.
    # Start with all zeros (camera coincident with EE) and tweak if poses are off.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ee_to_camera_tf',
        arguments=[
            '0.0',  # x
            '0.0',  # y
            '0.0',  # z
            '0.0',  # roll
            '0.0',  # pitch
            '0.0',  # yaw
            'end_effector',
            'camera_frame',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_camera',          default_value='true'),
        DeclareLaunchArgument('camera_index',        default_value='0'),
        DeclareLaunchArgument('camera_device',       default_value=''),
        DeclareLaunchArgument('aruco_marker_size_m', default_value='0.03',
                              description='Physical side length of ArUco markers in metres'),
        camera_tf,
        Node(
            package='autonomous_typing_package',
            executable='autonomous_typing',
            name='autonomous_typing_publisher',
            output='screen',
            condition=IfCondition(use_camera),
            parameters=[{
                'camera_index':       camera_index,
                'camera_device':      camera_device,
                'aruco_marker_size_m': marker_size,
            }],
        ),
        Node(
            package='autonomous_typing_package',
            executable='arm_coordinator',
            name='arm_coordinator',
            output='screen',
        ),
    ])
