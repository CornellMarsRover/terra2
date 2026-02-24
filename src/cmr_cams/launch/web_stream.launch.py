"""Launch file for the MJPEG web stream node."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='8000',
        description='HTTP port for MJPEG streams'
    )

    quality_arg = DeclareLaunchArgument(
        'quality', default_value='70',
        description='JPEG quality (1-100)'
    )

    stream_node = Node(
        package='cmr_cams',
        executable='web_stream_node',
        name='web_stream_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'quality': LaunchConfiguration('quality'),
        }],
    )

    return LaunchDescription([
        port_arg,
        quality_arg,
        stream_node,
    ])
