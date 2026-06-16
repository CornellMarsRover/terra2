"""Full autonomous navigation + detection run (JETSON side).

Boots the complete nav stack (via localization_real.launch.py: ZED, IMU,
drives, state machine, RTK localization, costmap, planners, controller) plus
the on-Jetson telemetry GUI that draws the detection overlay.

The base-station YOLO detection node is launched SEPARATELY on the base laptop
(cmr_cams base_detection.launch.py) -- the Launch Hub "FULL AUTON RUN" button
starts both with one click. RTK corrections (base) and GPS rover (Jetson) are
prerequisites started beforehand, as in the normal workflow.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('autonomous_navigation')

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'localization_real.launch.py'))
    )

    # On-Jetson telemetry GUI (viewed from the base laptop). Renders the ZED
    # feed plus the detection overlay + "OBJECT DETECTED" banner.
    telemetry_gui = Node(
        package='autonomous_navigation',
        executable='live_telemetry_tool',
        name='realtime_robot_plotter',
        output='screen',
        parameters=[{}],
    )

    return LaunchDescription([nav_stack, telemetry_gui])
