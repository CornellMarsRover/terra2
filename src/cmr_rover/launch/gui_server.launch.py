from os import path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource

composition_ns = "rover"


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch rosbridge for frontend access via socket on port 9090
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    path.join(
                        get_package_share_directory("foxglove_bridge"),
                        "foxglove_bridge_launch.xml",
                    )
                )
            ),
        ]
    )
