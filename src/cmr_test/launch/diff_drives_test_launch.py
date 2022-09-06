from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_ign_gazebo")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "ign_gazebo.launch.py")
        ),
        launch_arguments={
            "ign_args": "-r /cmr/terra/src/cmr_test/config/visualize_lidar.sdf"
        }.items(),
    )
    return LaunchDescription(
        [
            gz_sim,
            Node(
                package="ros_ign_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                    "/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                ],
                remappings=[
                    ("/model/vehicle_blue/cmd_vel", "/test/cmd_vel"),
                    ("/model/vehicle_blue/odometry", "/test/odometry"),
                ],
                output="screen",
            ),
            Node(
                executable="/cmr/terra/install/cmr_test/lib/cmr_test/test_tf2",
                name="position_controller",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                executable="/cmr/terra/install/cmr_test/lib/cmr_test/test_controller",
                name="position_client",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
