import launch
import launch_ros.actions
from os import path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    drive_config = path.join(
        get_package_share_directory("cmr_rovernet"), "config", "drivesnet.toml"
    )
    return launch.LaunchDescription([

        # Use the same drive node as teleop. It arbitrates manual commands with
        # autonomy's /cmd_vel_drives input before shared swerve/Moteus handling.
        launch_ros.actions.Node(
            package='cmr_rovernet',
            executable='usama_control_testing_node',
            name='drivesnet',
            output='screen',
            parameters=[{
                'config_path': drive_config
            }],
        ),

        # Start state machine node
        launch_ros.actions.Node(
            package='autonomous_navigation',
            executable='led_node',
            name='led_node',
            output='screen',
            parameters=[{}]
        ),

        # Start state machine node
        launch_ros.actions.Node(
            package='autonomous_navigation',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[{}]
        ),

        # Start localization node
        launch.actions.TimerAction(
            period=1.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='new_kalman',
                    name='new_kalman',
                    output='screen',
                    parameters=[{}]
                ),
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='costmap',
                    name='costmap',
                    output='screen',
                    parameters=[{'real': True, 'use_sim_time': False}]
                ),
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='local_planner',
                    name='local_planner',
                    output='screen',
                    parameters=[{'visualize': True}]
                ),
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='object_detection',
                    name='object_detection',
                    output='screen',
                    parameters=[{}]
                ),
            ],
        ),



        # Start controller node after long wait to ensure nothing is going wrong
        launch.actions.TimerAction(
            period=30.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='controller',
                    name='controller',
                    output='screen',
                    parameters=[{'visualize': True}]
                ),
            ],
        ),

    ])
