import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Start imu node first
        launch_ros.actions.Node(
            package='cmr_imu',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{}]
        ),

        # Start zed node
        launch_ros.actions.Node(
            package='cmr_zed',
            executable='zed_autonomy',
            name='zed_autonomy',
            output='screen',
            parameters=[{}]
        ),

        # Start RTK GPS rover node
        launch_ros.actions.Node(
            package='cmr_rtkgps',
            executable='gps_rover',
            name='gps_rover',
            output='screen',
            parameters=[{}]
        ),

        # Start localization node
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='rtk_localization',
                    name='rtk_localization',
                    output='screen',
                    parameters=[{}]
                ),
            ],
        ),

        # Start costmap
        launch.actions.TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='costmap_sim',
                    name='costmap_sim',
                    output='screen',
                    parameters=[{'real': True, 'use_sim_time': False}]
                ),
            ],
        ),

        # Start planner
        launch.actions.TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='planner',
                    name='planner',
                    output='screen',
                    parameters=[{'visualize': True}]
                ),
            ],
        ),
    ])
