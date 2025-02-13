import launch
import launch_ros.actions


'''
        # Start localization_sim after state_machine starts
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='ekf',
                    name='ekf_node',
                    output='screen',
                    parameters=[{}]
                ),
            ],
        ),

'''

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

        # Start costmap after localization starts
        launch.actions.TimerAction(
            period=8.0,
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

        # Start planner after costmap_sim starts
        launch.actions.TimerAction(
            period=9.0,
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
