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

        # Start drives node
        launch_ros.actions.Node(
            package='cmr_controls',
            executable='swerve_controller_node',
            name='swerve_controller_node',
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
                    executable='ukf',
                    name='ukf',
                    output='screen',
                    parameters=[{'real': True}]
                ),
            ],
        ),

        # Start costmap
        launch.actions.TimerAction(
            period=2.0,
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
                    executable='p2',
                    name='p2',
                    output='screen',
                    parameters=[{'visualize': True}]
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
