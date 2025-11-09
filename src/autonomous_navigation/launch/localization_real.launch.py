import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
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
        # zenchang replaced this
        launch.actions.TimerAction(
        period=1.0,
        actions=[
            launch_ros.actions.Node(
                package='autonomous_navigation',
                executable='rtk_localization',
                name='rtk_localization',
                output='screen',
                parameters=[{'real': True}],
                remappings=[
                    ('/gps/fix', '/rtk/navsatfix_data'),
                    ('/imu/data', '/imu'),
                ],
            ),
        ],
    ),


        # Start costmap
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    # executable='costmap_sim',
                    # name='costmap_sim',
                    executable='costmap',
                    name='costmap',
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
                    executable='local_planner',
                    name='local_planner',
                    output='screen',
                    parameters=[{'visualize': True, 'real': True}]
                ),
            ],
        ),

        # Start controller node after long wait to ensure nothing is going wrong
        launch.actions.TimerAction(
            period=5.0,# it was 30 seconds before
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
