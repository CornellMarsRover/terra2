import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Start state_machine node first
        launch_ros.actions.Node(
            package='autonomous_navigation',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[{'real': False, 'use_sim_time': True}]
        ),

        # Start localization_sim after state_machine starts
        launch.actions.TimerAction(
            period=1.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='localization_sim',
                    name='localization_sim',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                ),
            ],
        ),

        # Start costmap_sim after localization_sim starts
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='costmap_sim',
                    name='costmap_sim',
                    output='screen',
                    parameters=[{'real': False, 'use_sim_time': True}]
                ),
            ],
        ),

        # Start planner after costmap_sim starts
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

        # Start controller after planner starts
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='controller',
                    name='controller',
                    output='screen',
                    parameters=[{'real': False}]
                ),
            ],
        ),
    ])
