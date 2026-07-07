import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Start state_machine & object detection nodes first
        launch_ros.actions.Node(
            package='autonomous_navigation',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[{
                'real': False,
                'use_sim_time': True,
                # state_machine uses this only to capture the rover's first
                # GPS fix as the local-frame origin.
                'gps_topic': '/navsatfix',
            }]
        ),
        launch_ros.actions.Node(
            package='autonomous_navigation',
            executable='object_detection',
            name='object_detection',
            output='screen',
            parameters=[{'real': False, 'use_sim_time': True}]
        ),

        # Start localization after state_machine starts
        launch.actions.TimerAction(
            period=1.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='localization_sim',
                    name='localization_sim',
                    output='screen',
                    parameters=[{'real': False, 'use_sim_time': True}]
                ),
            ],
        ),

        # Start costmap_sim after localization_sim starts
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='costmap',
                    name='costmap',
                    output='screen',
                    parameters=[{'real': False, 'use_sim_time': True}]
                ),
            ],
        ),

        # Start planners after costmap starts
        launch.actions.TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='local_planner',
                    name='local_planner',
                    output='screen',
                    parameters=[{'visualize': True, 'real': False, 'use_sim_time': True}]
                ),
            ],
        ),
        launch.actions.TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='global_planner',
                    name='global_planner',
                    output='screen',
                    parameters=[{'real': False, 'use_sim_time': True}]
                ),
            ],
        ),

        # Start controller after planner starts
        launch.actions.TimerAction(
            period=6.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='controller',
                    name='controller',
                    output='screen',
                    parameters=[{'real': False, 'use_sim_time': True}]
                ),
            ],
        ),
    ])
