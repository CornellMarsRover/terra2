"""Sim autonomy with the global planner in the loop.

Identical to sim_autonomy.launch.py except for two things:

  * the state machine runs with publish_local_target:=false, so it stops
    publishing the mission waypoint straight onto /autonomy/target/local and
    /autonomy/target/global becomes the single canonical mission route;
  * global_planner runs between them and owns /autonomy/target/local.

Kept as a separate file on purpose. The global planner has only been exercised
against the committed logic tests and the Gazebo harness, never outside on the
rover, so the default launch path is unchanged until it has been.
"""

import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cmr_rovernet',
            executable='drive_command_mux',
            name='drive_command_mux',
            parameters=[{'active_source': 'autonomy'}],
        ),
        launch_ros.actions.Node(
            package='autonomous_navigation',
            executable='state_machine',
            name='state_machine',
            output='screen',
            parameters=[{
                'real': False,
                'use_sim_time': True,
                'publish_local_target': False,
            }],
        ),
        launch_ros.actions.Node(
            package='autonomous_navigation',
            executable='object_detection',
            name='object_detection',
            output='screen',
            parameters=[{'real': False, 'use_sim_time': True}],
        ),
        launch.actions.TimerAction(
            period=1.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='localization_sim',
                    name='localization_sim',
                    output='screen',
                    parameters=[{'real': False, 'use_sim_time': True}],
                ),
            ],
        ),
        launch.actions.TimerAction(
            period=2.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='costmap',
                    name='costmap',
                    output='screen',
                    parameters=[{'real': False, 'use_sim_time': True}],
                ),
            ],
        ),
        launch.actions.TimerAction(
            period=3.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='global_planner',
                    name='global_planner',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'cell_size': 0.5,
                        'lookahead': 6.0,
                        'inflation_radius': 1,
                    }],
                ),
            ],
        ),
        launch.actions.TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='local_planner',
                    name='local_planner',
                    output='screen',
                    parameters=[{'visualize': True, 'real': False}],
                ),
            ],
        ),
        launch.actions.TimerAction(
            period=6.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='controller',
                    name='controller',
                    output='screen',
                    parameters=[{'real': False}],
                ),
            ],
        ),
    ])
