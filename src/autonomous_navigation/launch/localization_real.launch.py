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

        # Start HWT905-RS232 IMU node as the canonical /imu publisher for autonomy.
        launch_ros.actions.Node(
            package='cmr_imu',
            executable='imu_node',
            name='hwt905_imu',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                'baud': 9600,
                'frame_topic': '/imu',
                'yaw_offset_degrees': 0.0,
                'auto_zero_yaw': False,
            }]
        ),

        # Start drives node
        launch_ros.actions.Node(
            package='cmr_controls',
            executable='new_swerve_node',
            name='new_swerve_node',
            output='screen',
            parameters=[{'drive_rps': 22.0}]
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
                parameters=[{
                    'real': True,
                    # HWT905-RS232 anglez on this rover comes out
                    # CCW-positive (0=N, +90=W, -90=E), so do NOT invert.
                    # Inverting was the cause of the E/W swap and reversed
                    # rotation seen on the live telemetry.
                    'invert_imu_yaw': False,
                    'magnetic_declination_degrees': -11.5,
                }],
                remappings=[
                    ('/gps/fix', '/rtk/navsatfix_data'),
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
        launch.actions.TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    package='autonomous_navigation',
                    executable='global_planner',
                    name='global_planner',
                    output='screen',
                    parameters=[{'real': True}]
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
                    parameters=[{'real': True}]
                ),
            ],
        ),

    ])
