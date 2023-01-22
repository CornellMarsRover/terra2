from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions.if_condition import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    declare_rviz_config = DeclareLaunchArgument(name = 'use_rviz', 
                    default_value='True', 
                    description='Whether to launch RViz')
    declare_astro_motor_mode = DeclareLaunchArgument(name = 'astro_motor_mode',
                    default_value='"position"',
                    description='Either position or velocity, depending on the mode of the astro motor')
    use_rviz = LaunchConfiguration('use_rviz')
    astro_motor_mode = LaunchConfiguration('astro_motor_mode')


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("cmr_drives_description"),
                    "urdf",
                    "drives.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("cmr_control"), "config", "drives_controllers.yaml"]
    )

    astro_broadcasters = PathJoinSubstitution(
        [FindPackageShare("cmr_control"), "config", "astro_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("cmr_drives_description"), "config", "drives.rviz"]
    )

    astro_motor_controllers = PathJoinSubstitution(
        [FindPackageShare("cmr_control"), "config", "astro_motor_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, astro_broadcasters, astro_motor_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/drives_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    
    astro_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "astro_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drives_controller", "-c", "/controller_manager"],
    )

    astro_sensor_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["astro_controller", "-c", "/controller_manager"]
    )

    astro_motor_pos_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["astro_motor_pos_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(
            [astro_motor_mode, " == 'position'"]
        ))
    )

    astro_motor_vel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["astro_motor_vel_controller", "-c", "/controller_manager"],
        condition=IfCondition(PythonExpression(
            [astro_motor_mode, " == 'velocity'"]
        ))
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(use_rviz),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    # Delay start of astro_controller after `astro_sensor_broadcaster`
    delay_astro_controller_spawner_after_astro_sensor_broadcast_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=astro_sensor_broadcaster_spawner,
                on_exit=[astro_sensor_spawner],
            )
        )
    )

    # Delay start of astro_motor_pos_controller after `joint_state_broadcaster`
    delay_astro_motor_pos_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[astro_motor_pos_controller_spawner],
            ),
            condition=IfCondition(PythonExpression(
                [astro_motor_mode, " == 'position'"]
            ))
        )
    )

        # Delay start of astro_motor_vel_controller after `joint_state_broadcaster`
    delay_astro_motor_vel_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[astro_motor_vel_controller_spawner],
            ),
            condition=IfCondition(PythonExpression(
                [astro_motor_mode, " == 'velocity'"]
            ))
        )
    )

    nodes = [
        declare_rviz_config,
        declare_astro_motor_mode,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        astro_sensor_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_astro_controller_spawner_after_astro_sensor_broadcast_spawner,
        delay_astro_motor_pos_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_astro_motor_vel_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
