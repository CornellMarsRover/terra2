import rclpy
from rclpy.node import Node
from cmr_msgs.msg import ControllerReading
from cmr_msgs.msg import MiniArmDegree
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import time
import serial
import toml
from cmr_rovernet.rovernet_utils import *

MAX_TORQUE = 0.7
MAX_ACCEL = 5.0
MINI_ARM_MAX_TORQUE = 0.7
MINI_ARM_VELOCITY_LIMIT = 10.0
MINI_ARM_ACCEL_LIMIT = 10.0
MINI_ARM_SOFT_GEAR_RATIO = [
    0.25,
    0.3,
    0.25,
    0.125,
    0.125,
    0.125,
]
MINI_ARM_DEADBAND = 0.002
JOINT_MOTOR_IDS = {
    "base_joint": 9,
    "shoulder_joint": 10,
    "elbow_joint": 11,
    "wrist_rotate": 12,
    "wrist_twist": 13,
    "wrist_rotate_two": 14,
}

logger = logging.getLogger("ArmLogger")
logger.setLevel(logging.INFO)

class JSInputSubscriber(Node):
    """
    This node subscribes to the /js_input topic output by the 
    armcontroller node. It will then convert the output to a pre-defined 40-byte 
    format and send the output to the CCB via UART. 
    """
    
    def __init__(self):
        super().__init__('js_input_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/main_arm_controller/joint_trajectory',
            self.listener_callback,
            10)
        self.miniarm_subscription = self.create_subscription(
            MiniArmDegree,
            '/mini_arm_controller/cmd_pos',
            self.listener_callback_mini,
            10)
        self.subscription  # prevent unused variable warning
        self.current_speed = 0
        self.current_speed_angular = 0
        self.last_time = time.time()
        self.prev_mini_arm_positions = None
        self.target_mini_arm_positions = [0.0] * len(JOINT_MOTOR_IDS)
        # self.port = "/dev/ttyTHS0"
        self.baud_rate = 115200
        # self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
        # self.serial_port = None
        self.logger = self.get_logger()
        self.declare_parameter("config_path", "")
        node_config = self._load_node_config()
        self.arm_can_port = str(node_config.get("can_port", "/dev/ttyACM1"))
        self.publish_hardware_joint_states = bool(
            node_config.get("publish_hardware_joint_states", False)
        )
        self.use_legacy_ik_command_mapping = bool(
            node_config.get("use_legacy_ik_command_mapping", True)
        )
        self.arm_transport, self.controllers = make_moteus_transport_and_controllers_sync(
            self.arm_can_port,
            JOINT_MOTOR_IDS,
        )
        self.joint_state_pub = None
        self.joint_state_timer = None
        if self.publish_hardware_joint_states:
            self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
            self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        self.get_logger().info(f"Arm moteus fdcanusb transport: {self.arm_can_port}")
        self.get_logger().info(
            f"Arm hardware joint-state publishing: {self.publish_hardware_joint_states}"
        )
        self.get_logger().info(
            f"Arm legacy IK command mapping: {self.use_legacy_ik_command_mapping}"
        )
        
        # Init constants given TOML file
        # arm_controller_table = parse_toml("TODO")
        # self.CONTROLLER_MAX_SPEED = arm_controller_table['node']['max_speed']

        # arm_net_table = parse_toml("armnet")
        # arm_net_node = arm_net_table['node']
        # self.MOTOR_MAX_SPEED = arm_net_node['motor_max_speed']
        # self.GRADUAL_INCREASE_RATE = arm_net_node['gradual_increase_rate']
        # self.BASE_DYNAMIC_RATIO = arm_net_node['base_dynamic_ratio']
        
    def _load_node_config(self):
        config_path = str(self.get_parameter("config_path").value)
        if not config_path:
            return {}

        try:
            data = toml.load(config_path)
        except Exception as exc:
            self.get_logger().warn(f"Failed to load config_path {config_path}: {exc!r}")
            return {}

        node_config = data.get("node", {})
        if not isinstance(node_config, dict):
            self.get_logger().warn(f"config_path {config_path} has no valid [node] table")
            return {}
        return node_config


    def convert_angle_to_custom_range(self, angle, max_range):
        """
        Converts an angle from the range [-pi, pi] to a custom range [max_range, 0],
        where -pi translates to max_range and pi translates to 0.
        
        Parameters:
        - angle (float): The angle in radians, must be in the range [-pi, pi].
        - max_range (float): The maximum value of the target range, corresponding to -pi.
        
        Returns:
        - float: The converted value in the range [max_range, 0].
        """
        # Normalize the input angle from [-pi, pi] to [1, 0]
        normalized_angle = (angle - 3.141592653589793) / (-2 * 3.141592653589793)
        
        # Scale the normalized angle to the custom range [max_range, 0]
        offset = max_range/2
        scaled_value = (normalized_angle * max_range) - offset
        
        return scaled_value
    

    def convert_radians_to_motor_rotation(self, vel, ratio):
        conversion = vel* 1/(2 * 3.141592653589793) * ratio
        return conversion

    
    def translate_to_electrical(self, positions, velocities):
        output_pos = [-self.convert_angle_to_custom_range(positions[0], 100), 
                      -self.convert_angle_to_custom_range(positions[1], 100), 
                      -self.convert_angle_to_custom_range(positions[2], 100), 
                      self.convert_angle_to_custom_range(positions[3], 50), 
                      self.convert_angle_to_custom_range(positions[4], 50), 
                      self.convert_angle_to_custom_range(positions[5], 50), 
                      ]
        # output_vels = [self.convert_radians_to_motor_rotation(velocities[0], 100), 
        #                self.convert_radians_to_motor_rotation(velocities[1], 100), 
        #                -self.convert_radians_to_motor_rotation(velocities[2], 100), 
        #                self.convert_radians_to_motor_rotation(velocities[3], 50), 
        #                self.convert_radians_to_motor_rotation(velocities[4], 50),
        #                self.convert_radians_to_motor_rotation(velocities[5], 50)]
        output_vels = [6, 6, 6, 6, 6, 6]
        #indexing [base_joint, shoulder_joint, elbow_joint, wrist_rotate, wrist_twist, wrist_rotate_two]
        self.get_logger().info(f'{output_pos}, {output_vels}')
        return output_pos, output_vels

    def translate_joint_to_electrical(self, joint_name, position):
        if joint_name == "base_joint":
            return -self.convert_angle_to_custom_range(position, 100)
        if joint_name == "shoulder_joint":
            return -self.convert_angle_to_custom_range(position, 100)
        if joint_name == "elbow_joint":
            return -self.convert_angle_to_custom_range(position, 100)
        if joint_name == "wrist_rotate":
            return self.convert_angle_to_custom_range(position, 50)
        if joint_name == "wrist_twist":
            return self.convert_angle_to_custom_range(position, 50)
        if joint_name == "wrist_rotate_two":
            return self.convert_angle_to_custom_range(position, 50)
        raise ValueError(f"Unknown arm joint: {joint_name}")

    def motor_rotation_to_joint_angle(self, joint_name, position):
        if joint_name in ("base_joint", "shoulder_joint", "elbow_joint"):
            return position * 2.0 * 3.141592653589793 / 100.0
        return -position * 2.0 * 3.141592653589793 / 50.0

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_MOTOR_IDS.keys())
        positions = []
        velocities = []

        try:
            for joint_name in msg.name:
                result = query_moteus_sync(self.controllers[joint_name])
                fault = result.values.get(moteus.Register.FAULT)
                if fault not in (None, 0):
                    self.get_logger().warn(f"{joint_name} moteus fault={fault}")
                    return

                motor_position = result.values.get(moteus.Register.POSITION)
                motor_velocity = result.values.get(moteus.Register.VELOCITY, 0.0)
                if motor_position is None:
                    self.get_logger().warn(f"{joint_name} query missing position")
                    return

                positions.append(self.motor_rotation_to_joint_angle(joint_name, motor_position))
                velocities.append(self.motor_rotation_to_joint_angle(joint_name, motor_velocity or 0.0))
        except Exception as exc:
            self.get_logger().warn(
                f"Could not publish moteus joint states: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        msg.position = positions
        msg.velocity = velocities
        self.joint_state_pub.publish(msg)

    def iter_joint_commands(self, msg):
        if not msg.points:
            self.get_logger().warning("Ignoring empty JointTrajectory.")
            return []

        point = msg.points[0]
        if len(msg.joint_names) != len(point.positions):
            self.get_logger().error(
                "Ignoring JointTrajectory with mismatched joint_names and positions."
            )
            return []

        commands = []
        for index, joint_name in enumerate(msg.joint_names):
            if joint_name not in JOINT_MOTOR_IDS:
                self.get_logger().warning(f"Ignoring unknown arm joint: {joint_name}")
                continue

            velocity_limit = 6
            if index < len(point.velocities) and point.velocities[index] > 0:
                velocity_limit = min(point.velocities[index], 6)

            commands.append(
                (
                    joint_name,
                    self.controllers[joint_name],
                    JOINT_MOTOR_IDS[joint_name],
                    self.translate_joint_to_electrical(joint_name, point.positions[index]),
                    velocity_limit,
                )
            )

        self.get_logger().info(
            f"JointTrajectory commands: {[(name, position, velocity) for name, _, _, position, velocity in commands]}"
        )
        return commands


    def listener_callback(self, msg):
        if self.use_legacy_ik_command_mapping:
            if not msg.points:
                self.get_logger().warning("Ignoring empty JointTrajectory.")
                return

            point = msg.points[0]
            if len(point.positions) < 6:
                self.get_logger().error(
                    "Ignoring JointTrajectory with fewer than 6 positions."
                )
                return

            positions, velocities = self.translate_to_electrical(
                point.positions,
                point.velocities,
            )
            for joint_name, position, velocity_limit in zip(
                JOINT_MOTOR_IDS.keys(),
                positions,
                velocities,
            ):
                send_moteus_command_sync(
                    controller=self.controllers[joint_name],
                    motor=JOINT_MOTOR_IDS[joint_name],
                    position=position,
                    drives_velocity=None,
                    maximum_torque=MAX_TORQUE,
                    velocity_limit=velocity_limit,
                    accel_limit=MAX_ACCEL,
                    ff_torque=None,
                    logger=logger,
                )
            return

        commands = self.iter_joint_commands(msg)

        for _, controller, motor_id, position, velocity_limit in commands:
            send_moteus_command_sync(controller=controller, motor=motor_id, position=position, drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=velocity_limit,  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)



    def listener_callback_mini(self, msg):  
        mini_arm_positions = [
            msg.base_angle,
            msg.shoulder_angle,
            msg.elbow_angle,
            msg.first_rotate_angle,
            msg.tilt_angle,
            msg.second_rotate_angle,
        ]

        if self.prev_mini_arm_positions is None:
            self.prev_mini_arm_positions = mini_arm_positions
            self.target_mini_arm_positions = [0.0] * len(mini_arm_positions)
            self.get_logger().info(
                f"Mini arm initialized at: {mini_arm_positions}"
            )
            return

        for index, position in enumerate(mini_arm_positions):
            delta = position - self.prev_mini_arm_positions[index]
            if abs(delta) < MINI_ARM_DEADBAND:
                delta = 0.0
            self.target_mini_arm_positions[index] += (
                MINI_ARM_SOFT_GEAR_RATIO[index] * delta
            )

        self.prev_mini_arm_positions = mini_arm_positions
        print(self.target_mini_arm_positions)

        send_moteus_command_sync(controller=self.controllers["base_joint"], motor=9, position=self.target_mini_arm_positions[0], drives_velocity=None, maximum_torque=MINI_ARM_MAX_TORQUE, velocity_limit=MINI_ARM_VELOCITY_LIMIT,  accel_limit=MINI_ARM_ACCEL_LIMIT, ff_torque=0.0, logger=logger)
        send_moteus_command_sync(controller=self.controllers["shoulder_joint"], motor=10, position=self.target_mini_arm_positions[1], drives_velocity=None, maximum_torque=MINI_ARM_MAX_TORQUE, velocity_limit=MINI_ARM_VELOCITY_LIMIT,  accel_limit=MINI_ARM_ACCEL_LIMIT, ff_torque=0.0, logger=logger)
        send_moteus_command_sync(controller=self.controllers["elbow_joint"], motor=11, position=self.target_mini_arm_positions[2], drives_velocity=None, maximum_torque=MINI_ARM_MAX_TORQUE, velocity_limit=MINI_ARM_VELOCITY_LIMIT,  accel_limit=MINI_ARM_ACCEL_LIMIT, ff_torque=0.0, logger=logger)
        send_moteus_command_sync(controller=self.controllers["wrist_rotate"], motor=12, position=self.target_mini_arm_positions[3], drives_velocity=None, maximum_torque=MINI_ARM_MAX_TORQUE, velocity_limit=MINI_ARM_VELOCITY_LIMIT,  accel_limit=MINI_ARM_ACCEL_LIMIT, ff_torque=0.0, logger=logger)
        send_moteus_command_sync(controller=self.controllers["wrist_twist"], motor=13, position=self.target_mini_arm_positions[4], drives_velocity=None, maximum_torque=MINI_ARM_MAX_TORQUE, velocity_limit=MINI_ARM_VELOCITY_LIMIT,  accel_limit=MINI_ARM_ACCEL_LIMIT, ff_torque=0.0, logger=logger)
        send_moteus_command_sync(controller=self.controllers["wrist_rotate_two"], motor=14, position=self.target_mini_arm_positions[5], drives_velocity=None, maximum_torque=MINI_ARM_MAX_TORQUE, velocity_limit=MINI_ARM_VELOCITY_LIMIT,  accel_limit=MINI_ARM_ACCEL_LIMIT, ff_torque=0.0, logger=logger)


def main(args=None):
    rclpy.init(args=args)
    init_moteus_loop()
    cmd_vel_subscriber = JSInputSubscriber()
    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()
