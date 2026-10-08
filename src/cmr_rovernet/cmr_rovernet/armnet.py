import rclpy
from rclpy.node import Node
from cmr_msgs.msg import ControllerReading
from cmr_msgs.msg import MiniArmDegree
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import time
import math
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
        self.button_subscription = self.create_subscription(
            ControllerReading,
            '/arm_controller/cmd_buttons',
            self.listener_button_callback,
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
        self.moveit_mode = int(bool(node_config.get("start_in_moveit_mode", False)))
        self.arm_can_port = str(node_config.get("can_port", "/dev/ttyACM1"))
        self.publish_hardware_joint_states = bool(
            node_config.get("publish_hardware_joint_states", True)
        )
        self.use_legacy_ik_command_mapping = bool(
            node_config.get("use_legacy_ik_command_mapping", False)
        )
        self.require_hardware_state = bool(
            node_config.get("require_hardware_state", True)
        )
        self.hardware_state_timeout_s = float(
            node_config.get("hardware_state_timeout_s", 0.5)
        )
        self.max_joint_command_delta_rad = float(
            node_config.get("max_joint_command_delta_rad", 0.15)
        )
        self.motor_velocity_limit = float(
            node_config.get("motor_velocity_limit", 0.5)
        )
        self.motor_accel_limit = float(
            node_config.get("motor_accel_limit", 1.0)
        )
        self.maximum_torque = float(node_config.get("maximum_torque", MAX_TORQUE))
        self.joint_state_rate_hz = float(node_config.get("joint_state_rate_hz", 10.0))
        self.latest_joint_positions = None
        self.last_hardware_state_time = None
        self.hardware_state_ready_logged = False
        self.hardware_fault_active = False
        self.arm_transport, self.controllers = make_moteus_transport_and_controllers_sync(
            self.arm_can_port,
            JOINT_MOTOR_IDS,
        )
        self.joint_state_pub = None
        self.joint_state_timer = None
        if self.publish_hardware_joint_states:
            self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
            timer_period = 1.0 / max(self.joint_state_rate_hz, 1.0)
            self.joint_state_timer = self.create_timer(timer_period, self.publish_joint_states)
        self.get_logger().info(f"Arm moteus fdcanusb transport: {self.arm_can_port}")
        self.get_logger().info(
            f"Arm hardware joint-state publishing: {self.publish_hardware_joint_states}"
        )
        self.get_logger().info(
            f"Arm legacy IK command mapping: {self.use_legacy_ik_command_mapping}"
        )
        self.get_logger().info(
            "Arm safety limits: "
            f"velocity={self.motor_velocity_limit} rev/s, "
            f"acceleration={self.motor_accel_limit} rev/s^2, "
            f"max joint step={self.max_joint_command_delta_rad} rad"
        )
        self.get_logger().info(f"Arm initial mode: {'IK' if self.moveit_mode else 'mini arm'}")
        
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
        try:
            results = query_moteus_batch_sync(self.arm_transport, self.controllers)
            positions = []
            velocities = []
            for joint_name in msg.name:
                result = results.get(joint_name)
                if result is None:
                    raise RuntimeError(f"no reply from {joint_name}")
                fault = result.values.get(moteus.Register.FAULT)
                if fault not in (None, 0):
                    self.hardware_fault_active = True
                    self.get_logger().error(
                        f"BLOCKING ARM COMMANDS: {joint_name} moteus fault={fault}",
                        throttle_duration_sec=1.0,
                    )
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
                f"Could not read arm hardware state ({type(exc).__name__}): {exc!r}",
                throttle_duration_sec=1.0,
            )
            return

        self.hardware_fault_active = False
        self.latest_joint_positions = dict(zip(msg.name, positions))
        self.last_hardware_state_time = time.monotonic()
        msg.position = positions
        msg.velocity = velocities
        self.joint_state_pub.publish(msg)
        if not self.hardware_state_ready_logged:
            self.get_logger().info(
                "Arm hardware state ready; MoveIt commands are now allowed."
            )
            self.hardware_state_ready_logged = True

    def send_arm_position(self, joint_name, position, velocity_limit, maximum_torque=MAX_TORQUE, accel_limit=MAX_ACCEL):
        motor_id = JOINT_MOTOR_IDS[joint_name]
        try:
            send_moteus_command_sync(
                controller=self.controllers[joint_name],
                motor=motor_id,
                position=position,
                drives_velocity=None,
                maximum_torque=maximum_torque,
                velocity_limit=velocity_limit,
                accel_limit=accel_limit,
                ff_torque=None,
                logger=self.get_logger(),
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to command {joint_name} motor {motor_id} on {self.arm_can_port}: {exc!r}",
                throttle_duration_sec=1.0,
            )

    def decode_button_combo(self, msg):
        buttons = list(msg.button_array)
        if len(buttons) >= 8:
            trigger_val = 0
            button_val = 0
            if buttons[0]:
                trigger_val = L1
            elif buttons[1]:
                trigger_val = R1

            if buttons[4]:
                button_val = SQUARE
            elif buttons[5]:
                button_val = X
            elif buttons[6]:
                button_val = CIRCLE
            elif buttons[7]:
                button_val = TRIANGLE
            return trigger_val, button_val

        if len(buttons) >= 2:
            return int(buttons[0]), int(buttons[1])

        self.get_logger().warning(
            f"Ignoring arm button message with {len(buttons)} entries."
        )
        return 0, 0

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

        positions_by_name = dict(zip(msg.joint_names, point.positions))
        missing_joints = [name for name in JOINT_MOTOR_IDS if name not in positions_by_name]
        if missing_joints:
            self.get_logger().error(
                f"Ignoring incomplete JointTrajectory; missing {missing_joints}."
            )
            return []

        commands = []
        for index, joint_name in enumerate(msg.joint_names):
            if joint_name not in JOINT_MOTOR_IDS:
                self.get_logger().warning(f"Ignoring unknown arm joint: {joint_name}")
                continue

            joint_position = point.positions[index]
            if not math.isfinite(joint_position):
                self.get_logger().error(
                    f"Ignoring non-finite target for {joint_name}: {joint_position}"
                )
                return []

            commands.append(
                (
                    joint_name,
                    self.controllers[joint_name],
                    JOINT_MOTOR_IDS[joint_name],
                    self.translate_joint_to_electrical(joint_name, joint_position),
                    self.motor_velocity_limit,
                )
            )
        return commands

    def hardware_state_is_fresh(self):
        if self.hardware_fault_active or self.last_hardware_state_time is None:
            return False
        return (
            time.monotonic() - self.last_hardware_state_time
            <= self.hardware_state_timeout_s
        )

    def trajectory_is_safe(self, msg):
        if not msg.points:
            return False
        if self.require_hardware_state and not self.hardware_state_is_fresh():
            self.get_logger().error(
                "Ignoring trajectory: arm hardware state is missing, stale, or faulted.",
                throttle_duration_sec=1.0,
            )
            return False

        if self.latest_joint_positions is None:
            return not self.require_hardware_state

        point = msg.points[0]
        targets = dict(zip(msg.joint_names, point.positions))
        for joint_name, current_position in self.latest_joint_positions.items():
            if joint_name not in targets:
                continue
            delta = abs(targets[joint_name] - current_position)
            if delta > self.max_joint_command_delta_rad:
                self.get_logger().error(
                    "Ignoring unsafe trajectory jump: "
                    f"{joint_name} delta={delta:.3f} rad exceeds "
                    f"{self.max_joint_command_delta_rad:.3f} rad.",
                    throttle_duration_sec=1.0,
                )
                return False
        return True

    def send_arm_position_batch(self, commands):
        batch = [
            {
                "controller": controller,
                "position": position,
                "maximum_torque": self.maximum_torque,
                "velocity_limit": velocity_limit,
                "accel_limit": self.motor_accel_limit,
            }
            for _, controller, _, position, velocity_limit in commands
        ]
        try:
            results = send_moteus_position_batch_sync(self.arm_transport, batch)
            for joint_name, _, motor_id, _, _ in commands:
                result = results.get(motor_id)
                if result is None:
                    raise RuntimeError(f"no command reply from {joint_name} motor {motor_id}")
                fault = result.values.get(moteus.Register.FAULT)
                if fault not in (None, 0):
                    self.hardware_fault_active = True
                    self.get_logger().error(
                        f"{joint_name} motor {motor_id} faulted while commanding: {fault}"
                    )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to command arm batch ({type(exc).__name__}): {exc!r}",
                throttle_duration_sec=1.0,
            )


    def listener_callback(self, msg):
        if self.moveit_mode:
            if not self.trajectory_is_safe(msg):
                return

            commands = self.iter_joint_commands(msg)
            # base = byte_command_converter(ARM, ARM_BASE, positions[0], None, MAX_TORQUE, velocities[0], MAX_ACCEL, None, self.get_logger())
            # shoulder = byte_command_converter(ARM, ARM_SHOULDER, positions[1], None, MAX_TORQUE, velocities[1], MAX_ACCEL, None, self.get_logger())
            # elbow = byte_command_converter(ARM, ARM_ELBOW, positions[2], None, MAX_TORQUE, velocities[2], MAX_ACCEL, None, self.get_logger())
            # wrist_rotate_1 = byte_command_converter(ARM, WRIST_ROTATE_1, positions[3], None, MAX_TORQUE, velocities[3], MAX_ACCEL, None, self.get_logger())
            # wrist_tilt = byte_command_converter(ARM, WRIST_TILT, positions[4], None, MAX_TORQUE, velocities[4], MAX_ACCEL, None, self.get_logger())
            # wrist_rotate_2 = byte_command_converter(ARM, WRIST_ROTATE_2, positions[5], None, MAX_TORQUE, velocities[5], MAX_ACCEL, None, self.get_logger())
            # send_number(self.serial_port, base)
            # send_number(self.serial_port, shoulder)
            # send_number(self.serial_port, elbow)
            # send_number(self.serial_port, wrist_rotate_1)
            # send_number(self.serial_port, wrist_tilt)
            # send_number(self.serial_port, wrist_rotate_2)

            if len(commands) == len(JOINT_MOTOR_IDS):
                self.send_arm_position_batch(commands)



    def listener_callback_mini(self, msg):  
        if not self.moveit_mode:
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

            for joint_name, position in zip(JOINT_MOTOR_IDS.keys(), self.target_mini_arm_positions):
                self.send_arm_position(
                    joint_name,
                    position,
                    MINI_ARM_VELOCITY_LIMIT,
                    maximum_torque=MINI_ARM_MAX_TORQUE,
                    accel_limit=MINI_ARM_ACCEL_LIMIT,
                )



    def listener_button_callback(self, msg):
            trigger_val, button_val = self.decode_button_combo(msg)
            # self.logger.info(f'button_array: {msg.button_array[0]}')
            if trigger_val == R1 and button_val == SQUARE:
                self.moveit_mode = 0
                self.logger.info(f'Mini arm mode: {str(self.moveit_mode)}')
                print("Mini arm mode: " + str(self.moveit_mode))
            if trigger_val == R1 and button_val == CIRCLE:
                self.moveit_mode = 1
                self.logger.info(f'IK mode: {str(self.moveit_mode)}')
                print("MoveIt arm mode: " + str(self.moveit_mode))
            if trigger_val == R1 and button_val == TRIANGLE:
                self.moveit_mode = 1
                print("MoveIt arm mode: " + str(self.moveit_mode))


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
