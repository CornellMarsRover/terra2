import rclpy
from rclpy.node import Node
from cmr_msgs.msg import ControllerReading
from cmr_msgs.msg import MiniArmDegree
from trajectory_msgs.msg import JointTrajectory
import time
import serial
from cmr_rovernet.rovernet_utils import *

MAX_TORQUE = 0.7
MAX_ACCEL = 5.0

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
        # self.port = "/dev/ttyTHS0"
        self.baud_rate = 115200
        # self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
        self.moveit_mode = 1
        # self.serial_port = None
        self.logger = self.get_logger()
        
        # Init constants given TOML file
        # arm_controller_table = parse_toml("TODO")
        # self.CONTROLLER_MAX_SPEED = arm_controller_table['node']['max_speed']

        # arm_net_table = parse_toml("armnet")
        # arm_net_node = arm_net_table['node']
        # self.MOTOR_MAX_SPEED = arm_net_node['motor_max_speed']
        # self.GRADUAL_INCREASE_RATE = arm_net_node['gradual_increase_rate']
        # self.BASE_DYNAMIC_RATIO = arm_net_node['base_dynamic_ratio']
        

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
                      self.convert_angle_to_custom_range(positions[1], 100), 
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


    def listener_callback(self, msg):
        if self.moveit_mode:
            positions, velocities = self.translate_to_electrical(msg.points[0].positions, msg.points[0].velocities)
            print(positions)
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

            base = moteus.Controller(id=9)
            shoulder = moteus.Controller(id=10)
            elbow = moteus.Controller(id=11)
            wrist_rotate_1 = moteus.Controller(id=12)
            wrist_tilt = moteus.Controller(id=13)
            wrist_rotate_2 = moteus.Controller(id=14)

            send_moteus_command_sync(controller=base, motor=9, position=positions[0], drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=velocities[0],  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=shoulder, motor=10, position=positions[1], drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=velocities[1],  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=elbow, motor=11, position=positions[2], drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=velocities[2],  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=wrist_rotate_1, motor=12, position=positions[3], drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=velocities[3],  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=wrist_tilt, motor=13, position=positions[4], drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=velocities[4],  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=wrist_rotate_2, motor=14, position=positions[5], drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=velocities[5],  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)



    def listener_callback_mini(self, msg):  
        if not self.moveit_mode:
            max_velocity = 6
            print([msg.base_angle, msg.shoulder_angle, msg.elbow_angle, msg.first_rotate_angle, msg.tilt_angle, msg.second_rotate_angle])
            # base = byte_command_converter(ARM, ARM_BASE, msg.base_angle, None, MAX_TORQUE, max_velocity, MAX_ACCEL, None, self.get_logger())
            # shoulder = byte_command_converter(ARM, ARM_SHOULDER, msg.shoulder_angle, None, MAX_TORQUE, max_velocity, MAX_ACCEL, None, self.get_logger())
            # elbow = byte_command_converter(ARM, ARM_ELBOW, msg.elbow_angle, None, MAX_TORQUE, max_velocity, MAX_ACCEL, None, self.get_logger())
            # wrist_rotate_1 = byte_command_converter(ARM, WRIST_ROTATE_1, msg.first_rotate_angle, None, MAX_TORQUE, max_velocity, MAX_ACCEL, None, self.get_logger())
            # wrist_tilt = byte_command_converter(ARM, WRIST_TILT, msg.tilt_angle, None, MAX_TORQUE, max_velocity, MAX_ACCEL, None, self.get_logger())
            # wrist_rotate_2 = byte_command_converter(ARM, WRIST_ROTATE_2, msg.second_rotate_angle, None, MAX_TORQUE, max_velocity, MAX_ACCEL, None, self.get_logger())
            # send_number(self.serial_port, base)
            # send_number(self.serial_port, shoulder)
            # send_number(self.serial_port, elbow)
            # send_number(self.serial_port, wrist_rotate_1)
            # send_number(self.serial_port, wrist_tilt)
            # send_number(self.serial_port, wrist_rotate_2)

            base = moteus.Controller(id=9)
            shoulder = moteus.Controller(id=10)
            elbow = moteus.Controller(id=11)
            wrist_rotate_1 = moteus.Controller(id=12)
            wrist_tilt = moteus.Controller(id=13)
            wrist_rotate_2 = moteus.Controller(id=14)

            send_moteus_command_sync(controller=base, motor=9, position=msg.base_angle, drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=max_velocity,  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=shoulder, motor=10, position=msg.shoulder_angle, drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=max_velocity,  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=elbow, motor=11, position=msg.elbow_angle, drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=max_velocity,  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=wrist_rotate_1, motor=12, position=msg.tilt_angle, drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=max_velocity,  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=wrist_tilt, motor=13, position=msg.first_rotate_angle, drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=max_velocity,  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)
            send_moteus_command_sync(controller=wrist_rotate_2, motor=14, position=msg.second_rotate_angle, drives_velocity=None, maximum_torque=MAX_TORQUE, velocity_limit=max_velocity,  accel_limit=MAX_ACCEL, ff_torque=None, logger=logger)



    def listener_button_callback(self, msg):
            trigger_val = msg.button_array[0]
            button_val = msg.button_array[1]
            # self.logger.info(f'button_array: {msg.button_array[0]}')
            if trigger_val == L1 and button_val == SQUARE:
                self.moveit_mode = 0
                print("Mini arm mode: " + str(self.moveit_mode))
            if trigger_val == L1 and button_val == CIRCLE:
                self.moveit_mode = 1
                print("MoveIt arm mode: " + str(self.moveit_mode))
            if trigger_val == R1 and button_val == SQUARE:
                self.moveit_mode = 1
                print("MoveIt arm mode: " + str(self.moveit_mode))
            if trigger_val == R1 and button_val == CIRCLE:
                self.moveit_mode = 1
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
