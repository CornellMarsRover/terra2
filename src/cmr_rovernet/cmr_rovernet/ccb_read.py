import rclpy
from rclpy.node import Node
from cmr_msgs.msg import MotorReadData
import time
import serial
from cmr_rovernet.rovernet_utils import *

class CCBReadPublisher(Node):
    """
    This node subscribes to the /drives_controller/cmd_vel topic output by the 
    drivescontroller node. It will then convert the output to a pre-defined 40-byte 
    format and send the output to the CCB via UART. 
    """

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.encoder_data_publisher_ = self.create_publisher(MotorReadData, '/ccb/read', 10)
        #self.read_timer = self.create_timer(0.1, self.read_data)
        self.timer = self.create_timer(0.001, self.publish_msg)
        self.port = "/dev/ttyTHS0"
        self.baud_rate = 115200
        self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
        self.logger = self.get_logger()
        self.stored_data = MotorReadData()
        self.motor_data_mapping = {
            FRONT_LEFT: self.stored_data.front_left,
            BACK_RIGHT: self.stored_data.back_right,
            FRONT_RIGHT: self.stored_data.front_right,
            BACK_LEFT: self.stored_data.back_left,
            ARM_BASE: self.stored_data.base, 
            ARM_SHOULDER: self.stored_data.shoulder,
            ARM_ELBOW: self.stored_data.elbow,
            WRIST_ROTATE_1: self.stored_data.wrist_r1, 
            WRIST_TILT: self.stored_data.wrist_twist,
            WRIST_ROTATE_2: self.stored_data.wrist_r2,
            FRONT_LEFT_SWERVE: self.stored_data.front_left_swerve, 
            BACK_RIGHT_SWERVE: self.stored_data.back_right_swerve, 
            FRONT_RIGHT_SWERVE: self.stored_data.front_right_swerve,
            BACK_LEFT_SWERVE: self.stored_data.back_left_swerve
        }

    def publish_msg(self):
        back_right_request = byte_command_converter(RECEIVE_DATA, BACK_RIGHT, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, back_right_request)
        self.read_data()

        front_right_request = byte_command_converter(RECEIVE_DATA, FRONT_RIGHT, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, front_right_request)
        self.read_data()

        front_left_request = byte_command_converter(RECEIVE_DATA, FRONT_LEFT, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, front_left_request)
        self.read_data()

        back_left_request = byte_command_converter(RECEIVE_DATA, BACK_LEFT, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, back_left_request)
        self.read_data()

        base_request = byte_command_converter(RECEIVE_DATA, ARM_BASE, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, base_request)
        self.read_data()

        shoulder_request = byte_command_converter(RECEIVE_DATA, ARM_SHOULDER, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, shoulder_request)
        self.read_data()

        elbow_request = byte_command_converter(RECEIVE_DATA, ARM_ELBOW, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, elbow_request)
        self.read_data()

        wrist_rotate_1_request = byte_command_converter(RECEIVE_DATA, WRIST_ROTATE_1, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, wrist_rotate_1_request)
        self.read_data()

        wrist_twist_request = byte_command_converter(RECEIVE_DATA, WRIST_ROTATE_2, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, wrist_twist_request)
        self.read_data()

        wrist_rotate_2_request = byte_command_converter(RECEIVE_DATA, WRIST_TILT, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, wrist_rotate_2_request)
        self.read_data()

        back_right_swerve_request = byte_command_converter(RECEIVE_DATA, BACK_RIGHT_SWERVE, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, back_right_swerve_request)
        self.read_data()

        front_right_swerve_request = byte_command_converter(RECEIVE_DATA, FRONT_RIGHT_SWERVE, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, front_right_swerve_request)
        self.read_data()

        front_left_swerve_request = byte_command_converter(RECEIVE_DATA, FRONT_LEFT_SWERVE, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, front_left_swerve_request)
        self.read_data()

        back_left_swerve_request = byte_command_converter(RECEIVE_DATA, BACK_LEFT_SWERVE, 10, 10, 10, 10, 10, 10, self.logger)
        send_number(self.serial_port, back_left_swerve_request)
        self.read_data()

    def read_data(self):
        data = self.serial_port.read(20)  # Adjust read method based on your data format
        #self.logger.info(f'data: {data.hex()}')
        self.process_data(data)
            
    def process_data(self, data):
        format_string = '<BBBffffB'
        try: 
            subteam, motor_id, mode, position, velocity, torque, current, fault = struct.unpack(format_string, data)
            #self.logger.info(f"Received - Subteam: {subteam}, Motor ID: {motor_id}, Mode: {mode}, Position: {position}, Velocity: {velocity}, Torque: {torque}, Current: {current}")
            if motor_id in self.motor_data_mapping:
                motor_data = self.motor_data_mapping[motor_id]
                motor_data.mode = mode 
                motor_data.position = position
                #self.logger.info(f'pos: {position.hex()}')
                motor_data.velocity = velocity
                #self.logger.info(f'vel: {velocity.hex()}')
                motor_data.torque = torque 
                #self.logger.info(f'tor: {torque.hex()}')
                motor_data.current = current
                #self.logger.info(f'curr: {current.hex()}')
            self.encoder_data_publisher_.publish(self.stored_data)

        except struct.error as e: 
            self.logger.error(f"Error unpacking data: {e}")

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CCBReadPublisher()
    rclpy.spin(cmd_vel_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()















