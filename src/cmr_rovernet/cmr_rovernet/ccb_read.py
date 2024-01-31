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
        self.encoder_data_publisher_ = self.create_publisher(MotorReadData, '/ccb/cmd_vel', 10)
        self.read_timer = self.create_timer(0.1, self.read_data)
        self.timer = self.create_timer(0.5, self.publish_msg)
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
            # Add mappings for ARM TODO
        }

    def publish_msg(self):
        back_left_request = byte_command_converter(RECEIVE_DATA, BACK_LEFT, None, None, None, None, None, self.logger)
        back_right_request = byte_command_converter(RECEIVE_DATA, BACK_RIGHT, None, None, None, None, None, self.logger)
        front_left_request = byte_command_converter(RECEIVE_DATA, FRONT_LEFT, None, None, None, None, None, self.logger)
        front_right_request = byte_command_converter(RECEIVE_DATA, FRONT_RIGHT, None, None, None, None, None, self.logger)
        send_number(self.serial_port, back_right_request)
        send_number(self.serial_port, front_right_request)
        send_number(self.serial_port, front_left_request)
        send_number(self.serial_port, back_left_request)

    def read_data(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.read_until()  # Adjust read method based on your data format
            self.process_data(data)

    def process_data(self, data):
        format_string = 'BBBiii'
        try: 
            subteam, motor_id, mode, position, velocity, torque = struct.unpack(format_string, data)
            self.logger.info(f"Received - Subteam: {subteam}, Motor ID: {motor_id}, Mode: {mode}, Position: {position}, Velocity: {velocity}, Torque: {torque}")
            if motor_id in self.motor_data_mapping:
                motor_data = self.motor_data_mapping[motor_id]
                motor_data.mode = mode 
                motor_data.position = position 
                motor_data.velocity = velocity 
                motor_data.torque = torque 

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















