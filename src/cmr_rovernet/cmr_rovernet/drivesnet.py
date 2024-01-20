import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import DrivesControllerReading
import time
import serial
from cmr_rovernet.rovernet_utils import *


class CmdVelSubscriber(Node):
    """
    This node subscribes to the /drives_controller/cmd_vel topic output by the 
    drivescontroller node. It will then convert the output to a pre-defined 40-byte 
    format and send the output to the CCB via UART. 
    """
    
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/drives_controller/cmd_vel',
            self.listener_callback,
            10)
        self.button_subscription = self.create_subscription(
            DrivesControllerReading,
            '/drives_controller/cmd_buttons',
            self.listener_button_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_speed = 0
        self.current_speed_angular = 0
        self.last_time = time.time()
        self.port = "/dev/ttyTHS0"
        self.baud_rate = 115200
        self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
        # self.serial_port = None
        self.logger = self.get_logger()
        
        # Init constants given TOML file
        drives_controller_table = parse_toml("connect")
        drives_net_table = parse_toml("drivesnet")
        drives_net_node = drives_net_table['node']
        self.CONTROLLER_MAX_SPEED = drives_controller_table['node']['max_speed']
        self.MOTOR_MAX_SPEED = drives_net_node['motor_max_speed']
        self.GRADUAL_INCREASE_RATE = drives_net_node['gradual_increase_rate']
        self.BASE_DYNAMIC_RATIO = drives_net_node['base_dynamic_ratio']

    
    def gradually_increase_speed_linear(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time
        self.last_time = current_time

        # Base rate of speed change
        base_rate = self.GRADUAL_INCREASE_RATE * time_difference

        # Modify the rate based on how close the target speed is to neutral (0)
        # The rate of change is faster when the joystick is near neutral
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35)  # Assuming 35 is the max speed

        # Ensure dynamic_rate does not drop below a certain threshold
        dynamic_rate = max(dynamic_rate, base_rate * self.BASE_DYNAMIC_RATIO)

        if self.current_speed < target_speed:
            self.current_speed = min(self.current_speed+dynamic_rate, target_speed)
        elif self.current_speed > target_speed:
            self.current_speed = max(self.current_speed-dynamic_rate, target_speed)

    def gradually_increase_speed_angular(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time
        self.last_time = current_time

        # Base rate of speed change
        base_rate = self.GRADUAL_INCREASE_RATE * time_difference

        # Modify the rate based on how close the target speed is to neutral (0)
        # The rate of change is faster when the joystick is near neutral
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35)  # Assuming 35 is the max speed

        # Ensure dynamic_rate does not drop below a certain threshold
        dynamic_rate = max(dynamic_rate, base_rate * self.BASE_DYNAMIC_RATIO)

        if self.current_speed_angular < target_speed:
            self.current_speed_angular = min(self.current_speed_angular+dynamic_rate, target_speed)
        elif self.current_speed_angular > target_speed:
            self.current_speed_angular = max(self.current_speed_angular-dynamic_rate, target_speed)


    def listener_callback(self, msg):
        
        # if msg.twist.angular.z == 2.5: 
        #    output = byte_command_converter(DRIVES, "back_right", None, None, None, None, None, self.get_logger())
        if msg.twist.linear.x != 0:
            target_speed = scale_value(msg.twist.linear.x, -self.CONTROLLER_MAX_SPEED, self.CONTROLLER_MAX_SPEED, 
                                       -self.MOTOR_MAX_SPEED, self.MOTOR_MAX_SPEED)
            self.gradually_increase_speed_linear(target_speed)
            back_right = byte_command_converter(DRIVES, BACK_RIGHT, None, self.current_speed, 1.5, None, 5.0, self.logger)
            front_right = byte_command_converter(DRIVES, FRONT_RIGHT, None, self.current_speed, 1.5, None, 5.0, self.logger)
            front_left = byte_command_converter(DRIVES, FRONT_LEFT, None, self.current_speed, 1.5, None, 5.0, self.logger)
            back_left = byte_command_converter(DRIVES, BACK_LEFT, None, self.current_speed, 1.5, None, 5.0, self.logger)
            send_number(self.serial_port, back_right)
            send_number(self.serial_port, front_right)
            send_number(self.serial_port, front_left)
            send_number(self.serial_port, back_left)
            
        elif msg.twist.angular.z != 0:
            target_speed_angular = scale_value(msg.twist.angular.y, -self.CONTROLLER_MAX_SPEED, self.CONTROLLER_MAX_SPEED, 
                                       -self.MOTOR_MAX_SPEED, self.MOTOR_MAX_SPEED)
            self.gradually_increase_speed_angular(target_speed_angular)
            back_right = byte_command_converter(DRIVES, BACK_RIGHT, None, -self.current_speed_angular, 1.5, None, 5.0, self.logger)
            front_right = byte_command_converter(DRIVES, FRONT_RIGHT, None, -self.current_speed_angular, 1.5, None, 5.0, self.logger)
            front_left = byte_command_converter(DRIVES, FRONT_LEFT, None, self.current_speed_angular, 1.5, None, 5.0, self.logger)
            back_left = byte_command_converter(DRIVES, BACK_LEFT, None, self.current_speed_angular, 1.5, None, 5.0, self.logger)
            send_number(self.serial_port, back_right)
            send_number(self.serial_port, front_right)
            send_number(self.serial_port, front_left)
            send_number(self.serial_port, back_left)

        #output = byte_command_converter(DRIVES, "back_right", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
        #send_number(self.serial_port, output)
            
    def listener_button_callback(self, msg):
        trigger_val = msg.button_array[0]
        button_val = msg.button_array[1]
        # self.logger.info(f'button_array: {msg.button_array[1]}')
        if trigger_val == L2 and button_val == TRIANGLE: 
            self.current_speed = 0
            self.current_speed_angular = 0
            back_right_stop = byte_command_converter(DRIVES, BACK_RIGHT, None, None, None, None, None, self.logger)
            front_right_stop = byte_command_converter(DRIVES, FRONT_RIGHT, None, None, None, None, None, self.logger)
            front_left_stop = byte_command_converter(DRIVES, FRONT_LEFT, None, None, None, None, None, self.logger)
            back_left_stop = byte_command_converter(DRIVES, BACK_LEFT, None, None, None, None, None, self.logger)
            send_number(self.serial_port, back_right_stop)
            send_number(self.serial_port, front_right_stop)
            send_number(self.serial_port, front_left_stop)
            send_number(self.serial_port, back_left_stop)

        

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
