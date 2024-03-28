import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import ControllerReading
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
            ControllerReading,
            '/drives_controller/cmd_buttons',
            self.listener_button_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_speed_left = 0
        self.current_speed_right = 0
        self.turn_thread_lock = 0
        self.last_time_left = time.time()
        self.last_time_right = time.time()
        self.port = "/dev/ttyTHS0"
        self.baud_rate = 115200
        # self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
        self.feed_forward_torque = 0.0
        self.serial_port = None
        self.logger = self.get_logger()
        
        # Init constants given TOML file
        #drives_controller_table = parse_toml("connect")
        drives_net_table = parse_toml("drivesnet")
        drives_net_node = drives_net_table['node']
        self.CONTROLLER_MAX_SPEED = 2.5
        self.MOTOR_MAX_SPEED = drives_net_node['motor_max_speed']
        self.GRADUAL_INCREASE_RATE = drives_net_node['gradual_increase_rate']
        self.BASE_DYNAMIC_RATIO = drives_net_node['base_dynamic_ratio']
        self.MAX_ACCELERATION = drives_net_node['acceleration_limit']
        self.MAX_TORQUE = drives_net_node['torque_limit']
        self.MAX_FEED_FORWARD_TORQUE = drives_net_node['feed_forward_torque_limit']

    # def set_feed_forward_torque(self, trigger_input):
    #     trigger_input = trigger_input/255
    #     ff_torque_output = trigger_input*self.MAX_FEED_FORWARD_TORQUE
    #     self.feed_forward_torque = ff_torque_output
    #     return ff_torque_output
    
    def gradually_increase_speed_left(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time_left
        self.last_time_left = current_time

        # Base rate of speed change
        base_rate = self.GRADUAL_INCREASE_RATE * time_difference

        # Modify the rate based on how close the target speed is to neutral (0)
        # The rate of change is faster when the joystick is near neutral
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35)  # Assuming 35 is the max speed

        # Ensure dynamic_rate does not drop below a certain threshold
        dynamic_rate = max(dynamic_rate, base_rate * self.BASE_DYNAMIC_RATIO)


        if self.current_speed_left < target_speed:
            self.current_speed_left = min(self.current_speed_left+dynamic_rate, target_speed)
        elif self.current_speed_left > target_speed:
            self.current_speed_left = max(self.current_speed_left-dynamic_rate, target_speed)

    def gradually_increase_speed_right(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time_right
        self.last_time_right = current_time

        # Base rate of speed change
        base_rate = self.GRADUAL_INCREASE_RATE * time_difference

        # Modify the rate based on how close the target speed is to neutral (0)
        # The rate of change is faster when the joystick is near neutral
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35)  # Assuming 35 is the max speed
        # Ensure dynamic_rate does not drop below a certain threshold
        dynamic_rate = max(dynamic_rate, base_rate * self.BASE_DYNAMIC_RATIO)

        if self.current_speed_right < target_speed:
            self.current_speed_right = min(self.current_speed_right+dynamic_rate, target_speed)
        elif self.current_speed_right > target_speed:
            self.current_speed_right = max(self.current_speed_right-dynamic_rate, target_speed)
        #self.logger.info(f'{self.current_speed_right}')


    def listener_callback(self, msg):
        
        # if msg.twist.angular.z == 2.5: 
        #    output = byte_command_converter(DRIVES, "back_right", None, None, None, None, None, self.get_logger())
        target_speed_left = scale_value(msg.twist.linear.y, -self.CONTROLLER_MAX_SPEED, self.CONTROLLER_MAX_SPEED, 
                                -self.MOTOR_MAX_SPEED, self.MOTOR_MAX_SPEED)
        target_speed_right = scale_value(msg.twist.angular.y, -self.CONTROLLER_MAX_SPEED, self.CONTROLLER_MAX_SPEED, 
                                -self.MOTOR_MAX_SPEED, self.MOTOR_MAX_SPEED)
        self.gradually_increase_speed_left(target_speed_left)
        self.gradually_increase_speed_right(target_speed_right)
        if self.current_speed_right < 0.2 and self.current_speed_right > -0.2:
            right_speed = 0
        else:
            right_speed = self.current_speed_right

        if self.current_speed_left < 0.2 and self.current_speed_left > -0.2:
            left_speed = 0
        else:
            left_speed = self.current_speed_left
        back_right =  byte_command_converter(DRIVES, BACK_RIGHT, None, -left_speed, self.MAX_TORQUE, None,  self.MAX_ACCELERATION, self.feed_forward_torque, self.logger)
        front_right = byte_command_converter(DRIVES, FRONT_RIGHT, None, -left_speed, self.MAX_TORQUE, None, self.MAX_ACCELERATION, self.feed_forward_torque, self.logger)
        front_left =  byte_command_converter(DRIVES, FRONT_LEFT, None, left_speed, self.MAX_TORQUE, None,    self.MAX_ACCELERATION, self.feed_forward_torque, self.logger)
        back_left =   byte_command_converter(DRIVES, BACK_LEFT, None, left_speed, self.MAX_TORQUE, None,     self.MAX_ACCELERATION, self.feed_forward_torque, self.logger)
        self.logger.info(f'Left Speed: {left_speed}, Right Speed: {right_speed}')
        # send_number(self.serial_port, back_right)
        # send_number(self.serial_port, front_right)
        # send_number(self.serial_port, front_left)
        # send_number(self.serial_port, back_left)

        #output = byte_command_converter(DRIVES, "back_right", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
        #send_number(self.serial_port, output)
    
    def r2TriggerConverter(self, val):
        # Convert the integer to a hexadecimal string
        hex_value = hex(val & 0xFFFFFFFF)  # Mask with 0xFFFFFFFF to handle negative values correctly
        # Take the first two hex digits (after '0x')
        first_two_hex = hex_value[2:4]
        # Convert the two hex digits back to an integer
        result_int = int(first_two_hex, 16)
        return result_int
            
    def listener_button_callback(self, msg):
        trigger_val = msg.button_array[0]
        button_val = msg.button_array[1]
        # self.logger.info(f'button_array: {msg.button_array[0]}')
        R2trigger = self.r2TriggerConverter(trigger_val)
        if R2trigger >= 0 and R2trigger <= 255:
            FF_val = scale_value(R2trigger, float(0), float(255), 0, self.MAX_FEED_FORWARD_TORQUE)
            self.set_feed_forward_torque(FF_val)
            # self.logger.info(f'R2: {r2val}')
        if trigger_val == L2 and button_val == TRIANGLE: 
            self.current_speed = 0
            self.current_speed_angular = 0
            back_right_stop = byte_command_converter(DRIVES, BACK_RIGHT, None, None, None, None, None, None, self.logger)
            front_right_stop = byte_command_converter(DRIVES, FRONT_RIGHT, None, None, None, None, None, None, self.logger)
            front_left_stop = byte_command_converter(DRIVES, FRONT_LEFT, None, None, None, None, None, None, self.logger)
            back_left_stop = byte_command_converter(DRIVES, BACK_LEFT, None, None, None, None, None, None, self.logger)
            # send_number(self.serial_port, back_right_stop)
            # send_number(self.serial_port, front_right_stop)
            # send_number(self.serial_port, front_left_stop)
            # send_number(self.serial_port, back_left_stop)
        if trigger_val == L2 and button_val == CIRCLE: 
            self.turn_thread_lock = not self.turn_thread_lock
            if self.turn_thread_lock == False:
                self.logger.info('Switching to Linear')
            else:
                self.logger.info('Switching to Angular')
        if trigger_val == L2 and button_val == X:
            self.feed_forward_torque += 0.1
        if trigger_val == L1 and button_val == X: 
            self.feed_forward_torque = 0 

        

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
