import rclpy
from rclpy.node import Node
from cmr_msgs.msg import JoystickReading
import time
import serial
from cmr_rovernet.rovernet_utils import *


class JSInputSubscriber(Node):
    """
    This node subscribes to the /js_input topic output by the 
    armcontroller node. It will then convert the output to a pre-defined 40-byte 
    format and send the output to the CCB via UART. 
    """
    
    def __init__(self):
        super().__init__('js_input_subscriber')
        self.subscription = self.create_subscription(
            JoystickReading,
            '/js_input',
            self.listener_callback,
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
        arm_controller_table = parse_toml("TODO")
        self.CONTROLLER_MAX_SPEED = arm_controller_table['node']['max_speed']

        arm_net_table = parse_toml("armnet")
        arm_net_node = arm_net_table['node']
        self.MOTOR_MAX_SPEED = arm_net_node['motor_max_speed']
        self.GRADUAL_INCREASE_RATE = arm_net_node['gradual_increase_rate']
        self.BASE_DYNAMIC_RATIO = arm_net_node['base_dynamic_ratio']
        
    def listener_callback(self, msg):
        #TODO: figure out what commands to send/fine-tune parameters
        return



def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = JSInputSubscriber()
    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()
