import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
import serial
from cmr_rovernet.rovernet_utils import *


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/drives_controller/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.current_speed = 0
        self.last_time = time.time()
        self.port = "/dev/ttyTHS0"
        self.baud_rate = 115200
        self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)

    
    def gradually_increase_speed_linear(self, target_speed):
        current_time = time.time()
        time_difference = current_time - self.last_time
        self.last_time = current_time

        # Base rate of speed change
        base_rate = 24 * time_difference

        # Modify the rate based on how close the target speed is to neutral (0)
        # The rate of change is faster when the joystick is near neutral
        dynamic_rate = base_rate * (1 - abs(target_speed) / 35)  # Assuming 35 is the max speed

        # Ensure dynamic_rate does not drop below a certain threshold
        dynamic_rate = max(dynamic_rate, base_rate * 0.5)

        if self.current_speed < target_speed:
            self.current_speed += dynamic_rate
            if self.current_speed > target_speed:
                self.current_speed = target_speed
        elif self.current_speed > target_speed:
            self.current_speed -= dynamic_rate
            if self.current_speed < target_speed:
                self.current_speed = target_speed


    def listener_callback(self, msg):
        target_speed = scale_value(msg.twist.linear.x, -2.5, 2.5, -53, 53)
        self.gradually_increase_speed_linear(target_speed)
        # if msg.twist.angular.z == 2.5: 
        #    output = byte_command_converter("drives", "back_right", None, None, None, None, None, self.get_logger())
        if msg.twist.linear.x != 0:
            back_right = byte_command_converter("drives", "back_right", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
            front_right = byte_command_converter("drives", "front_right", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
            front_left = byte_command_converter("drives", "front_left", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
            back_left = byte_command_converter("drives", "back_left", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
            send_number(self.serial_port, back_right)
            send_number(self.serial_port, front_right)
            send_number(self.serial_port, front_left)
            send_number(self.serial_port, back_left)
        
        if msg.twist.angular.z != 0:
            back_right = byte_command_converter("drives", "back_right", None, -self.current_speed, 1.5, None, 5.0, self.get_logger())
            front_right = byte_command_converter("drives", "front_right", None, -self.current_speed, 1.5, None, 5.0, self.get_logger())
            front_left = byte_command_converter("drives", "front_left", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
            back_left = byte_command_converter("drives", "back_left", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
            send_number(self.serial_port, back_right)
            send_number(self.serial_port, front_right)
            send_number(self.serial_port, front_left)
            send_number(self.serial_port, back_left)

        #output = byte_command_converter("drives", "back_right", None, self.current_speed, 1.5, None, 5.0, self.get_logger())
        #send_number(self.serial_port, output)




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
