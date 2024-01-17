import rclpy
from rclpy.node import Node
from cmr_msgs.msg import JoystickReading
import time
import serial
from cmr_rovernet.rovernet_utils import *


class JSInputSubscriber(Node):
    def __init__(self):
        next()


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = JSInputSubscriber()
    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
