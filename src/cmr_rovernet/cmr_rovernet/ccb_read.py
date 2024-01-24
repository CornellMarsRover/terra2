import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import DrivesControllerReading
import time
# import serial
# from cmr_rovernet.rovernet_utils import *


# class CCBReadPublisher(Node):
#     """
#     This node subscribes to the /drives_controller/cmd_vel topic output by the 
#     drivescontroller node. It will then convert the output to a pre-defined 40-byte 
#     format and send the output to the CCB via UART. 
#     """

#     def __init__(self):
#         super().__init__('cmd_vel_publisher')
#         self.encoder_data_publisher_ = self.create_publisher(DrivesControlleReading, '/ccb/cmd_vel', 10)
#         self.timer = self.create_timer(0.1, self.publish_msg)

#     def publish_msg(self):



