#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import GroundPlaneStamped
from cv_bridge import CvBridge
import numpy as np
import math
from collections import deque
import heapq  # For priority queue in A*

import rerun as rr

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')

        self.declare_parameter('real', True)  # False if running sim
        self.real = self.get_parameter('real').get_parameter_value().bool_value
        

        # Subscriptions
        self.next_target_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/target/global',
            self.target_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            TwistStamped,
            '/autonomy/pose/robot/global',
            self.update_pose,
            10
        )

        # Publishers 
        self.waypoint_publisher = self.create_publisher(Float32MultiArray, '/autonomy/target/local', 10)

        # Variables
        self.robot_position = [0.0, 0.0]
        self.yaw = 0.0
        self.curr_target = [0.0, 0.0]
        self.waypoints = deque()
        self.publish_timer = self.create_timer(0.5, self.publish_waypoint)
        self.get_logger().info("Global Planner Node initialized")

    def update_pose(self, msg: TwistStamped):
        x, y = msg.twist.linear.x, msg.twist.linear.y
        self.robot_position = [x, y]
        self.yaw = msg.twist.angular.z

    def target_callback(self, msg):
        self.curr_target = [float(msg.data[0]), float(msg.data[1])]
        
    def publish_waypoint(self):
        waypoint = Float32MultiArray()
        waypoint.data = self.curr_target
        self.waypoint_publisher.publish(waypoint)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
