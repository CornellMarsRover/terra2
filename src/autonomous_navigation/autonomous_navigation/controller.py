#!/usr/bin/env python3

"""
NavAckerman ROS2 Node

This node autonomously navigates the robot to predefined GPS waypoints using Ackermann steering.
It subscribes to pose and velocity data from a localization node and calculates movement commands
to reach the waypoints.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray, String
import time
import math
from ament_index_python.packages import get_package_share_directory

from cmr_msgs.msg import AutonomyDrive


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('real', False) # FALSE IF RUNNING IN SIMULATION
        self.real = self.get_parameter('real').get_parameter_value().bool_value

        # Subscribe to the robot pose topic
        self.pose_subscription = self.create_subscription(
            TwistStamped,
            '/autonomy/pose/robot/global',
            self.update_pose,
            10
        )

        self.waypoint_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/path/next_waypoint',
            self.update_waypoint,
            10
        )

        # Store current robot position
        self.robot_position = (0.0, 0.0)
        self.yaw = 0.0

        # Drive command publishers
        self.ackerman_publisher = self.create_publisher(AutonomyDrive, '/autonomy/move/ackerman', 10)
        self.point_turn_publisher = self.create_publisher(Twist, '/autonomy/move/point_turn', 10)
        self.movement_id_publisher = self.create_publisher(String, '/autonomy/move/move_type', 10)
        self.point_turn_velocity = 0.062
        self.ackerman_velocity = 0.055

        # Store last movement for pausing before and after point turns
        self.last_movement = 'ackerman'
        self.last_command_time = self.get_clock().now().to_msg()
        self.min_wait = 2.0 # Minimum wait between different movement commands to allow wheel repositioning

        # Next waypoint in path
        self.waypoint = None

        self.drive_commander = self.create_timer(0.1, self.follow_waypoint)

    def follow_waypoint(self):
        """
        Logic for sending drive commands
        NEED TO ADD WAITS HERE OR IN DRIVES NODE
        """
        if self.waypoint is None:
            return

        x = self.waypoint[0] - self.robot_position[0]
        y = self.waypoint[1] - self.robot_position[1]
        angle_to_target = math.atan2(y, x)
        angle_error = math.degrees(angle_to_target - self.yaw)
        self.get_logger().info(f"Angle error to next waypoint: {angle_error}")
        if self.last_movement == 'ackerman':
            error_threshold = 20.0
        else:
            error_threshold = 10.0
        
        curr_time = self.get_clock().now().to_msg()
        dt = self.compute_time_delta(curr_time, self.last_command_time)
        if abs(angle_error) > error_threshold:
            n = -1.0
            if angle_error < 0:
                n = 1.0
            if self.last_movement == "ackerman" and dt < self.min_wait:
                self.publish_point_turn(n * 0.00001)
            else:
                self.publish_point_turn(n * self.point_turn_velocity)
                self.last_movement = 'point_turn'
                self.last_command_time = curr_time
        else:
            if self.last_movement == 'point_turn' and dt < self.min_wait:
                self.publish_ackerman(0.0, angle_error)
            else:
                self.publish_ackerman(self.ackerman_velocity, angle_error)
                self.last_movement = 'ackerman'
                self.last_command_time = curr_time
        
        self.publish_movement(self.last_movement)

    def publish_movement(self, movement):
        """
        Publishes id of last movement
        """
        msg = String()
        msg.data = movement
        self.movement_id_publisher.publish(msg)

    def publish_point_turn(self, point_turn_velocity):
        """
        Publishes point turn message
        """
        point_turn_msg = Twist()
        point_turn_msg.angular.z = point_turn_velocity
        self.point_turn_publisher.publish(point_turn_msg)

    def publish_ackerman(self, vel, steer_angle):
        """
        Publish ackerman drive message (steer_angle in degrees)
        """
        drive_msg = AutonomyDrive()
        drive_msg.vel = vel
        drive_msg.fl_angle = steer_angle
        drive_msg.fr_angle = steer_angle
        drive_msg.bl_angle = 0.0
        drive_msg.br_angle = 0.0

        self.ackerman_publisher.publish(drive_msg)

    def update_pose(self, msg):
        """
        Callback function to update the robot position and yaw
        """
        self.robot_position = (msg.twist.linear.x, msg.twist.linear.y)
        self.yaw = msg.twist.angular.z

    def update_waypoint(self, msg):
        """
        Callback function to update waypoint to follow
        """
        self.waypoint = (msg.data[0], msg.data[1])
    
    def compute_time_delta(self, current_stamp, last_stamp):
        """
        Compute time difference in seconds between two ROS 2 Time objects.
        """
        # In ROS 2, stamp.sec and stamp.nanosec are the fields
        dt = (current_stamp.sec - last_stamp.sec) + \
             (current_stamp.nanosec - last_stamp.nanosec) * 1e-9
        return dt
    
    def stop_robot(self):
        """
        Send drive command to stop the robot
        """

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down controller node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
