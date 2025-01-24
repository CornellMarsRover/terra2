#!/usr/bin/env python3

"""
NavAckerman ROS2 Node

This node autonomously navigates the robot to predefined GPS waypoints using Ackermann steering.
It subscribes to pose and velocity data from a localization node and calculates movement commands
to reach the waypoints.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int16, Float32MultiArray
import yaml
import math
import os
import time
from typing import Dict, Tuple, Optional
from ament_index_python.packages import get_package_share_directory


class NavigationControlLoop(Node):
    def __init__(self):
        super().__init__('nav_control_loop')

        # Declare parameters
        self.declare_parameter('real', False)  # FALSE IF TESTING IN SIM
        self.declare_parameter('max_linear_vel', 0.6)  # m/s
        self.declare_parameter('waypoint_tolerance', 2.0)  # meters
        self.declare_parameter('proportional_gain', 0.5)  # Proportional gain for linear velocity

        # Get parameters
        self.real = self.get_parameter('real').get_parameter_value().bool_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.proportional_gain = self.get_parameter('proportional_gain').get_parameter_value().double_value

        # Choose which waypoints file to load
        if self.real:
            waypoints_file = 'config/waypoints_real.yaml'
        else:
            waypoints_file = 'config/waypoints.yaml'

        # Load waypoints
        self.waypoints = self.load_waypoints(waypoints_file)
        self.current_waypoint_index = 0

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Shutting down.')
            rclpy.shutdown()
            return

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

        # Subscriptions to localization data
        self.pose_subscription = self.create_subscription(
            Float32MultiArray, '/autonomy/pose/robot/global', self.pose_callback, 10
        )
        self.velocity_subscription = self.create_subscription(
            Float32MultiArray, '/autonomy/velocity', self.velocity_callback, 10
        )

        # Publishers
        self.coordinate_offset_publisher = self.create_publisher(
            Float32MultiArray, '/autonomy/coordinate_error', 10
        )
        self.waypoint_reached_publisher = self.create_publisher(
            Bool, '/autonomy/waypoint_reached', 10
        )
        self.waypoint_index_publisher = self.create_publisher(
            Int16, '/autonomy/waypoint_index', 10
        )

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Current state
        self.current_north = 0.0
        self.current_west = 0.0
        self.current_yaw = 0.0
        self.current_action = "drive_to_coord"

    def load_waypoints(self, waypoints_file):
        """
        Loads waypoints from a YAML file.
        """
        package_share = get_package_share_directory('autonomous_navigation')
        full_path = os.path.join(package_share, waypoints_file)

        if not os.path.isfile(full_path):
            self.get_logger().error(f'Waypoints file not found: {full_path}')
            return []

        with open(full_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
                self.get_logger().info('Waypoints loaded successfully.')
            except yaml.YAMLError as e:
                self.get_logger().error(f'Error parsing YAML file: {e}')
                return []

        waypoints = data.get('waypoints', [])
        return waypoints

    def pose_callback(self, msg):
        """
        Updates the current pose of the robot (north, west, yaw).
        """
        self.current_north = msg.data[0]
        self.current_west = msg.data[1]
        self.current_yaw = msg.data[2]

    def velocity_callback(self, msg):
        """
        Updates the current velocity of the robot (not used directly here but available if needed).
        """
        # Velocity information can be stored or logged if needed
        pass

    def control_loop(self):
        """
        Main control loop running at 10 Hz.
        Determines movement commands based on current position and waypoints.
        """
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping robot.')
            return

        if self.current_action == "drive_to_coord":
            self.drive_to_coord()

    def drive_to_coord(self):
        """
        Drives the robot toward the current waypoint using Ackermann steering.
        """
        waypoint = self.waypoints[self.current_waypoint_index]
        target_lat = waypoint['latitude']
        target_lon = waypoint['longitude']

        # Convert target GPS waypoint to displacement in the north/west directions
        target_north, target_west = self.get_north_west_meters(
            target_lat, target_lon
        )
        distance = math.sqrt((target_north - self.current_north)**2 + (target_west - self.current_west)**2)
        angle_to_target = math.atan2(target_west - self.current_west, target_north - self.current_north)

        # Calculate angle error
        angle_error = angle_to_target - self.current_yaw
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        if distance < self.waypoint_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}')
            waypoint_reached = Bool()
            waypoint_reached.data = True
            self.waypoint_reached_publisher.publish(waypoint_reached)
            self.current_waypoint_index += 1
            time.sleep(2)  # Optional pause
        else:
            self.publish_gps_offset(target_north - self.current_north, target_west - self.current_west, angle_error)

    def publish_gps_offset(self, north, west, yaw):
        """
        Publish the offset to the next GPS coordinate and the steer angle error.
        """
        offsets = [float(north), float(west), float(yaw)]
        offset_msg = Float32MultiArray()
        offset_msg.data = offsets
        self.coordinate_offset_publisher.publish(offset_msg)

    def get_north_west_meters(self, target_lat, target_lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters).
        """
        R = 6378137.0
        lat1_rad = math.radians(target_lat)
        lat2_rad = math.radians(self.current_north)
        lon1_rad = math.radians(target_lon)
        lon2_rad = math.radians(self.current_west)
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0
        delta_north = delta_lat * R
        delta_east = delta_lon * R * math.cos(mean_lat)
        return delta_north, -delta_east

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControlLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NavigationControlLoop node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
