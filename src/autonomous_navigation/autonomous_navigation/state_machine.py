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
from std_msgs.msg import Bool, Int16, Float32MultiArray, String
import yaml
import math
import os
import time
from typing import Dict, Tuple, Optional
from ament_index_python.packages import get_package_share_directory


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # Declare parameters
        self.declare_parameter('real', False)  # FALSE IF TESTING IN SIM
        self.declare_parameter('waypoint_tolerance', 2.0)  # meters

        # Get parameters
        self.real = self.get_parameter('real').get_parameter_value().bool_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value

        # Choose which waypoints file to load
        if self.real:
            waypoints_file = 'config/waypoints_real.yaml'
        else:
            waypoints_file = 'config/waypoints.yaml'

        # Load waypoints
        # NOTE first coordinate is precise starting point
        self.waypoints = self.load_waypoints(waypoints_file)

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Shutting down.')
            rclpy.shutdown()
            return

        # First coordinate given is precise reference starting coordinate
        starting_waypoint = self.waypoints[0]
        self.initial_lat = starting_waypoint['latitude']
        self.initial_lon = starting_waypoint['longitude']
        self.previous_target_lat = self.initial_lat
        self.previous_target_lon = self.initial_lon

        # Define first target
        self.current_waypoint_index = 1
        self.next_waypoint = self.waypoints[self.current_waypoint_index]
        self.next_target_lat = self.next_waypoint['latitude']
        self.next_target_lon = self.next_waypoint['longitude']

        # Change from initial pose
        self.d_north = 0.0 # meters
        self.d_west = 0.0 # meters
        self.yaw = 0.0 # radians

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

        # Define states
        self.states = {'drive_to_coord',
                       'search_ar_tag',
                       'search_water_bottle',
                       'search_mallet'}
                       
        # Initial state is driving to first target coordinate
        self.current_state = 'drive_to_coord'

        # Subscriptions to localization and movement data
        self.pose_subscription = self.create_subscription(
            Float32MultiArray, '/autonomy/pose/robot/global', self.pose_callback, 10
        )
        self.velocity_subscription = self.create_subscription(
            Float32MultiArray, '/autonomy/velocity', self.velocity_callback, 10
        )

        # Publisher to broadcast current state
        self.state_publisher = self.create_publisher(String, '/autonomy/state', 10)

        # Publishers to broadcast previous and next position targets for planning
        # Format is [delta_north in m, delta_west in m, desired_yaw in rad (optional)]
        self.previous_target_publisher = self.create_publisher(Float32MultiArray, '/autonomy/target/previous', 10)
        self.next_target_publisher = self.create_publisher(Float32MultiArray, '/autonomy/target/next', 10)
        
        self.previous_target = [0.0, 0.0]
        n, w = self.get_north_west_meters(self.next_target_lat, self.next_target_lon)
        self.next_target = [n, w]

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        """
        Updates the current pose of the robot (north, west, yaw).
        """
        self.d_north = msg.data[0]
        self.d_west = msg.data[1]
        self.yaw = msg.data[2]

    def velocity_callback(self, msg):
        """
        Updates the current velocity of the robot (not used directly here but available if needed).
        """
        return

    def control_loop(self):
        """
        Main control loop running at 10 Hz.
        Determines movement commands based on current position and waypoints.
        """
        self.publish_state()

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached.')
            return

        if self.current_state == 'drive_to_coord':
            waypoint = self.waypoints[self.current_waypoint_index]
            lat = waypoint['latitude']
            lon = waypoint['longitude']
            self.publish_targets()

    def publish_targets(self):
        """
        Publish previous and next targets
        """
        prev_target = Float32MultiArray()
        prev_target.data = [float(self.previous_target[0]), float(self.previous_target[1])]

        next_target = Float32MultiArray()
        next_target.data = [float(self.next_target[0]), float(self.next_target[1])]

        self.previous_target_publisher.publish(prev_target)
        self.next_target_publisher.publish(next_target)
    
    def get_north_west_meters(self, target_lat, target_lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        using previous GPS coordinate as a reference
        Publishes to the target pose topic
        """
        R = 6378137.0
        lat1_rad = math.radians(target_lat)
        lat2_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(target_lon)
        lon2_rad = math.radians(self.initial_lon)
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        # Compute north west distances
        north = -1.0 * delta_lat * R
        west = delta_lon * R * math.cos(mean_lat)
        return north, west

    def publish_state(self):
        """
        Publish current state
        """
        state = String()
        state.data = self.current_state
        self.state_publisher.publish(state)

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

    def destroy_node(self):
        super().destroy_node()

    '''
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
    '''

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down state machine node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
