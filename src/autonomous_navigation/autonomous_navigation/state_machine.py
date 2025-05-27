#!/usr/bin/env python3

"""
State Machine ROS2 Node
High-level logic for autonomy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int16, Float32MultiArray, String
from geometry_msgs.msg import Twist, TwistStamped
import yaml
import math
import os
import time
import numpy as np
from collections import deque
from typing import Dict, Tuple, Optional, Deque
from ament_index_python.packages import get_package_share_directory
from pyubx2 import llh2ecef

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # Declare parameters
        self.declare_parameter('real', True)  # FALSE IF TESTING IN SIM
        self.declare_parameter('waypoint_tolerance', 2.0)  # meters

        # Get parameters
        self.real = self.get_parameter('real').get_parameter_value().bool_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value

        # Choose which waypoints file to load
        if self.real:
            waypoints_file = 'config/waypoints_engquad.yaml'
        else:
            #waypoints_file = 'config/sim_waypoints_condensed.yaml'
            waypoints_file = 'config/waypoints.yaml'
            waypoints_file = 'config/waypoints_long.yaml'

        self.get_logger().info(f"Waypoints file: {waypoints_file}")
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
        self.next_coordinate_lat = self.next_waypoint['latitude']
        self.next_coordinate_lon = self.next_waypoint['longitude']

        # Change from initial pose
        self.north = 0.0 # meters
        self.west = 0.0 # meters
        self.yaw = 0.0 # radians

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')


        # Subscriptions to localization and game object data
        self.pose_subscription = self.create_subscription(
            TwistStamped, '/autonomy/pose/robot/global', self.pose_callback, 10
        )
        self.object_subscription = self.create_subscription(
            Twist, '/autonomy/target_object/position', self.object_callback, 10
        )

        # Publishers
        self.state_publisher = self.create_publisher(String, '/autonomy/state', 10)
        self.previous_target_publisher = self.create_publisher(Float32MultiArray, '/autonomy/target/previous', 10)
        self.next_coordinate_publisher = self.create_publisher(Float32MultiArray, '/autonomy/target/global', 10)
        self.object_publisher = self.create_publisher(String, '/autonomy/target_object/name', 10)
        self.previous_target = [0.0, 0.0]
        n, w = self.get_north_west_meters(self.next_coordinate_lat, self.next_coordinate_lon)
        self.next_coordinate = (n, w)
        
        # Targets for the mission with (object name, threshold, search_radius, radial step for search, angular step for search)
        self.targets = {
            1: ('coordinate', 2.0, 0.0, 0.0, 0.0),
            2: ('ar1', 1.5, 10.0, 0.2, 15.0),
            3: ('ar2', 1.5, 15.0, 0.2, 15.0),
            4: ('ar3', 1.5, 20.0, 0.2, 15.0), # 1
            5: ('ar1', 1.5, 10.0, 0.2, 30.0),
            6: ('ar2', 1.5, 15.0, 0.2, 30.0),
            7: ('ar3', 1.5, 20.0, 0.2, 30.0), # 2
            8: ('ar1', 1.5, 10.0, 0.2, 45.0),
            9: ('ar2', 1.5, 15.0, 0.2, 45.0),
            10: ('ar3', 1.5, 20.0, 0.2, 45.0), # 3
            11: ('ar1', 1.5, 10.0, 0.4, 15.0),
            12: ('ar2', 1.5, 15.0, 0.4, 15.0),
            13: ('ar3', 1.5, 20.0, 0.4, 15.0), # 4
            14: ('ar1', 1.5, 10.0, 0.4, 30.0),
            15: ('ar2', 1.5, 15.0, 0.4, 30.0),
            16: ('ar3', 1.5, 20.0, 0.4, 30.0), # 5
            17: ('ar1', 1.5, 10.0, 0.4, 45.0),
            18: ('ar2', 1.5, 15.0, 0.4, 45.0),
            19: ('ar3', 1.5, 20.0, 0.4, 45.0),  # 6
            20: ('ar1', 1.5, 10.0, 0.6, 30.0),
            21: ('ar2', 1.5, 15.0, 0.6, 30.0),
            22: ('ar3', 1.5, 20.0, 0.6, 30.0), # 7
            #5: ('mallet', 1.5, 10.0),
            #6: ('water bottle', 1.5, 10.0)
        }

        # Initial state is driving to first target coordinate
        self.current_state = None
        self.threshold = None
        self.search_radius = None
        self.r_step = None
        self.theta_step = None
        self.searched = False
        self.update_search_params() # Updates above values using current waypoint index

        self.target_position = None
        self.search_waypoints = deque()
        self.dist_to_coord = math.inf

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def update_targets(self):
        """
        Updates targets from gps coordinate
        """
        # Update distance to the target GPS coordinate
        dx = self.next_coordinate[0] - self.north
        dy = self.next_coordinate[1] - self.west
        self.dist_to_coord = math.sqrt(dx**2 + dy**2)
        
        reached = False
        # If driving to just coordinate, distance to coord is sufficient to check
        if self.dist_to_coord < self.threshold:
            if self.current_state == 'coordinate':
                reached = True
            elif len(self.search_waypoints) > 0 and self.target_position is None:
                self.next_coordinate = self.search_waypoints.popleft()
            elif self.target_position is None and not self.searched:
                self.search_waypoints = generate_search_points(self.next_coordinate, self.search_radius, self.r_step, self.theta_step)
                self.searched = True
                self.threshold = 2.0 # to account for badly selected waypoints
                self.get_logger().info(f"Search waypoints generated: {self.search_waypoints}")
            else:
                reached = True
                self.searched = False
        # If the object has been found and within allowed threshold, target reached
        elif self.target_position is not None:
            dx = self.target_position[0] - self.north
            dy = self.target_position[1] - self.west
            d = math.sqrt(dx**2 + dy**2)
            reached = (d < self.threshold)
            #self.get_logger().info(f"Distance to target object: {d}")
            if d < self.threshold * 2:
                self.publish_target_name()
        if not reached:
            return

        self.current_waypoint_index += 1
        if self.current_waypoint_index >= len(self.waypoints):
            return
        # Update target object and coordinates
        self.update_search_params()
        self.previous_target = self.next_coordinate
        self.next_waypoint = self.waypoints[self.current_waypoint_index]
        self.next_coordinate_lat = self.next_waypoint['latitude']
        self.next_coordinate_lon = self.next_waypoint['longitude']
        n, w = self.get_north_west_meters(self.next_coordinate_lat, self.next_coordinate_lon)
        self.next_coordinate = [n, w]


    def pose_callback(self, msg):
        """
        Updates the current pose of the robot (north, west, yaw).
        """
        self.north = msg.twist.linear.x
        self.west = msg.twist.linear.y
        self.yaw = msg.twist.angular.z
    
    def object_callback(self, msg):
        self.target_position = [msg.linear.x, msg.linear.y]
        self.get_logger().info(f"Current target {self.current_state} found at {self.target_position}")

    def control_loop(self):
        """
        Main control loop running at 10 Hz.
        Determines movement commands based on current position and waypoints.
        """
        self.publish_state()

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached.')
            return

        self.update_targets()
        self.publish_targets()

    def publish_targets(self):
        """
        Publish previous and next targets
        """
        prev_target = Float32MultiArray()
        prev_target.data = [float(self.previous_target[0]), float(self.previous_target[1])]
        self.previous_target_publisher.publish(prev_target)

        next_target = Float32MultiArray()
        if self.target_position is None:
            next_target.data = [float(self.next_coordinate[0]), float(self.next_coordinate[1])]
        else:
            next_target.data = [float(self.target_position[0]), float(self.target_position[1])]
        self.next_coordinate_publisher.publish(next_target)

        # Look for target if not yet found
        if self.dist_to_coord < self.search_radius and self.target_position is None:
            self.publish_target_name()

    def publish_target_name(self):
        """
        Publish the name of the autonomy game object to be searched for
        """
        object_msg = String()
        object_msg.data = self.current_state
        self.object_publisher.publish(object_msg)

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
    
    def update_search_params(self):
        """
        Update search params with the current waypoint index
        """
        t = self.targets[self.current_waypoint_index]
        self.current_state = t[0]
        self.threshold = t[1]
        self.search_radius = t[2]
        self.r_step = t[3]
        self.theta_step = t[4]
        # Reset object variables
        self.target_position = None
        self.search_waypoints = deque()
        
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

def generate_search_points(center, radius, radial_step, angular_step_deg):
    """
    Helper function to generate waypoints in an outwards spiral
    to look for objects once a coordinate has been reached
    """
    points = deque()
    r = 0
    theta = 0
    while r < radius:
        r += radial_step
        theta += angular_step_deg
        if theta >= 360:
            theta %= 360
        x = r * np.cos(math.radians(theta)) + center[0]
        y = r * np.sin(math.radians(theta)) + center[1]
        points.append((x, y))
    return points

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
