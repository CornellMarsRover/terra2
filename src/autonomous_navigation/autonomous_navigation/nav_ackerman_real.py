#!/usr/bin/env python3

"""
NavAckerman ROS2 Node

This node autonomously navigates the robot to predefined GPS waypoints using Ackermann steering.
It interfaces with the combined `controller` node by:
- Publishing `ackerman` to `/control_mode` to activate Ackermann control mode.
- Publishing linear velocity commands to `/cmd_vel`.
- Publishing yaw error to `/angle_error`.

The node processes GPS and IMU data to determine the robot's current position and orientation,
calculates the necessary heading towards each waypoint, and issues movement commands accordingly.
Yaw corrections are handled by the `controller` node's PID controller.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
import time
from cmr_msgs.msg import IMUSensorData

class NavAckermanReal(Node):
    def __init__(self):
        super().__init__('nav_ackerman_real')

        # Declare parameters
        self.declare_parameter('waypoints_file', 'config/waypoints.yaml')
        self.declare_parameter('max_linear_vel', 0.6)  # m/s
        self.declare_parameter('waypoint_tolerance', 2.0)  # meters
        self.declare_parameter('proportional_gain', 0.5)  # Proportional gain for linear velocity

        # Get parameters
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.proportional_gain = self.get_parameter('proportional_gain').get_parameter_value().double_value

        # Load waypoints
        self.waypoints = self.load_waypoints(waypoints_file)
        self.current_waypoint_index = 0

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Shutting down.')
            rclpy.shutdown()
            return

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

        # Publisher to /control_mode to activate Ackermann mode
        self.control_mode_publisher = self.create_publisher(String, '/control_mode', 10)
        self.publish_ackerman_true()

        # Publisher for desired (x), current (y), and error (z) yaw angles
        self.angle_publisher = self.create_publisher(Twist, '/angle_error', 10)
        self.get_logger().info("Publishing to /angle_error")

        # Publisher for linear velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publishing to /cmd_vel")

        # Subscriber to GPS
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            'navsatfixdata',
            self.gps_callback,
            10
        )
        self.get_logger().info("Subscribed to navsatfixdata")

        # Subscriber to IMU
        self.imu_subscriber = self.create_subscription(
            IMUSensorData,
            '/imu',
            self.imu_callback,
            10
        )
        self.get_logger().info("Subscribed to /imu")

        # Current position and orientation
        self.current_position = None
        self.current_yaw = None  # In radians

        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def load_waypoints(self, waypoints_file):
        """
        Loads waypoints from a YAML file.

        Expected YAML format:
        waypoints:
          - latitude: <float>
            longitude: <float>
          - latitude: <float>
            longitude: <float>
          ...
        """
        # Resolve the full path
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

    def publish_ackerman_true(self):
        """
        Publishes ackerman to /control_mode to activate Ackermann control mode.
        This is published once at startup.
        """
        msg = String()
        msg.data = "ackerman"
        self.control_mode_publisher.publish(msg)
        self.get_logger().info("Published ackerman to /control_mode to activate Ackermann mode.")

    def gps_callback(self, msg):
        """
        Callback for /gps/fix subscriber.
        Updates the current GPS position.
        """
        self.current_position = msg
        self.get_logger().debug(f"Updated GPS Position: lat={msg.latitude}, lon={msg.longitude}")

    def imu_callback(self, msg):
        """
        Callback for /demo/imu subscriber.
        Extracts and updates the current yaw angle from IMU data.
        """
        self.current_yaw = msg.anglez
        self.get_logger().debug(f"Updated Yaw Angle: {math.degrees(self.current_yaw):.2f} degrees")

    def control_loop(self):
        """
        Main control loop running at 10 Hz.
        Determines movement commands based on current position and waypoints.
        """
        if self.current_position is None:
            self.get_logger().debug('Waiting for GPS data...')
            return  # No GPS data received yet

        if self.current_yaw is None:
            self.get_logger().debug('Waiting for IMU data...')
            return  # No IMU data received yet

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping robot.')
            self.stop_robot()
            return

        # Current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        target_lat = waypoint['latitude']
        target_lon = waypoint['longitude']
        # Optionally, handle altitude if needed
        # target_alt = waypoint.get('altitude', self.current_position.altitude)

        # Current position
        current_lat = self.current_position.latitude
        current_lon = self.current_position.longitude
        # current_alt = self.current_position.altitude  # Not used in 2D navigation

        # Calculate north and east distances to the target
        x, y = self.get_north_west_meters(current_lat, current_lon, target_lat, target_lon)

        distance = math.sqrt(x**2 + y**2)
        angle_to_target = math.atan2(y, x)  # Desired heading in radians

        if distance < self.waypoint_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}')
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.stop_robot()
                return
            self.stop_robot()
            next_waypoint = self.waypoints[self.current_waypoint_index]
            next_lat = next_waypoint['latitude']
            next_lon = next_waypoint['longitude']
            # Calculate north and east distances to the next target
            x, y = self.get_north_west_meters(current_lat, current_lon, next_lat, next_lon)
            self.get_logger().info('Next waypoint')
            self.get_logger().info(f'delta north = {x:.3f} meters')
            self.get_logger().info(f'delta west = {y:.3f} meters')
            # Pause briefly before moving to the next waypoint
            time.sleep(2)
        else:
            # Calculate angle error between desired angle and current yaw
            angle_error = angle_to_target - self.current_yaw

            # Normalize the angle error to [-pi, pi]
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            #self.get_logger().info(f'Angle Error: {math.degrees(angle_error):.2f} degrees')

            # Publish yaw error to /angle_error
            angle_msg = Twist()
            angle_msg.angular.x = self.current_yaw
            angle_msg.angular.y = angle_to_target
            angle_msg.angular.z = angle_error
            self.angle_publisher.publish(angle_msg)
            self.get_logger().debug('Published yaw error to /angle_error')

            # Publish linear velocity to /cmd_vel
            cmd_vel_msg = Twist()
            # Simple proportional control for linear velocity based on distance
            proportional_linear = self.proportional_gain * distance
            cmd_vel_msg.linear.x = max(0.3, min(self.max_linear_vel, proportional_linear))
            cmd_vel_msg.angular.z = 0.0  # No angular velocity; controller handles yaw
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            #self.get_logger().info(f'Moving forward: linear.x = {cmd_vel_msg.linear.x:.2f} m/s')

    def get_north_west_meters(self, start_lat, start_lon, target_lat, target_lon):
        """
        Calculates the distance in meters to travel north and west from the starting
        coordinates to reach the target coordinates.

        Parameters:
        - start_lat (float): Starting latitude in decimal degrees.
        - start_lon (float): Starting longitude in decimal degrees.
        - target_lat (float): Target latitude in decimal degrees.
        - target_lon (float): Target longitude in decimal degrees.

        Returns:
        - delta_north (float): Meters to travel north (positive) or south (negative).
        - delta_west (float): Meters to travel west (positive) or east (negative).
        """
        # Earth's radius in meters (WGS84 ellipsoid)
        R = 6378137

        # Convert degrees to radians
        lat1_rad = math.radians(start_lat)
        lat2_rad = math.radians(target_lat)
        lon1_rad = math.radians(start_lon)
        lon2_rad = math.radians(target_lon)

        # Differences in coordinates
        delta_lat_rad = lat2_rad - lat1_rad
        delta_lon_rad = lon2_rad - lon1_rad

        # Calculate mean latitude for scaling longitude
        mean_lat_rad = (lat1_rad + lat2_rad) / 2.0

        # Distance to travel north (meters)
        delta_north = delta_lat_rad * R

        # Distance to travel east (meters)
        delta_east = delta_lon_rad * R * math.cos(mean_lat_rad)

        # Distance to travel west is negative of east
        delta_west = -delta_east

        return delta_north, delta_west

    def stop_robot(self):
        """
        Stop the robot by setting all wheel velocities and steering angles to zero.
        """
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_cmd)
        #self.get_logger().info("Robot stopped.")

    def destroy_node(self):
        """
        Cleanup before shutting down the node.
        """
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavAckermanReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NavAckermanReal node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
