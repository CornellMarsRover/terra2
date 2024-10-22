#!/usr/bin/env python3

"""
NavControlMethodsDemo ROS2 Node

This node autonomously navigates the robot to predefined GPS waypoints using different control methods.
It switches between control modes ("ackerman", "swerve", "direct") for each new GPS waypoint.

For Ackermann control:
- Implements the same algorithm as the previous Ackermann navigation node.

For Swerve control:
- Goes forward until the yaw of the robot is more than 10 degrees different than the desired yaw.
- When this happens, executes a point turn until the robot is within 5 degrees of the desired yaw.
- Includes a 0.25 second wait between turning and going forward actions.

For Direct control:
- Points the angles of the wheels in the desired yaw direction and applies a velocity to the wheels.

All movements are bounded by:
- Maximum linear velocity of 0.5 m/s.
- Maximum angular velocity of 0.5 rad/s.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
import time


class NavControlMethodsDemo(Node):
    def __init__(self):
        super().__init__('nav_control_methods_demo')

        # Declare parameters
        self.declare_parameter('waypoints_file', 'config/waypoints.yaml')
        self.declare_parameter('max_angular_vel', 0.5)  # meters
        self.declare_parameter('max_linear_vel', 0.5)  # Proportional gain for linear velocity
        self.declare_parameter('waypoint_tolerance', 2.0)  # meters
        self.declare_parameter('proportional_gain', 0.5)  # Proportional gain for linear velocity

        # Get parameters
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
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

        # Wheel radius pulled from robot_geometry file from swerve_controller
        self.wheel_radius = 0.1397

        # Variable angle threshold for point turns with swerve control
        self.angle_threshold = 10.0

        # Publisher to /control_mode to set control mode
        self.control_mode_publisher = self.create_publisher(String, '/control_mode', 10)

        # Publisher for yaw error (used in Ackermann mode)
        self.angle_error_publisher = self.create_publisher(Twist, '/angle_error', 10)
        self.get_logger().info("Publishing to /angle_error")

        # Publisher for linear velocity
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Publishing to /cmd_vel")

        # Publisher for direct control commands (used in Direct mode)
        self.direct_control_publisher = self.create_publisher(Float32MultiArray, '/direct_control', 10)
        self.get_logger().info("Publishing to /direct_control")

        # Subscriber to GPS
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )
        self.get_logger().info("Subscribed to /gps/fix")

        # Subscriber to IMU
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/demo/imu',
            self.imu_callback,
            10
        )
        self.get_logger().info("Subscribed to /demo/imu")

        # Current position and orientation
        self.current_position = None
        self.current_yaw = None  # In radians

        # Control mode
        self.control_modes = ['swerve', 'ackerman']
        self.current_control_mode_index = 0
        self.set_control_mode(self.control_modes[self.current_control_mode_index])

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

    def set_control_mode(self, mode):
        """
        Sets the control mode by publishing to /control_mode.
        """
        msg = String()
        msg.data = mode
        self.control_mode_publisher.publish(msg)
        self.get_logger().info(f"Control mode set to {mode}")

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
        # Extract orientation quaternion
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]  # Yaw is the third element
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

        # Current position
        current_lat = self.current_position.latitude
        current_lon = self.current_position.longitude

        # Calculate north and east distances to the target
        x, y = self.get_north_west_meters(current_lat, current_lon, target_lat, target_lon)

        distance = math.sqrt(x**2 + y**2)
        angle_to_target = math.atan2(y, x)  # Desired heading in radians

        if distance < self.waypoint_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}')
            self.current_waypoint_index += 1
            self.stop_robot()
            time.sleep(5)  # Pause briefly before moving to the next waypoint

            # Switch to the next control mode
            self.current_control_mode_index = (self.current_control_mode_index + 1) % len(self.control_modes)
            self.set_control_mode(self.control_modes[self.current_control_mode_index])
            return

        # Calculate angle error between desired angle and current yaw
        angle_error = angle_to_target - self.current_yaw

        # Normalize the angle error to [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Convert angle error to degrees for comparisons
        angle_error_deg = math.degrees(angle_error)

        # Determine behavior based on control mode
        current_mode = self.control_modes[self.current_control_mode_index]

        if current_mode == 'ackerman':
            self.run_ackerman_mode(distance, angle_to_target, angle_error)
        elif current_mode == 'swerve':
            self.run_swerve_mode(distance, angle_to_target, angle_error, angle_error_deg)
        elif current_mode == 'direct':
            self.run_direct_mode(distance, angle_to_target, angle_error)
        else:
            self.get_logger().error(f"Unknown control mode: {current_mode}")

    def run_ackerman_mode(self, distance, angle_to_target, angle_error):
        """
        Implements Ackermann control algorithm.
        """
        # Publish yaw error to /angle_error
        angle_msg = Twist()
        angle_msg.angular.x = angle_to_target  # Desired yaw
        angle_msg.angular.y = self.current_yaw  # Current yaw
        angle_msg.angular.z = angle_error      # Yaw error
        self.angle_error_publisher.publish(angle_msg)
        self.get_logger().debug('Published yaw error to /angle_error')

        # Publish linear velocity to /cmd_vel
        cmd_vel_msg = Twist()
        # Simple proportional control for linear velocity based on distance
        proportional_linear = self.proportional_gain * distance
        cmd_vel_msg.linear.x = max(0.3, min(self.max_linear_vel, proportional_linear))  # Max linear velocity is 0.5
        cmd_vel_msg.angular.z = 0.0  # No angular velocity; controller handles yaw
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Ackermann mode: Moving forward at {cmd_vel_msg.linear.x:.2f} m/s')

    def run_swerve_mode(self, distance, angle_to_target, angle_error, angle_error_deg):
        """
        Implements Swerve control algorithm.
        Goes forward until the yaw of the robot is more than 10 degrees different than the desired yaw.
        When this happens, executes a point turn until the robot is within 5 degrees of the desired yaw.
        Includes a 0.25 second wait between turning and going forward actions.
        """
        cmd_vel_msg = Twist()

        if abs(angle_error_deg) <= self.angle_threshold:
            if self.angle_threshold == 5.0:
                self.stop_robot()
                self.angle_threshold = 10.0
                time.sleep(0.25)
            # Move forward
            cmd_vel_msg.linear.x = self.max_linear_vel  # Max linear velocity is 0.5
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.get_logger().info(f'Swerve mode: Moving forward at {cmd_vel_msg.linear.x:.2f} m/s')
        else:
            if self.angle_threshold == 10.0:
                self.stop_robot()
                self.angle_threshold = 5.0
                time.sleep(0.25)

            # Execute point turn
            cmd_vel_msg.linear.x = 0.0
            # Turn in the direction of the angle error
            cmd_vel_msg.angular.z = self.max_angular_vel if angle_error > 0 else -self.max_angular_vel  # Max angular velocity is 0.5
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.get_logger().info(f'Swerve mode: Turning at {cmd_vel_msg.angular.z:.2f} rad/s')
            

    def run_direct_mode(self, distance, angle_to_target, angle_error):
        """
        Implements Direct control algorithm.
        Points the angles of the wheels in the desired yaw direction and applies a velocity to the wheels.
        """
        # Calculate the steering angle (desired yaw)
        steering_angle = angle_to_target - self.current_yaw

        # Ensure the steering angle is within [-pi/2, pi/2]
        if steering_angle > math.pi/2:
            steering_angle -= math.pi
        elif steering_angle < -math.pi/2:
            steering_angle += math.pi

        # Prepare the direct control message
        direct_msg = Float32MultiArray()
        # Steering angles for all wheels (front_right, front_left, rear_right, rear_left)
        steering_angles = [steering_angle, steering_angle, steering_angle, steering_angle]
        # Wheel velocities for all wheels (front_right, front_left, rear_right, rear_left)
        wheel_velocity = self.max_linear_vel/self.wheel_radius  # Max linear velocity is 0.5 m/s
        wheel_velocities = [wheel_velocity, wheel_velocity, wheel_velocity, wheel_velocity]
        # Combine into one list
        direct_msg.data = steering_angles + wheel_velocities

        # Publish direct control command
        self.direct_control_publisher.publish(direct_msg)
        self.get_logger().info(f'Direct mode: Steering angle {math.degrees(steering_angle):.2f} degrees, wheel velocity {wheel_velocity:.2f} m/s')

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

        self.set_control_mode("ackerman")

        # Stop cmd_vel
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_cmd)

        self.get_logger().info("Robot stopped.")

    def destroy_node(self):
        """
        Cleanup before shutting down the node.
        """
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavControlMethodsDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NavControlMethodsDemo node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
