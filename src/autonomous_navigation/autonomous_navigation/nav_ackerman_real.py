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
from typing import Dict, Tuple, Optional
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
import time
from cmr_msgs.msg import IMUSensorData, AutonomyDrive
from rclpy.callback_groups import ReentrantCallbackGroup


class PIDController:
    """
    Simple PID Controller
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        correcting: bool,
        output_limits: Tuple[Optional[float], Optional[float]] = (None, None),
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_steer_angle = 0.0
        self.max_d = 0.1
        self.output_limits = output_limits
        self.last_time: Optional[rclpy.time.Time] = None
        self.correcting = correcting

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = None

    def compute(self, error: float, current_time: rclpy.time.Time) -> float:
        """
        Compute the PID controller output.

        :param error: The current error value.
        :param current_time: The current time (rclpy Time object).
        :return: The control output.
        """

        if self.last_time is None:
            delta_time = 0.0
        else:
            delta_time = (current_time - self.last_time).nanoseconds * 1e-9

        self.last_time = current_time

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * delta_time
        I = self.Ki * self.integral

        # Derivative term
        derivative = (
            (error - self.previous_error) / delta_time if delta_time > 0 else 0.0
        )
        D = self.Kd * derivative

        # Save error for next derivative calculation
        self.previous_error = error

        # Compute the output
        output = P + I + D

        # Apply output limits
        lower, upper = self.output_limits
        lower = max(lower, self.previous_steer_angle - self.max_d)
        upper = min(upper, self.previous_steer_angle + self.max_d)
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
        self.previous_steer_angle = output
        
        return output
    
class NavAckermanReal(Node):
    def __init__(self):
        super().__init__('nav_ackerman_real')

        # Declare parameters
        self.declare_parameter('real', False)
        self.declare_parameter('max_linear_vel', 0.6)  # m/s
        self.declare_parameter('waypoint_tolerance', 2.0)  # meters
        self.declare_parameter('proportional_gain', 0.5)  # Proportional gain for linear velocity

        # Get parameters
        self.real = self.get_parameter('real').get_parameter_value().bool_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.proportional_gain = self.get_parameter('proportional_gain').get_parameter_value().double_value

        # FALSE IF TESTING IN SIM
        self.real = False

        if self.real:
            waypoints_file = 'config/waypoints_real.yaml'

        # Initialize PID controller for ackerman
        self.pid = PIDController(
            Kp=0.5,
            Ki=0.0,
            Kd=0.1,
            correcting=False,
            output_limits=(-math.radians(30), math.radians(30)),
        )
        self.get_logger().info("Initialized PID controller for ackerman control")
        
        # Offset to account for IMU offset from North = 0 deg heading
        self.imu_offset = math.radians(14.7)

        # Load waypoints
        self.waypoints = self.load_waypoints(waypoints_file)
        self.current_waypoint_index = 0

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Shutting down.')
            rclpy.shutdown()
            return

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

        # Subscriber to GPS
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            'navsatfixdata',
            self.gps_callback,
            10
        )
        self.get_logger().info("Subscribed to navsatfixdata")

        # Subscriber to IMU
        self.imu_subscriber = None
        if self.real:
            self.imu_subscriber = self.create_subscription(
                IMUSensorData,
                '/imu',
                self.real_imu_callback,
                10
            )
            self.get_logger().info("Subscribed to /imu")
        else:
            self.imu_subscriber = self.create_subscription(
                Imu,
                '/imu',
                self.sim_imu_callback,
                10
            )

        # Current position and orientation
        self.current_position = None
        self.current_yaw = None  # In radians

        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Autonomy Drive Publisher
        self.drive_publisher = self.create_publisher(AutonomyDrive, '/autonomy_move', 10)

        # Control Mode to determine what the robot is doing in the control loop
        self.current_action = "drive_to_coord"


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
    
    
    def gps_callback(self, msg):
        """
        Callback for navsatfixdata subscriber.
        Updates the current GPS position.
        """
        self.current_position = msg
        self.get_logger().debug(f"Updated GPS Position: lat={msg.latitude}, lon={msg.longitude}")
    

    def real_imu_callback(self, msg):
        """
        Callback for /imu subscriber for real robot.
        Extracts and updates the current yaw angle from IMU data.
        """
        self.current_yaw = math.radians(msg.anglez) - self.imu_offset
        self.get_logger().debug(f"Updated Yaw Angle: {math.degrees(self.current_yaw):.2f} degrees")

    def sim_imu_callback(self, msg):
        """
        Callback for /imu subscriber for sim.
        Extracts and updates the current yaw angle from IMU data.
        """
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
            return  # No GPS data received yet'''

        if self.current_yaw is None:
            self.get_logger().debug('Waiting for IMU data...')
            return  # No IMU data received yet

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping robot.')
            self.stop_robot()
            return
        
        # Which function to call based off what mode is
        # Once we implement other stuff (obstacle avoidance, AR tag detection, etc.)
        # Can switch control modes based off 
        if self.current_action == "drive_to_coord":
            self.drive_to_coord()



    def drive_to_coord(self):

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
        #self.get_logger.info(f"Angle to target{math.degrees(angle_to_target)}")
        if distance < self.waypoint_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}')
            self.current_waypoint_index += 1
            self.stop_robot()
            if self.current_waypoint_index >= len(self.waypoints):
                return

            # We would set the current action to the AR tag detection algo here
            # For example:
            # self.current_action == "detect_ar_tag"

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

            self.get_logger().info(f'Angle Error: {math.degrees(angle_error):.2f} degrees')

            # Compute PID controller output
            proportional_linear = self.proportional_gain * distance
            current_time = self.get_clock().now()
            steer_angle = math.degrees(self.pid.compute(angle_error, current_time))
            vel = max(0.3, min(self.max_linear_vel, proportional_linear))
            self.publish_ackerman(vel,steer_angle)


    def publish_ackerman(self, vel, steer_angle):
        
        drive_msg = AutonomyDrive()
        drive_msg.vel = vel
        drive_msg.fl_angle = -steer_angle
        drive_msg.fr_angle = -steer_angle
        drive_msg.bl_angle = 0.0
        drive_msg.br_angle = 0.0

        self.drive_publisher.publish(drive_msg)


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
        self.publish_ackerman(0.0,0.0)
        self.get_logger().info("Robot stopped.")

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


######## STOP COMMAND #########
# ros2 topic pub /autonomy_move cmr_msgs/msg/AutonomyDrive '{vel: 0.0, fl_angle: 0.0, fr_angle: 0.0, bl_angle: 0.0, br_angle: 0.0}'

