# autonomous_navigation/waypoint_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
import time

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        # Declare parameters
        self.declare_parameter('waypoints_file', 'config/waypoints.yaml')
        self.declare_parameter('max_linear_vel', 0.4)  # m/s
        self.declare_parameter('max_angular_vel', 0.6)  # rad/s
        self.declare_parameter('waypoint_tolerance', 1.0)  # meters
        self.declare_parameter('angle_threshold_deg', 5.0)  # degrees

        # Get parameters
        waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        self.max_linear_vel = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        angle_threshold_deg = self.get_parameter('angle_threshold_deg').get_parameter_value().double_value
        self.angle_threshold_rad = math.radians(angle_threshold_deg)  # Convert to radians

        # Load waypoints
        self.waypoints = self.load_waypoints(waypoints_file)
        self.current_waypoint_index = 0

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Shutting down.')
            rclpy.shutdown()

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

        # Subscriber to GPS
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        # Subscriber to IMU
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/demo/imu',
            self.imu_callback,
            10
        )

        # Publisher for Twist
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Current position and orientation
        self.current_position = None
        self.current_yaw = None  # In radians

        # Bool for waiting in between turning and driving
        self.turned_last = False

        #other params
        self.max_dist = None
        self.threshold_adjustment = math.radians(10.0)

        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def load_waypoints(self, waypoints_file):
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
        self.current_position = msg

    def imu_callback(self, msg):
        # Extract orientation quaternion
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]  # Yaw is the third element

        # Store angular velocity magnitude
        av = msg.angular_velocity
        self.current_angular_velocity_magnitude = math.sqrt(av.x**2 + av.y**2 + av.z**2)

        # Store linear acceleration magnitude
        la = msg.linear_acceleration
        self.current_linear_acceleration_magnitude = math.sqrt(la.x**2 + la.y**2 + la.z**2)

    def control_loop(self):
        if self.current_position is None:
            self.get_logger().debug('Waiting for GPS data...')
            return  # No GPS data received yet

        if self.current_yaw is None:
            self.get_logger().debug('Waiting for IMU data...')
            return  # No IMU data received yet

        if self.current_angular_velocity_magnitude is None or self.current_linear_acceleration_magnitude is None:
            self.get_logger().debug('Waiting for IMU angular velocity and linear acceleration data...')
            return  # No angular velocity or linear acceleration data yet

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping rover.')
            twist = Twist()  # Stop the rover
            self.cmd_vel_publisher.publish(twist)
            return

        # Current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        target_lat = waypoint['latitude']
        target_lon = waypoint['longitude']
        target_alt = waypoint['altitude']

        # Current position
        current_lat = self.current_position.latitude
        current_lon = self.current_position.longitude
        current_alt = self.current_position.altitude

        x, y = self.get_north_west_meters(current_lat, current_lon, target_lat, target_lon)

        distance = math.sqrt(x**2 + y**2)
        temp = 0.0
        if not self.max_dist:
            self.max_dist = distance
        else:
            temp = math.ceil(self.threshold_adjustment * (distance/self.max_dist))

        angle_to_target = math.atan2(y, x)

        self.get_logger().info(f'x to target: {x:.5f}, y to target: {y:.5f}')

        twist = Twist()


        if distance < self.waypoint_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}')
            self.current_waypoint_index += 1
            twist = Twist()  # Stop the rover
            time.sleep(1)
            self.max_dist = None
        else:
            # Calculate angle error between desired angle and current yaw
            angle_error = angle_to_target - self.current_yaw

            # Normalize the angle error to [-pi, pi]
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            self.get_logger().info(f'Angle Error: {math.degrees(angle_error):.2f} degrees')

            # Determine if the angular error exceeds the threshold
            #if abs(angle_error) > (self.angle_threshold_rad + min(math.radians(10.0), math.radians(1.0) * (self.max_dist/distance))):
            if abs(angle_error) > self.angle_threshold_rad:
                # **Publish only angular velocity**

                if self.turned_last:
                    # Proportional controller for angular velocity
                    #angular_vel = -0.5 * angle_error
                    if angle_error < 0: angular_vel = 0.3
                    else: angular_vel = -0.3
                    # Clamp angular velocity
                    #angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

                    twist.linear.x = 0.0  # No linear movement
                    twist.angular.z = angular_vel  # Only rotate

                    self.get_logger().info(f'Publishing Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f} (Angular Correction)')
                    # implement wait if switching actions
                else:
                    self.turned_last = True
                    self.cmd_vel_publisher.publish(Twist())
                    self.get_logger().info('Pausing')
                    time.sleep(0.5)
            else:
                # **Publish only linear velocity**

                
                if not self.turned_last:

                    if distance < 6:
                        linear_vel = -0.15
                    else:
                        linear_vel = -0.29

                    # Clamp linear velocity
                    linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))

                    twist.linear.x = linear_vel  # Move forward
                    twist.angular.z = 0.0  # No rotation

                    self.get_logger().info(f'Publishing Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}')
                # implement wait if switching actions
                else:
                    self.turned_last = False
                    self.cmd_vel_publisher.publish(Twist())
                    time.sleep(0.5)

        self.cmd_vel_publisher.publish(twist)

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

def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
