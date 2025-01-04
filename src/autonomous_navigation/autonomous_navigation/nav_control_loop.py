#!/usr/bin/env python3

"""
NavAckerman ROS2 Node with Kalman Filter for Displacement Estimation

This node autonomously navigates the robot to predefined GPS waypoints using Ackermann steering,
and maintains a Kalman filter to fuse GPS-based positions with IMU-based accelerations, producing
a continuous estimate of displacement in the north/west directions relative to the starting GPS coordinate.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, Int16, Float32MultiArray
import yaml
import math
import os
import time
from typing import Dict, Tuple, Optional
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion
from cmr_msgs.msg import IMUSensorData, AutonomyDrive
import numpy as np

# Kalman filter class for updating guess of north & west displacement based on noisy GPS and IMU measurements
class KalmanFilter2D:
    def __init__(self):
        # State vector: [p_n, p_w, v_n, v_w]^T
        self.x = np.zeros((4, 1), dtype=float)

        # State covariance
        self.P = np.eye(4) * 1.0   # You can tune these initial uncertainties

        # Process noise covariance
        # Adjust these for your IMU noise or other sources of uncertainty
        self.Q = np.eye(4) * 0.1

        # Measurement noise covariance
        # For GPS measurement of position (p_n, p_w)
        self.R = np.eye(2) * 5.0

        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],  # measure p_n
            [0, 1, 0, 0]   # measure p_w
        ], dtype=float)

        # Last update time for delta_t
        self.last_time = None

    def predict(self, dt: float, a_n: float, a_w: float):
        """
        Predict step of the Kalman Filter with 2D position/velocity.
        a_n, a_w are the accelerations in the north/west directions.
        """
        # State transition matrix F
        F = np.array([
            [1, 0, dt, 0 ],
            [0, 1, 0,  dt],
            [0, 0, 1,  0 ],
            [0, 0, 0,  1 ]
        ], dtype=float)

        # Control matrix G for acceleration
        G = np.array([
            [0.5*(dt**2), 0          ],
            [0,           0.5*(dt**2)],
            [dt,          0          ],
            [0,           dt         ]
        ], dtype=float)

        # Construct acceleration vector
        a = np.array([[a_n], [a_w]], dtype=float)

        # Predict next state: x = F*x + G*a
        self.x = F @ self.x + G @ a

        # Predict next covariance: P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q

    def update(self, p_n_meas: float, p_w_meas: float):
        """
        Update step with GPS measurement of (p_n, p_w).
        """
        # Measurement vector z
        z = np.array([[p_n_meas], [p_w_meas]], dtype=float)

        # Residual y = z - H*x
        y = z - (self.H @ self.x)

        # S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state: x = x + K*y
        self.x = self.x + K @ y

        # Update covariance: P = (I - K*H) * P
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

    def get_position(self):
        """
        Returns the current estimated position (north, west) in meters.
        """
        p_n = self.x[0, 0]
        p_w = self.x[1, 0]
        return p_n, p_w

    def get_velocity(self):
        """
        Returns the current estimated velocity (north, west) in meters per second.
        """
        v_n = self.x[2, 0]
        v_w = self.x[3, 0]
        return v_n, v_w

    def get_speed(self):
        """
        Returns the current estimated speed (magnitude of velocity) in meters per second.
        """
        v_n, v_w = self.get_velocity()
        speed = np.sqrt(v_n**2 + v_w**2)
        return speed


class NavigationControlLoop(Node):
    def __init__(self):
        super().__init__('nav_control_loop')

        # Declare parameters
        self.declare_parameter('real', False) # FALSE IF TESTING IN SIM
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


        self.kf = KalmanFilter2D()
        # We will store a separate variable for the initial GPS lat/lon to compute displacement
        self.initial_lat = None
        self.initial_lon = None
        # We'll also store final "estimated" displacement
        self.estimated_north = 0.0
        self.estimated_west  = 0.0

        # Load waypoints
        self.waypoints = self.load_waypoints(waypoints_file)
        self.current_waypoint_index = 0

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Shutting down.')
            rclpy.shutdown()
            return

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')

        # Create Subscribers for IMU and GPS
        # Real robot uses /imu => IMUSensorData, navsatfixdata => NavSatFix
        # Sim robot uses /imu => Imu, /navsatfix => NavSatFix
        # IMU offset from North
        self.imu_offset = 0.0
        if self.real:
            self.imu_subscriber = self.create_subscription(
                IMUSensorData,
                '/imu',
                self.real_imu_callback,
                10
            )
            self.imu_offset = math.radians(14.7)
            self.gps_subscriber = self.create_subscription(
                NavSatFix,
                'navsatfixdata',
                self.gps_callback,
                10
            )
        else:
            self.imu_subscriber = self.create_subscription(
                Imu,
                '/imu',
                self.sim_imu_callback,
                10
            )
            self.gps_subscriber = self.create_subscription(
                NavSatFix,
                '/navsatfix',
                self.gps_callback,
                10
            )

        self.get_logger().info("Subscribed to IMU and GPS topics.")

        # Current raw messages
        self.current_position = None
        self.current_yaw = None  # In radians
        # Keep track of time for IMU-based dt
        self.last_imu_time = None

        # Timer for publishing commands (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Pose offset from next coordinate
        self.coordinate_offset_publisher = self.create_publisher(Float32MultiArray, '/autonomy/coordinate_error', 10)
        
        # Publishes True if rover has reached a waypoint
        self.waypoint_reached_publisher = self.create_publisher(Bool, '/autonomy/waypoint_reached', 10)

        # Current GPS waypoint we are at, -1 if not at any
        self.waypoint_index_publisher = self.create_publisher(Int16, '/autonomy/waypoint_index', 10)
        # Pose
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/autonomy/pose', 10)
        # Velocity
        self.velocity_publisher = self.create_publisher(Float32MultiArray, '/autonomy/velocity', 10)
        # Control Mode for the robot state machine
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

    def publish_pose(self):
        '''
        Publishes current pose in the following format
        [displacement_north (m), displacement_west (m), rotation about vertical axis (rad)]
        '''
        pose = [float(self.estimated_north), float(self.estimated_west), float(self.current_yaw)]
        pose_msg = Float32MultiArray()
        pose_msg.data = pose
        self.pose_publisher.publish(pose_msg)

    def publish_velocity(self):
        '''
        Publishes current velocity in the following format
        [velocity_north (m/s), velocity_west (m/s), robot speed (m/s)]
        '''
        vn, vw = self.kf.get_velocity()
        s = self.kf.get_speed()
        velocity = [float(vn), float(vw), float(s)]
        velocity_msg = Float32MultiArray()
        velocity_msg.data = velocity
        self.velocity_publisher.publish(velocity_msg)

    # ------------------------------------------------------------
    # GPS Callback
    # ------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        """
        Callback for GPS subscriber.
        Updates the current GPS position and performs a Kalman update with the
        measured (north, west) displacement if possible.
        """
        self.current_position = msg

        # If first time receiving GPS, store as reference
        if self.initial_lat is None or self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude
            self.get_logger().info("Set initial GPS reference.")

        # Calculate north/west from initial
        meas_north, meas_west = self.get_north_west_meters(
            self.initial_lat, self.initial_lon,
            msg.latitude, msg.longitude
        )

        # Kalman update step
        # If we haven't predicted at all yet (no IMU callback?), you can still do an update
        self.kf.update(meas_north, meas_west)

        # Store the updated estimate in local variables
        self.estimated_north = float(self.kf.x[0])  # p_n
        self.estimated_west  = float(self.kf.x[1])  # p_w

        '''self.get_logger().debug(
            f"GPS Callback - Measured N={meas_north:.2f}, W={meas_west:.2f}; "
            f"Estimated N={self.estimated_north:.2f}, W={self.estimated_west:.2f}"
        )'''

    # ------------------------------------------------------------
    # IMU Callbacks
    # ------------------------------------------------------------
    def real_imu_callback(self, msg: IMUSensorData):
        """
        Callback for /imu subscriber on real robot. 
        msg.anglez is presumably heading in degrees, plus we have linear accelerations if available.
        """
        # Yaw
        self.current_yaw = math.radians(msg.anglez) - self.imu_offset

        # For the Kalman filter, we need acceleration in north/west directions.
        # Suppose IMUSensorData has linear_acceleration x, y in the robot frame.
        # A simple approach is to assume x ~ forward, y ~ left, and that the robot heading
        # is self.current_yaw w.r.t. north. Then rotate the accelerations into the
        # global north/west frame. (You can refine this logic if your IMU data is oriented differently.)
        ax_robot = msg.accx
        ay_robot = msg.accy

        # Rotate from robot frame to 'north/west' world frame
        # Robot heading 0 means facing north. 
        # The rotation matrix for heading θ is:
        # [ cosθ, -sinθ ]
        # [ sinθ,  cosθ ]
        # But we must be mindful that "west" is +y in our chosen coordinate if we treat x = north, y = west.
        # Let θ = -current_yaw for rotating from robot to inertial if the robot frame x-forward is heading.
        # This can vary depending on your coordinate convention, but let's assume:
        #   inertial_n =  cos(θ)*ax_robot - sin(θ)*ay_robot
        #   inertial_w =  sin(θ)*ax_robot + cos(θ)*ay_robot
        # Adjust sign if needed to match your 'west' axis as positive left or right.

        theta = - self.current_yaw
        a_n = math.cos(theta)*ax_robot - math.sin(theta)*ay_robot
        a_w = math.sin(theta)*ax_robot + math.cos(theta)*ay_robot

        self.perform_kf_predict(a_n, a_w)

    def sim_imu_callback(self, msg: Imu):
        """
        Callback for /imu subscriber in simulation. 
        We can read orientation from the quaternion, plus linear_acceleration.x and .y
        in the IMU's frame. Then rotate them to the north/west frame as above.
        """
        # Extract yaw
        q = msg.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        euler = euler_from_quaternion(quaternion)
        self.current_yaw = euler[2]  # Yaw is the third element

        # Extract linear accelerations in robot frame
        ax_robot = msg.linear_acceleration.x
        ay_robot = msg.linear_acceleration.y

        # Rotate into north/west frame
        theta = - self.current_yaw
        a_n = math.cos(theta)*ax_robot - math.sin(theta)*ay_robot
        a_w = math.sin(theta)*ax_robot + math.cos(theta)*ay_robot

        self.perform_kf_predict(a_n, a_w)

    def perform_kf_predict(self, a_n, a_w):
        """
        Perform the Kalman filter prediction step given accelerations in the
        north/west reference frame and the time since last IMU update.
        """
        current_time = self.get_clock().now()
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds * 1e-9
        else:
            dt = 0.01  # small default for first time

        self.last_imu_time = current_time

        # Predict with dt and measured accelerations
        self.kf.predict(dt, a_n, a_w)

        # Update local displacement variables
        self.estimated_north = float(self.kf.x[0])
        self.estimated_west  = float(self.kf.x[1])

        self.get_logger().debug(
            f"IMU Callback - Predicted with a_n={a_n:.2f}, a_w={a_w:.2f}, dt={dt:.3f}; "
            f"Est N={self.estimated_north:.2f}, W={self.estimated_west:.2f}"
        )

    # ------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------
    def control_loop(self):
        """
        Main control loop running at 10 Hz.
        Determines movement commands based on current position and waypoints.
        """
        # If no GPS or IMU data yet, wait
        if self.current_position is None:
            self.get_logger().debug('Waiting for GPS data...')
            return
        if self.current_yaw is None:
            self.get_logger().debug('Waiting for IMU data...')
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached. Stopping robot.')
            return

        # Currently drive to coordinate
        if self.current_action == "drive_to_coord":
            self.drive_to_coord()

        self.publish_pose()
        self.publish_velocity()

    def drive_to_coord(self):

        #waypoint_msg = Int16()
        if self.current_waypoint_index >= len(self.waypoints):
            return
        # Current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        target_lat = waypoint['latitude']
        target_lon = waypoint['longitude']

        # Current lat/lon
        current_lat = self.current_position.latitude
        current_lon = self.current_position.longitude

        # Convert to local displacement in the north/west directions
        x, y = self.get_north_west_meters(current_lat, current_lon, target_lat, target_lon)
        distance = math.sqrt(x**2 + y**2)
        angle_to_target = math.atan2(y, x)  # desired heading in radians

        # Calculate angle error
        angle_error = angle_to_target - self.current_yaw
        # Normalize angle error to [-pi, pi]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # Check if within distance within tolerance
        if distance < self.waypoint_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}')
            #waypoint_msg.data = int(self.current_waypoint_index)
            #self.waypoint_index_publisher.publish(waypoint_msg)
            waypoint_reached = Bool()
            waypoint_reached.data = True
            self.waypoint_reached_publisher.publish(waypoint_reached)
            self.current_waypoint_index += 1
            time.sleep(2)
        else:
            #waypoint_msg.data = int(-1)
            #self.waypoint_index_publisher.publish(waypoint_msg)
            self.publish_gps_offset(x, y, angle_error)


    def publish_gps_offset(self, north, west, yaw):
        """
        Publish the offset to the next gps coordinate and the steer angle error
        in the following format: [d_north (m), d_west (m), angle_error (rad)]
        """
        offsets = [float(north), float(west), float(yaw)]
        offset_msg = Float32MultiArray()
        offset_msg.data = offsets
        self.coordinate_offset_publisher.publish(offset_msg)

    def get_north_west_meters(self, start_lat, start_lon, target_lat, target_lon):
        """
        Calculates the distance in meters to travel north and west from the starting
        coordinates to reach the target coordinates.
        """
        # Earth's radius in meters (WGS84 ellipsoid)
        R = 6378137.0

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
        # West is negative of east
        delta_west = -delta_east

        return (delta_north, delta_west)

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



######## STOP COMMAND #########
# ros2 topic pub /autonomy_move cmr_msgs/msg/AutonomyDrive '{vel: 0.0, fl_angle: 0.0, fr_angle: 0.0, bl_angle: 0.0, br_angle: 0.0}'

