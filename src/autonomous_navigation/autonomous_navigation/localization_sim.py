#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped

from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion

import math
import numpy as np


class LocalizationSim(Node):
    def __init__(self):
        super().__init__('localization_sim')

        # Check if 'use_sim_time' is already set
        use_sim_time = self.get_parameter_or('use_sim_time', True).value
        self.get_logger().info(f"Simulation time enabled: {use_sim_time}")

        # State for the Kalman filter: [north_position, west_position, north_velocity, west_velocity]
        self.state = np.zeros(4)  # Initial state: [n, w, vn, vw]
        self.P = np.eye(4) * 0.1  # State covariance matrix

        # NOTE: not technically using process noise because only
        #       using sensor measurement, but whatever works for now in sim
        # Process and measurement noise
        self.Q = np.diag([0.01, 0.01, 0.1, 0.1])  # Process noise covariance
        self.R = np.diag([0.5, 0.5])  # Measurement noise covariance

        # Measurement matrix
        self.H = np.array([[1, 0, 0, 0],  # Position north
                           [0, 1, 0, 0]])  # Position west

        # State transition matrix (will be updated with dt)
        self.F = np.eye(4)

        # Control matrix (used for IMU data)
        self.B = np.zeros((4, 2))  # Updated dynamically

        # Initialize timestamps
        self.last_filter_time = None
        self.last_imu_time = None

        # Publishers
        # Pose in format [linear.x = d_north (m), linear.y = d_west(m), angular.z = yaw (rad)]
        self.pose_publisher = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        self.velocity_publisher = self.create_publisher(Float32MultiArray, '/autonomy/velocity', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(NavSatFix, '/gps_exact', self.gps_odom_callback, 10)
        self.create_subscription(NavSatFix, '/navsatfix', self.gps_map_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Timer for running the Kalman filter
        self.kalman_timer = self.create_timer(1.0 / 10.0, self.run_kalman_filter)  # 10 Hz

        # Data from sensors
        self.gps_measurement = None
        self.imu_delta_position = [0.0, 0.0]
        self.imu_velocity = [0.0, 0.0, 0.0]  # [v_north, v_west, omega_z]
        self.yaw = 0.0 # Current yaw in radians

        # Initialize GPS reference coordinates
        self.initial_lat = None
        self.initial_lon = None

    def gps_map_callback(self, msg):
        """
        Updates map position using GPS data from /navsatfix.
        """
        if self.initial_lat is None or self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude

        self.current_map_position = self.get_north_west_meters(
            self.initial_lat, self.initial_lon, msg.latitude, msg.longitude
        )

    def gps_odom_callback(self, msg):
        """
        Updates the GPS measurement for the Kalman filter.
        """
        if self.initial_lat is None or self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude

        north, west = self.get_north_west_meters(
            self.initial_lat, self.initial_lon, msg.latitude, msg.longitude
        )
        self.gps_measurement = np.array([north, west])

    def imu_callback(self, msg):
        """
        Updates IMU-based velocity and delta position for the Kalman filter.
        """
        q = msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        last_yaw = self.yaw
        self.yaw = euler[2]

        if self.last_imu_time is None:
            self.last_imu_time = self.get_clock().now()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_imu_time).nanoseconds * 1e-9  # Convert to seconds
        self.last_imu_time = current_time
        if dt == 0:
            return

        # Update angular velocity
        omega_z = (self.yaw - last_yaw)/dt
        self.imu_velocity[2] = omega_z

        linear_accel_x = msg.linear_acceleration.x
        linear_accel_y = msg.linear_acceleration.y

        # Transform accelerations to global frame
        north_accel, west_accel = self.transform_to_global(linear_accel_x, linear_accel_y, self.yaw)

        # Update velocities
        self.imu_velocity[0] += north_accel * dt  # v_north
        self.imu_velocity[1] += west_accel * dt  # v_west

        # Calculate delta position
        self.imu_delta_position[0] = self.imu_velocity[0] * dt  # delta_north
        self.imu_delta_position[1] = self.imu_velocity[1] * dt  # delta_west

    def run_kalman_filter(self):
        """
        Runs the Kalman filter to fuse GPS and IMU data.
        """
        if self.last_filter_time is None:
            self.last_filter_time = self.get_clock().now()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_filter_time).nanoseconds * 1e-9
        self.last_filter_time = current_time

        # Update state transition matrix
        self.F = np.array([
            [1, 0, dt, 0],  # North position
            [0, 1, 0, dt],  # West position
            [0, 0, 1, 0],   # North velocity
            [0, 0, 0, 1]    # West velocity
        ])

        # Update control matrix
        self.B = np.array([
            [0.5 * dt**2, 0],  # North position
            [0, 0.5 * dt**2],  # West position
            [dt, 0],           # North velocity
            [0, dt]            # West velocity
        ])

        # Prediction step
        u = np.array(self.imu_delta_position)  # IMU delta position as control input
        self.state = np.dot(self.F, self.state) + np.dot(self.B, u)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

        # Measurement update step (if GPS measurement is available)
        if self.gps_measurement is not None:
            y = self.gps_measurement - np.dot(self.H, self.state)  # Innovation
            S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R  # Innovation covariance
            K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))  # Kalman gain
            self.state += np.dot(K, y)  # State update
            self.P = np.dot(np.eye(4) - np.dot(K, self.H), self.P)  # Covariance update

        # Publish the updated state
        self.publish_state()

    def publish_state(self):
        """
        Publishes the fused pose and velocity.
        """
        pose_msg = TwistStamped()
        pose_msg.twist.linear.x = self.state[0]
        pose_msg.twist.linear.y = self.state[1]
        pose_msg.twist.angular.z = self.yaw
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Get current time
        self.pose_publisher.publish(pose_msg)

        velocity_msg = Float32MultiArray()
        velocity_msg.data = [self.state[2], self.state[3], self.imu_velocity[2]]  # v_north, v_west, omega_z
        self.velocity_publisher.publish(velocity_msg)

    def transform_to_global(self, accel_x, accel_y, yaw):
        """
        Transforms local IMU linear accelerations to global north and west accelerations.
        """
        rotation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])
        accel_global = np.dot(rotation_matrix, np.array([accel_x, accel_y]))
        return accel_global[0], accel_global[1]

    def get_north_west_meters(self, lat1, lon1, lat2, lon2):
        """
        Converts latitude and longitude to north and west displacements in meters.
        """
        R = 6378137.0
        lat1_rad, lat2_rad = map(math.radians, [lat1, lat2])
        lon1_rad, lon2_rad = map(math.radians, [lon1, lon2])

        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        delta_north = delta_lat * R
        delta_east = delta_lon * R * math.cos(mean_lat)
        return delta_north, -delta_east  # West is negative east

    def quaternion_from_yaw(self, yaw):
        """
        Converts yaw angle to a quaternion.
        """
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    '''
    def publish_odom_to_base_link_transform(self):
        """
        Publishes the odom -> base_link transform based on odometry data.
        """
        delta_north = self.current_odom_position[0] - self.last_odom_position[0]
        delta_west = self.current_odom_position[1] - self.last_odom_position[1]
        delta_yaw = self.current_odom_yaw - self.last_odom_yaw

        # Update last position
        self.last_odom_position = self.current_odom_position

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = delta_north
        t.transform.translation.y = delta_west
        t.transform.translation.z = 0.0

        q = self.quaternion_from_yaw(delta_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_map_to_odom_transform(self):
        """
        Publishes the map -> odom transform based on localization data.
        """
        delta_north = self.current_map_position[0] - self.last_map_position[0]
        delta_west = self.current_map_position[1] - self.last_map_position[1]
        delta_yaw = self.current_map_yaw - self.last_map_yaw

        # Update last position
        self.last_map_position = self.current_map_position

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = delta_north
        t.transform.translation.y = delta_west
        t.transform.translation.z = 0.0

        q = self.quaternion_from_yaw(delta_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)
    '''

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
