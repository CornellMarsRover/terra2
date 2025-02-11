#!/usr/bin/env python3
"""
localization_node.py

A ROS2 node that implements an Extended Kalman Filter (EKF) to estimate
the robot's north-west displacement relative to an initial GPS coordinate.
It subscribes to GPS, IMU, and depth camera pose data, fuses these sensor
readings, and publishes the filtered estimate.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

# Import message types
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped
from cmr_msgs.msg import IMUSensorData
from builtin_interfaces.msg import Time

class EKFLocalizationNode(Node):
    def __init__(self):
        super().__init__('ekf_localization_node')

        # --- Subscriptions ---
        # GPS data subscription
        self.create_subscription(
            NavSatFix,
            '/navsatfix_data',
            self.gps_callback,
            10)
        # IMU data subscription
        self.create_subscription(
            IMUSensorData,
            '/imu',
            self.imu_callback,
            10)
        # Depth camera pose subscription
        self.create_subscription(
            TwistWithCovarianceStamped,
            '/zed/pose',
            self.pose_callback,
            10)

        # --- Publisher ---
        # Publisher for the filtered pose estimate.
        self.filtered_pub = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)

        # --- Timer ---
        # Timer used to run the EKF prediction step at a fixed rate.
        self.timer_period = 0.1  # seconds
        #self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # --- EKF State Initialization ---
        # Adjust based on initial direction
        initial_yaw = 0.0
        # Angle reading of the IMU when facing north (degrees)
        self.imu_yaw_offset = 87.0
        # State vector: [north (m), west (m), yaw (rad), north_velocity (m/s), west_velocity (m/s), yaw_rate (rad/s)]
        self.x = np.zeros((6,1))
        self.x[2] = initial_yaw
        # Initial state covariance matrix P_k
        self.P = np.diag([2.0, 2.0, 0.005, 0.0, 0.0, 0.0])

        # Continuous time process noise covariance (3x3)
        self.Qc = np.diag([0.001, 0.001, 0.001])

        self.R_imu = np.diag([0.05, 0.001])

        # Measurement matrix for GPS (extracts position from state vector)
        self.H_gps = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]
        ])

        # ZED
        self.H_zed = np.eye(6)

        # IMU
        self.H_imu = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        # --- Time management ---
        self.last_time = None

        # --- Latest Sensor Measurements to save ---
        # Latest IMU accelerations [x_imu (m/s^2), y_imu (m/s^2), yaw_acc (rad/s^2)]^T
        self.u = None # Initialize after at least two IMU messages
        self.last_imu_time = None
        self.last_imu_yaw_rate = 0.0
        # State estimate after last zed update (to compute relative change)
        self.last_zed_state = np.diag([0.0, 0.0, initial_yaw, 0.0, 0.0, 0.0])
        self.last_zed_time = None
        # --- Initial GPS reference coordinate ---
        # These coordinates serve as the origin for computing displacement.
        self.initial_lat = 42.4561988
        self.initial_lon = -76.4878323

        # --- Outlier rejection ---
        # Threshold (in meters) to reject aberrant GPS measurements.
        self.gps_outlier_threshold = 50.0
        # Threshold (in meters) to reject aberrant ZED measurements.
        self.zed_outlier_threshold = 10.0
        # Thresholds to reject velocity estimates of state
        self.linear_velocity_threshold = 1.5 # m/s
        self.angular_velocity_threshold = 3.0 # rad/s

        self.get_logger().info("Localization node has been started.")

    def get_north_west_meters(self, target_lat, target_lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        relative to the saved initial_lat, initial_lon.
        """
        R = 6378137.0  # Earth radius in meters
        lat1_rad = math.radians(target_lat)
        lat2_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(target_lon)
        lon2_rad = math.radians(self.initial_lon)
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        # Compute north and west distances
        north = -1.0 * delta_lat * R
        west = delta_lon * R * math.cos(mean_lat)
        return north, west
    
    def time_difference(self, t1: Time, t2: Time) -> float:
        """Compute the difference between two ROS 2 timestamps in seconds."""
        sec_diff = t1.sec - t2.sec
        nanosec_diff = t1.nanosec - t2.nanosec
        return sec_diff + nanosec_diff * 1e-9

    def state_estimate_valid(self, x):
        """
        Helper function to verify if a state estimate is within the expected
        velocity thresholds
        """
        if x[0] < self.linear_velocity_threshold and x[1] < self.linear_velocity_threshold and x[2] < self.angular_velocity_threshold:
            return True
        return False

    def predict(self, dt):
        """
        Prediction step of the Kalman Filter using motion model.
        The state transition matrix F is built using the time step dt.
        """
        if self.u is None:
            return
        theta = self.x[2]
        ax = self.u[0]
        ay = self.u[1]
        # Build the jacobian matrix F (6x6)
        F = np.array([[1.0, 0.0, -0.5*(dt**2)*((ax*math.sin(theta))+ay*math.cos(theta)), dt, 0.0, 0.0],
                      [0.0, 1.0, 0.5*(dt**2)*((ax*math.cos(theta))-ay*math.sin(theta)), 0.0, dt, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, dt],
                      [1.0, 0.0, -dt*((ax*math.sin(theta))+ay*math.cos(theta)), 1.0, 0.0, 0.0],
                      [0.0, 1.0, dt*((ax*math.cos(theta))-ay*math.sin(theta)), 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
        
        F = np.eye(6)
        F[0, 2] = -0.5*(dt**2)*((ax*math.sin(theta))+ay*math.cos(theta))
        F[1, 2] = 0.5*(dt**2)*((ax*math.cos(theta))-ay*math.sin(theta))
        F[3, 2] = -dt*((ax*math.sin(theta))+ay*math.cos(theta))
        F[4, 2] = dt*((ax*math.cos(theta))-ay*math.sin(theta))
        F[0, 3], F[1, 4], F[2, 5] = dt, dt, dt
        
        A = np.eye(6)
        A[0, 3], A[1, 4], A[2, 5] = dt, dt, dt

        B = np.array([[0.5*(dt**2)*math.cos(theta), -0.5*(dt**2)*math.sin(theta), 0.0],
                      [0.5*(dt**2)*math.sin(theta), 0.5*(dt**2)*math.cos(theta), 0.0],
                      [0.0, 0.0, 0.5*(dt**2)],
                      [dt*math.cos(theta), -dt*math.sin(theta), 0.0],
                      [dt*math.sin(theta), dt*math.cos(theta), 0.0],
                      [0.0, 0.0, dt]])
        
        # Predict the state: x_k = A * x_(k-1) + B * u
        self.x = A.dot(self.x) + B.dot(self.u)
        
        # Propagate process noise
        Qk = B.dot(self.Qc).dot(B.T)

        # Predict the state covariance: P = F P F^T + Q
        self.P = F.dot(self.P).dot(F.T) + Qk

    def update(self, z, H, R):
        """
        Generic Kalman Filter measurement update.
        :param z: Measurement vector (as a column vector)
        :param H: Measurement matrix (maps state to measurement)
        :param R: Measurement noise covariance
        """
        # Compute Kalman gain: K = P H^T (H P H^T + R)^(-1)
        PHt = self.P.dot(H.T)
        S = H.dot(self.P).dot(H.T) + R
        K = PHt.dot(np.linalg.inv(S))

        # Update the state with the measurement z: x = x + K (z - H x)
        y = z - H.dot(self.x)  # innovation (residual)
        self.x = self.x + K.dot(y)

        # Update the covariance: P = (I - K H) P
        I = np.eye(self.P.shape[0])
        self.P = (I - K.dot(H)).dot(self.P)

    def gps_callback(self, msg):
        """
        Callback for GPS data. Converts latitude/longitude to north-west displacement,
        then applies a measurement update to the EKF.
        """
        self.get_logger().info("GPS")
        # Convert GPS coordinates to displacement in meters (north, west)
        north, west = self.get_north_west_meters(msg.latitude, msg.longitude)
        z = np.array([[north], [west]])
        R_gps = np.array([[msg.position_covariance[0], 0],
                          [0, msg.position_covariance[4]]])

        if self.last_time is None:
            self.last_time = msg.header.stamp
        else:
            dt = self.time_difference(msg.header.stamp, self.last_time)
            # Execute the prediction step.
            if dt > 0:
                self.predict(dt)
                self.last_time = msg.header.stamp

        self.update(z, self.H_gps, R_gps)
        self.get_logger().info(f"State: {self.x}")

    def imu_callback(self, msg):
        """
        Callback for IMU data, stores values for the prediction step.
        """
        # Linear accelerations
        ax = msg.accx
        ay = msg.accy

        # Angular velocity rad/s
        yaw_rate = msg.gyroz

        # Yaw rad
        yaw = math.radians(msg.anglez - self.imu_yaw_offset)

        if self.last_imu_time is None:
            self.last_imu_time = msg.header.stamp
            self.last_imu_yaw_rate = yaw_rate
        else:
            dt = self.time_difference(msg.header.stamp, self.last_imu_time)
            self.predict(dt)
            yaw_acc = yaw_rate-self.last_imu_yaw_rate/dt
            self.u = np.array([[ax], [ay], [yaw_acc]])
            self.last_imu_time = msg.header.stamp

        # State update
        z = np.array([[yaw], [yaw_rate]])
        self.update(z, self.H_imu, self.R_imu)
        self.get_logger().info(f"State: {self.x}")

    def pose_callback(self, msg):
        """
        Callback for depth camera pose data. Extracts the translation (assumed as
        displacement in north and west) and rotation (yaw), then uses these as a
        measurement update to the EKF.
        """
        # Extract measured displacements and yaw from the message.
        # (Assumes twist.twist.linear.x -> north, twist.twist.linear.y -> west,
        #  and twist.twist.angular.z -> yaw in radians.)
        meas_north = msg.twist.twist.linear.x
        meas_west  = msg.twist.twist.linear.y
        meas_yaw   = msg.twist.twist.angular.z

        # Extract the 6x6 covariance matrix (flattened in row-major order) and
        # pick the variances for x (north), y (west), and yaw.
        cov = np.array(msg.twist.covariance).reshape(6, 6)
        var_north = cov[0, 0] if cov[0, 0] > 0 else self.R_depth[0, 0]
        var_west  = cov[1, 1] if cov[1, 1] > 0 else self.R_depth[1, 1]
        var_yaw   = cov[5, 5] if cov[5, 5] > 0 else self.R_depth[2, 2]
        R_depth_dynamic = np.diag([var_north, var_west, var_yaw])

        # Build the measurement vector and corresponding measurement matrix.
        z = np.array([[meas_north], [meas_west], [meas_yaw]])
        H = np.array([
            [1, 0, 0, 0, 0],  # position north
            [0, 1, 0, 0, 0],  # position west
            [0, 0, 0, 0, 1]   # yaw
        ])

        # Innovation and Kalman Gain computation
        z_pred = np.dot(H, self.state)
        innovation = z - z_pred
        S = np.dot(H, np.dot(self.P, H.T)) + R_depth_dynamic
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # Update state and covariance matrix
        self.state = self.state + np.dot(K, innovation)
        I = np.eye(5)
        self.P = np.dot((I - np.dot(K, H)), self.P)

        self.get_logger().info("Depth camera update: state = {}".format(self.state.flatten()))


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
