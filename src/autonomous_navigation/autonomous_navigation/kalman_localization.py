#!/usr/bin/env python3
"""
localization_node.py

This ROS2 node implements a Kalman Filter (KF) to estimate the robot’s north-west displacement
by fusing data from a GPS sensor and a depth camera. The GPS message provides an absolute 
position (latitude/longitude) that is converted to north and west displacements relative to an 
initial coordinate. The depth camera message provides the change in pose between timesteps, 
which is used to compute the robot’s velocity.

Author: [Your Name]
Date: [Date]
"""

import rclpy
from rclpy.node import Node
import math
import numpy as np

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from cmr_msgs.msg import IMUSensorData


class KalmanLocalizationNode(Node):
    def __init__(self):
        super().__init__('kalman_localization_node')

        # ---------------------------
        # State Variables and Matrices
        # ---------------------------
        # State vector: [x, y]^T. Units: meters.
        self.x = np.zeros((2, 1))

        # State covariance (2x2). Tune the initial uncertainty as needed.
        self.P = np.eye(2) * 2.0

        # Process noise covariance (2x2). Adjusted dynamically from zed.
        self.Q = np.eye(2) * 0.1

        # GPS measurement noise covariance in meters (2x2). Tune these values.
        self.R_gps = np.diag([5.0, 5.0])

        # Default depth camera noise covariance (2x2) in case message covariance is not used.
        self.R_depth_default = np.diag([0.5, 0.5])

        # Initial GPS coordinates
        self.initial_lat = 42.4562060
        self.initial_lon = -76.4878320

        self.last_z_depth = np.array([[0.0], [0.0]])

        # Last update time (rclpy Time) for computing delta time dt.
        self.last_time = None

        # ---------------------------
        # Subscribers
        # ---------------------------
        # GPS subscriber:
        self.create_subscription(
            NavSatFix,
            '/navsatfix_data',
            self.gps_callback,
            10
        )

        # Depth camera subscriber:
        self.create_subscription(
            TwistWithCovarianceStamped,
            '/zed/pose',
            self.depth_callback,
            30
        )

        # IMU subscriber:
        self.create_subscription(
            IMUSensorData,
            '/imu',
            self.update_yaw,
            10
        )

        # Current direction rover is facing (0 deg is north)
        self.current_yaw = 0.0
        # IMU yaw angle when pointing north
        self.yaw_offset = 93.0

        # ---------------------------
        # Publisher
        # ---------------------------
        # Publish the filtered estimate (position and velocity) as Odometry.
        self.pub_estimate = self.create_publisher(TwistStamped, '/autonomy/pose/global/robot', 10)
        self.pose_timer = self.create_timer(0.1, self.publish_estimate)

        self.get_logger().info("Localization node has been started.")

    # --------------------------------------------------------------------------
    # Kalman Filter: Prediction Step
    # --------------------------------------------------------------------------
    def predict(self, dt):
        """
        Prediction step of the Kalman Filter using a constant-velocity motion model.
        The state transition matrix F is built using the time step dt.
        """
        # Build the state transition matrix F (4x4)
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1,  0],
                      [0, 0, 0,  1]])
        # Predict the state: x_k = F * x_(k-1)
        self.x = F.dot(self.x)
        # Predict the state covariance: P = F P F^T + Q
        self.P = F.dot(self.P).dot(F.T) + self.Q

    # --------------------------------------------------------------------------
    # Kalman Filter: Generic Update Step
    # --------------------------------------------------------------------------
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

    # --------------------------------------------------------------------------
    # GPS Callback: Processes NavSatFix Data
    # --------------------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        # On the first GPS message, set the initial GPS coordinate reference.
        if self.initial_lat is None or self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude
            self.get_logger().info(
                f"Initial GPS coordinates set: lat={self.initial_lat:.6f}, lon={self.initial_lon:.6f}"
            )
            # Do not update state until a subsequent measurement is received.
            return

        # No motion model so no need to compute dt
        '''# Compute time difference (dt) for prediction step.
        now = self.get_clock().now()
        if self.last_time is None:
            # If first update, use a default dt.
            dt = 0.1
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9  # seconds
        self.last_time = now

        # Execute the prediction step.
        if dt > 0:
            self.predict(dt)'''

        # Convert GPS latitude/longitude to north-west displacements.
        north, west = self.get_north_west_meters(msg.latitude, msg.longitude)
        # Create measurement vector for GPS (position measurement).
        z_gps = np.array([[north],
                          [west]])
        # Measurement matrix for GPS: only position is measured.
        H_gps = np.array([[1, 0],
                          [0, 1]])

        # Perform the measurement update using GPS data.
        self.update(z_gps, H_gps, self.R_gps)

    # --------------------------------------------------------------------------
    # Depth Camera Callback: Processes TwistWithCovarianceStamped Data
    # --------------------------------------------------------------------------
    def depth_callback(self, msg: TwistWithCovarianceStamped):
        '''now = self.get_clock().now()
        if self.last_time is None:
            dt = 0.1
        else:
            dt = (now - self.last_time).nanoseconds * 1e-9  # seconds
        self.last_time = now

        # Execute the prediction step.
        if dt > 0:
            self.predict(dt)

        # Build the measurement vector for depth (velocity measurement).
        z_depth = np.array([[msg.twist.twist.linear.x],
                            [msg.twist.twist.linear.y]])

        # Extract the 6x6 covariance from the message and pick out the linear parts.
        # The covariance is stored as a flat list (36 elements) in row-major order.
        cov = np.array(msg.twist.covariance).reshape((6, 6))
        # Extract the covariance for the linear components: indices (0,0), (0,1), (1,0), (1,1)
        R_depth = np.array([[cov[0, 0], cov[0, 1]],
                            [cov[1, 0], cov[1, 1]]])
        # Since the measurement is computed as (Δ/ dt), scale the covariance accordingly.
        if dt > 0:
            R_depth = R_depth / (dt ** 2)
        else:
            R_depth = self.R_depth_default

        # Measurement matrix for depth camera
        H_depth = np.array([[0, 0, 1, 0],
                            [0, 0, 0, 1]])

        # Perform the measurement update using depth camera data.
        self.update(z_depth, H_depth, R_depth)
        self.get_logger().debug(
            f"Depth update: dot_x={dot_x:.3f}, dot_y={dot_y:.3f}, dt={dt:.3f}"
        )    
        '''

        R = np.array([
            [np.cos(self.current_yaw), -np.sin(self.current_yaw)],
            [np.sin(self.current_yaw),  np.cos(self.current_yaw)]
        ])

        rotated_translation = R.dot(np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y]))
        # Build the measurement vector for depth (velocity measurement).
        z_depth = np.array([[rotated_translation[0]],
                            [rotated_translation[1]]])
        #z_depth = np.array([[msg.twist.twist.linear.x],
        #                    [msg.twist.twist.linear.y]])
        # Extract the 6x6 covariance from the message and pick out the linear parts.
        # The covariance is stored as a flat list (36 elements) in row-major order.
        cov = np.array(msg.twist.covariance).reshape((6, 6))
        # Extract the covariance for the linear components: indices (0,0), (0,1), (1,0), (1,1)
        R_depth = np.array([[cov[0, 0], cov[0, 1]],
                            [cov[1, 0], cov[1, 1]]])

        # Measurement matrix for depth camera
        H_depth = np.array([[1, 0],
                            [0, 1]])

        # Perform the measurement update using depth camera data.
        if np.max(self.x) > 10:
            self.update(self.x + (z_depth - self.last_z_depth), H_depth, R_depth)
        else:
            self.update(z_depth, H_depth, R_depth)
        self.last_z_depth = z_depth

    # --------------------------------------------------------------------------
    # Publish the Filtered Estimate
    # --------------------------------------------------------------------------
    def publish_estimate(self):
        """
        Publishes the current state estimate (position and velocity) as a nav_msgs/Odometry message.
        The position is stored in the pose field and the velocity in the twist field.
        """
        pose_msg = TwistStamped()
        now = self.get_clock().now().to_msg()
        pose_msg.header.stamp = now

        # Set velocity (linear component)
        pose_msg.twist.linear.x = float(self.x[0])
        pose_msg.twist.linear.y = float(self.x[1])
        pose_msg.twist.angular.z = float(math.degrees(self.current_yaw))
        self.pub_estimate.publish(pose_msg)

    # Update yaw with IMU message
    def update_yaw(self, msg):
        """
        Update yaw, currently just directly set it to z rotation from IMU
        """
        self.current_yaw = math.radians(msg.anglez - self.yaw_offset)

    # --------------------------------------------------------------------------
    # Helper Function: Convert GPS Coordinates to North-West Displacements
    # --------------------------------------------------------------------------
    def get_north_west_meters(self, target_lat, target_lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        relative to the saved initial_lat, initial_lon.
        """
        R = 6378137.0  # Earth's radius in meters
        # Convert degrees to radians
        lat1_rad = math.radians(target_lat)
        lat2_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(target_lon)
        lon2_rad = math.radians(self.initial_lon)
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        # Compute north and west displacements
        north = -1.0 * delta_lat * R
        west = delta_lon * R * math.cos(mean_lat)
        return north, west

def main(args=None):
    rclpy.init(args=args)
    node = KalmanLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Localization node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
