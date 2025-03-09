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


class RTKLocalizationNode(Node):
    def __init__(self):
        super().__init__('rtk_localization_node')


        # Initial GPS coordinates
        self.initial_lat = None
        self.initial_lon = None

        self.x = None
        self.y = None

        # Current direction rover is facing (0 deg is north)
        self.current_yaw = 0.0
        # IMU yaw angle when pointing north
        self.yaw_offset = 93.0

        # ---------------------------
        # Subscribers
        # ---------------------------
        # GPS subscriber:
        self.create_subscription(
            NavSatFix,
            '/rtk/navsatfix_data',
            self.gps_callback,
            10
        )
        # IMU subscriber:
        self.create_subscription(
            IMUSensorData,
            '/imu',
            self.update_yaw,
            10
        )

        # ---------------------------
        # Publisher
        # ---------------------------
        self.pose_estimate = self.create_publisher(TwistStamped, '/autonomy/pose/global/robot', 10)
        self.pose_timer = self.create_timer(0.1, self.publish_pose)

        self.get_logger().info("Localization node has been started.")

        self.max_d = 0.0
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
            # Do not update values until a subsequent measurement is received.
            return
        
        # Convert GPS latitude/longitude to north-west displacements.
        north, west = self.get_north_west_meters(msg.latitude, msg.longitude)
        self.x = north
        self.y = west
        d = math.sqrt((north**2) + (west**2))
        self.max_d = max(d, self.max_d)
        self.get_logger().info(f"Max deviation in meters: {self.max_d}")

    # --------------------------------------------------------------------------
    # Publish the pose
    # --------------------------------------------------------------------------
    def publish_pose(self):
        """
        Publish position and yaw
        """
        if self.x is None:
            return
        pose_msg = TwistStamped()
        now = self.get_clock().now().to_msg()
        pose_msg.header.stamp = now

        pose_msg.twist.linear.x = float(self.x)
        pose_msg.twist.linear.y = float(self.y)
        pose_msg.twist.angular.z = float(math.degrees(self.current_yaw))
        self.pose_estimate.publish(pose_msg)

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
    node = RTKLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Localization node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
