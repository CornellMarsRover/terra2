#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import math
import numpy as np

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')

        # ---- Kalman filter state initialization ----
        # State vector x = [north, west, yaw]
        self.x = np.zeros(3, dtype=float)

        # State covariance
        self.P = np.eye(3, dtype=float) * 1.0

        # Process noise (tune as appropriate)
        self.Q = np.diag([0.001, 0.001, 0.0005])

        # GPS measurement covariance for (north, west)
        # (Only using 2D from the 3D covariance; ignoring altitude)
        self.R_gps = np.diag([0.01, 0.01])

        # Flags/Storage
        self.initial_lat = 38.161479
        self.initial_lon = -122.454630
        self.last_zed_pose = None  # (north, west, yaw)
        self.last_zed_time = None  # For computing dt

        # ---- Subscribers ----
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/navsatfix',
            self.gps_callback,
            10
        )

        self.zed_sub = self.create_subscription(
            TwistStamped,
            '/zed/pose',
            self.zed_pose_callback,
            10
        )

        # ---- Publisher ----
        self.pose_pub = self.create_publisher(
            TwistStamped,
            '/autonomy/pose/robot/global',
            10
        )

        self.get_logger().info("Localization node has been started.")


    def gps_callback(self, msg: NavSatFix):
        """
        GPS callback. We use the first GPS fix to set the reference latitude/longitude.
        Afterward, each new GPS measurement is used to update the Kalman filter.
        """
        # If we haven't set the initial latitude/longitude, do so now
        if self.initial_lat is None or self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude
            self.get_logger().info(
                f"Initial GPS reference set to lat={self.initial_lat}, lon={self.initial_lon}"
            )
            return

        # If we haven't started receiving ZED data yet, there's no motion model update
        # so we can at least anchor our position estimate to GPS directly.
        # But we typically wait for a ZED update or do a direct measurement update:
        z_north, z_west = self.get_north_west_meters(msg.latitude, msg.longitude)

        # Perform a measurement update for the (north, west) portion.
        # 1) Measurement model: z = H x, where H = [[1, 0, 0],
        #                                           [0, 1, 0]]
        #    so predicted measurement = [x[0], x[1]]
        z = np.array([z_north, z_west])
        H = np.array([[1., 0., 0.],
                      [0., 1., 0.]])
        # 2) Innovation y = z - H x
        z_pred = H @ self.x
        y = z - z_pred
        # 3) Innovation covariance S = H P H^T + R
        S = H @ self.P @ H.T + self.R_gps
        # 4) Kalman gain K = P H^T S^-1
        K = self.P @ H.T @ np.linalg.inv(S)
        # 5) State update x = x + K y
        self.x = self.x + K @ y
        # 6) Covariance update P = (I - K H) P
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

        # Publish the updated estimate
        self.publish_estimated_pose(msg.header.stamp)

    def zed_pose_callback(self, msg: TwistStamped):
        """
        ZED pose callback. The message twist represents an estimated pose
        from the starting position:
          - linear.x = displacement North (m)
          - linear.y = displacement West (m)
          - angular.z = current yaw (radians)
        We'll use the change from the previous ZED message for the prediction.
        """
        current_time = msg.header.stamp
        current_pose = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z
        ])

        # If we have a previous ZED pose, compute the difference
        if self.last_zed_pose is not None and self.last_zed_time is not None:
            dt = self.compute_time_delta(current_time, self.last_zed_time)
            delta_pose = current_pose - self.last_zed_pose  # [dNorth, dWest, dYaw]

            # ---- Kalman Filter Prediction Step ----
            # x' = x + delta_pose
            self.x = self.x + delta_pose

            # P' = P + Q * dt  (We scale process noise by dt if desired)
            self.P = self.P + self.Q * max(dt, 1e-3)

            # Note: If you want a more sophisticated motion model,
            # you could incorporate velocity, etc.

            # At this point, we do *not* do the measurement update,
            # because GPS callback does that on its own schedule.

        # Store for next iteration
        self.last_zed_pose = current_pose
        self.last_zed_time = current_time

        # Optionally publish after prediction step (though typically
        # you'd wait until a measurement update or the next iteration).
        # Here we only publish after the GPS update in gps_callback,
        # but if you want a more frequent output, uncomment:
        # self.publish_estimated_pose(current_time)

    def compute_time_delta(self, current_stamp, last_stamp):
        """
        Compute time difference in seconds between two ROS 2 Time objects.
        """
        # In ROS 2, stamp.sec and stamp.nanosec are the fields
        dt = (current_stamp.sec - last_stamp.sec) + \
             (current_stamp.nanosec - last_stamp.nanosec) * 1e-9
        return dt

    def publish_estimated_pose(self, stamp):
        """
        Publish the current state estimate as a TwistStamped message:
          - linear.x = north displacement
          - linear.y = west displacement
          - angular.z = yaw
        """
        ts = TwistStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = 'map'  # or any global frame you'd like

        ts.twist.linear.x = float(self.x[0])   # north
        ts.twist.linear.y = float(self.x[1])   # west
        ts.twist.angular.z = float(self.x[2])  # yaw

        self.pose_pub.publish(ts)

    def get_north_west_meters(self, target_lat, target_lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        relative to the saved initial_lat, initial_lon.
        """
        R = 6378137.0
        lat1_rad = math.radians(target_lat)
        lat2_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(target_lon)
        lon2_rad = math.radians(self.initial_lon)
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        # Compute north/west distances
        north = -1.0 * delta_lat * R
        west = delta_lon * R * math.cos(mean_lat)
        return north, west


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
