#!/usr/bin/env python3
"""
kalman_localization_node_2state.py

Replaces the 4-state CV KF with a 2-state ([north, west]) KF
that:
  - Prediction: adds ZED Δn, Δw when ‖v‖ < 1 m/s; otherwise discards.
  - Correction: applies Mahalanobis gating (χ²₀.₉₉=9.21) on GPS.
  - Floors covariance diagonals to a minimum variance.
  - Publishes TwistStamped: .linear.x=north, .linear.y=west,
    .angular.z=yaw (wrapped to [–π, π]).
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix

class KalmanLocalizationNode(Node):
    def __init__(self):
        super().__init__('kalman_localization_node')

        # --- Filter parameters ---
        self.x = np.zeros(2)                      # [north, west]
        self.P = np.diag([1e3, 1e3])              # large initial uncertainty
        self.min_var = 1e-2                       # covariance flooring
        self.gate_thresh = 9.21                   # χ²₀.₉₉, 2 DOF ≈9.21
        self.vel_limit = 1.5                      # m/s max for ZED

        # --- State trackers ---
        self.last_z_time = None  # nanoseconds
        self.last_z_meas = np.zeros(2)  # [n, w]
        self.last_g_time = None

        self.initial_lat = None
        self.initial_lon = None

        self.current_yaw = 0.0

        # --- I/O ---
        self.create_subscription(
            TwistStamped,
            '/zed/pose',
            self.depth_callback,
            30)
        self.create_subscription(
            NavSatFix,
            '/rtk/navsatfix_data',
            self.gps_callback,
            10)
        self.pub = self.create_publisher(
            TwistStamped,
            '/autonomy/pose/robot/global',
            10)
        self.create_timer(0.1, self.publish_estimate)
        self.get_logger().info("2-state Kalman Localization node started.")

    def depth_callback(self, msg: TwistStamped):
        # timestamp in ns
        t_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

        meas = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y
        ])

        # first ZED → just init
        if self.last_z_time is None:
            self.last_z_time = t_ns
            self.last_z_meas = meas
            self.current_yaw = msg.twist.angular.z
            return

        dt = (t_ns - self.last_z_time) * 1e-9
        if dt <= 1e-6:
            return

        delta = meas - self.last_z_meas
        v = delta / dt
        speed = np.linalg.norm(v)
        # always update for next iteration
        self.last_z_time = t_ns
        self.last_z_meas = meas
        self.current_yaw = msg.twist.angular.z

        if speed < self.vel_limit:
            # measurement covariance from ZED
            cov_flat = np.array([0.0001, 0.0, 0.0, 0.0001])
            R = cov_flat.reshape(2,2)
            # prediction = x + Δ; P = P + R
            self.x += delta
            self.P += R
        else:
            self.get_logger().warning(
                f"ZED update skipped (|v|={speed:.2f} m/s > {self.vel_limit})"
            )

    def gps_callback(self, msg: NavSatFix):
        # first GPS → set reference
        if self.initial_lat is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude
            self.get_logger().info(
                f"Initial GPS ref set to lat={self.initial_lat:.6f}, "
                f"lon={self.initial_lon:.6f}"
            )
            return

        # timestamp
        t_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

        # convert to north/west
        z = self._latlon_to_nw(msg.latitude, msg.longitude)

        # build R from NavSatFix covariance (use first 2×2)
        cov = msg.position_covariance
        R = np.array([[cov[0], cov[1]],
                      [cov[3], cov[4]]])

        # gating dt reference
        ref_t_ns = max(
            self.last_z_time or 0,
            self.last_g_time or 0
        )
        # innovation
        y = z - self.x
        S = self.P + R
        # Mahalanobis distance²
        maha2 = float(y.T @ np.linalg.inv(S) @ y)

        if maha2 < self.gate_thresh:
            K = self.P @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(2) - K) @ self.P
            # floor covariance
            self.P[0,0] = max(self.P[0,0], self.min_var)
            self.P[1,1] = max(self.P[1,1], self.min_var)
        '''else:
            self.get_logger().warning(
                f"GPS update skipped (maha²={maha2:.2f} ≥ {self.gate_thresh})"
            )'''

        self.last_g_time = t_ns

    def publish_estimate(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(self.x[0])   # north
        msg.twist.linear.y = float(self.x[1])   # west
        msg.twist.angular.z = float(self.current_yaw)
        #self.get_logger().info(f"{self.current_yaw}")
        self.pub.publish(msg)

    def _latlon_to_nw(self, lat, lon):
        R_e = 6_378_137.0
        φ0 = math.radians(self.initial_lat)
        λ0 = math.radians(self.initial_lon)
        φ  = math.radians(lat)
        λ  = math.radians(lon)
        dφ = φ - φ0
        dλ = λ - λ0
        μ  = 0.5*(φ + φ0)
        north = dφ * R_e
        west  = -dλ * R_e * math.cos(μ)
        return np.array([north, west])

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
