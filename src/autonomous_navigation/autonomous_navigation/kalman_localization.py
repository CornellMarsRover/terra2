#!/usr/bin/env python3
"""
kalman_localization_node_logging.py

Extends the original Kalman Localization node with optional data recording.  
If the ROS parameter `record_data` is set to `true`, the node logs every sensor
update (GPS or ZED odometry) to a CSV file. Each row contains:

1. ROS time (seconds) when the measurement was processed.
2. Measurement source ("gps" or "zed").
3. Measurement vector (2 elements: north, west) – *camera entries are Δn, Δw; GPS entries are absolute north, west w.r.t. the first fix*.
4. Measurement covariance (flattened 2 × 2 = 4 elements).
5. Current state estimate **after** the measurement (4 elements).
6. Current state covariance **after** the measurement (flattened 4 × 4 = 16 elements).

Author: [Your Name]
Date: [Date]
"""
import csv
import datetime as dt
import math
import os
from typing import Iterable, Union

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry  # (not used directly, kept for reference)
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# ------------------------------------------------------------------------------
# Helper utilities
# ------------------------------------------------------------------------------

def _flatten(array: Union[np.ndarray, Iterable[float]]) -> list:
    """Returns a 1‑D Python list for any NumPy array / iterable."""
    if isinstance(array, np.ndarray):
        return array.reshape(-1).tolist()
    return list(array)


# ------------------------------------------------------------------------------
# Kalman Localization Node
# ------------------------------------------------------------------------------
class KalmanLocalizationNode(Node):
    """Fuses GPS + ZED delta pose using a constant‑velocity Kalman filter."""

    # ----------------------------------
    # Construction / Parameters
    # ----------------------------------
    def __init__(self):
        super().__init__("kalman_localization_node")

        # ---------- ROS parameter to enable logging ----------
        self.declare_parameter("record_data", False)
        self.record_data: bool = (
            self.get_parameter("record_data").get_parameter_value().bool_value
        )
        self.record_data = True
        # ---------- Optional CSV logging ----------
        self.csv_writer = None
        self.csv_file = None
        if self.record_data:
            log_dir = os.path.join(os.path.expanduser("~"), "kalman_logs")
            os.makedirs(log_dir, exist_ok=True)
            timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(log_dir, f"kalman_log_{timestamp}.csv")
            self.csv_file = open(filename, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            # CSV header – keeps columns self‑describing
            header = (
                [
                    "ros_time_s",
                    "source",
                    "meas_n",
                    "meas_w",
                    "cov_00",
                    "cov_01",
                    "cov_10",
                    "cov_11",
                    "x_n",
                    "x_w",
                    "x_vn",
                    "x_vw",
                ]
                + [f"P_{i}{j}" for i in range(4) for j in range(4)]
            )
            self.csv_writer.writerow(header)
            self.get_logger().info(f"Recording enabled → {filename}")

        # ----------------------------------
        # Kalman Filter state (4‑state CV model)
        # ----------------------------------
        self.x = np.zeros(4, dtype=float)  # [n, w, vn, vw]
        self.P = np.eye(4) * 0.005
        self.Q = np.eye(4) * 0.005
        self.R_gps_base = np.array([[2.0, 0.4], [0.4, 2.0]])

        # State for camera prediction step
        self.last_camera_time = None
        self.last_camera_n = 0.0
        self.last_camera_w = 0.0
        self.last_camera_cov = None  # will store 4‑element vector (flattened 2×2)

        # GPS reference (lat/lon of first fix)
        self.initial_lat = None
        self.initial_lon = None

        # Latest yaw reading (passed through to published message)
        self.current_yaw = 0.0

        # ----------------------------------
        # ROS I/O – subscriptions & publication
        # ----------------------------------
        self.create_subscription(NavSatFix, "/rtk/navsatfix_data", self.gps_callback, 10)
        self.create_subscription(
            TwistWithCovarianceStamped, "/zed/pose", self.depth_callback, 30
        )

        self.pub_estimate = self.create_publisher(
            TwistStamped, "/autonomy/pose/robot/global", 10
        )
        self.create_timer(0.1, self.publish_estimate)

        self.get_logger().info("Kalman Localization node started.")

    # ------------------------------------------------------------------
    # ZED / Depth‑camera callback → *prediction* step
    # ------------------------------------------------------------------
    def depth_callback(self, msg: TwistWithCovarianceStamped):
        now_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # First callback: just initialise trackers
        if self.last_camera_time is None:
            self.last_camera_time = now_ns
            self.last_camera_n = msg.twist.twist.linear.x
            self.last_camera_w = msg.twist.twist.linear.y
            self.last_camera_cov = np.array(msg.twist.covariance[0:4])
            self.current_yaw = msg.twist.twist.angular.z
            return

        # Δt (s)
        dt = (now_ns - self.last_camera_time) * 1e-9
        if dt <= 1e-6:
            return  # ignore duplicates / zero‑time messages
        self.last_camera_time = now_ns

        # Δn, Δw since previous camera frame (global frame)
        dn = msg.twist.twist.linear.x - self.last_camera_n
        dw = msg.twist.twist.linear.y - self.last_camera_w
        self.last_camera_n = msg.twist.twist.linear.x
        self.last_camera_w = msg.twist.twist.linear.y
        self.current_yaw = msg.twist.twist.angular.z

        # Measurement covariance from camera (provided as 6×6; take top‑left 2×2)
        cov_new = np.array(msg.twist.covariance[0:4])
        S = (cov_new + self.last_camera_cov) / (dt ** 2)
        self.last_camera_cov = cov_new

        # Build process‑noise contribution (discretised continuous‑white‑noise)
        J = np.array([[dt, 0.0], [0.0, dt], [1.0, 0.0], [0.0, 1.0]])
        Q = J @ S.reshape(2, 2) @ J.T

        # ---------- Prediction step (constant velocity) ----------
        v_n = dn / dt
        v_w = dw / dt
        A = np.eye(4)
        A[0, 2] = dt  # n += vn·dt
        A[1, 3] = dt  # w += vw·dt

        # Predict state / covariance
        self.x[2] = v_n  # overwrite with camera‑derived velocity
        self.x[3] = v_w
        self.x = A @ self.x
        self.P = A @ self.P @ A.T + Q

        # ---------- Logging ----------
        if self.record_data:
            self._log_measurement("zed", np.array([dn, dw]), S.reshape(2, 2))

    # ------------------------------------------------------------------
    # GPS callback → *correction* step
    # ------------------------------------------------------------------
    def gps_callback(self, msg: NavSatFix):
        # Capture first fix as reference
        if self.initial_lat is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude
            self.get_logger().info(
                f"Initial GPS ref: lat={self.initial_lat:.6f}, lon={self.initial_lon:.6f}"
            )
            return

        # Convert to north/west (m) wrt reference
        meas_n, meas_w = self._latlon_to_nw(msg.latitude, msg.longitude)

        # Build measurement / covariance
        z = np.array([meas_n, meas_w])
        R = np.array([[msg.altitude, 0.0], [0.0, msg.altitude]])

        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])

        # ---------- Kalman correction ----------
        y = z - H @ self.x  # innovation
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ H) @ self.P

        # ---------- Logging ----------
        if self.record_data:
            self._log_measurement("gps", z, R)

    # ------------------------------------------------------------------
    # CSV logging helper
    # ------------------------------------------------------------------
    def _log_measurement(
        self, source: str, measurement: np.ndarray, cov: Union[np.ndarray, Iterable[float]]
    ) -> None:
        if not self.csv_writer:
            return
        now = self.get_clock().now().nanoseconds * 1e-9  # seconds (float)
        row = (
            [now, source]
            + _flatten(measurement)
            + _flatten(cov)
            + _flatten(self.x)
            + _flatten(self.P)
        )
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    # ------------------------------------------------------------------
    # Publisher – regularly publishes current global pose estimate
    # ------------------------------------------------------------------
    def publish_estimate(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(self.x[0])  # north (m)
        msg.twist.linear.y = float(self.x[1])  # west  (m)
        msg.twist.angular.z = float(self.current_yaw)  # yaw (rad)
        self.pub_estimate.publish(msg)

    # ------------------------------------------------------------------
    # Helper: lat/lon → local north/west (m) w.r.t. first fix
    # ------------------------------------------------------------------
    def _latlon_to_nw(self, lat: float, lon: float):
        R_e = 6_378_137.0  # WGS‑84 mean radius [m]
        lat0_rad = math.radians(self.initial_lat)
        lon0_rad = math.radians(self.initial_lon)
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        d_lat = lat_rad - lat0_rad
        d_lon = lon_rad - lon0_rad
        mean_lat = (lat_rad + lat0_rad) / 2.0
        north = d_lat * R_e
        west = -d_lon * R_e * math.cos(mean_lat)
        return north, west

    # ----------------------------------
    # Shutdown – cleanly close CSV file
    # ----------------------------------
    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


# ------------------------------------------------------------------------------
# Main entry point
# ------------------------------------------------------------------------------

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


if __name__ == "__main__":
    main()
