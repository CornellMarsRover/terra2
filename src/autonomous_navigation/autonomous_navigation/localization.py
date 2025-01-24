#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32MultiArray
from tf_transformations import euler_from_quaternion
import numpy as np
import math
from cmr_msgs.msg import IMUSensorData


class KalmanFilter2D:
    def __init__(self):
        self.x = np.zeros((4, 1), dtype=float)
        self.P = np.eye(4) * 1.0
        self.Q = np.eye(4) * 0.1
        self.R = np.eye(2) * 5.0
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        self.last_time = None

    def predict(self, dt, a_n, a_w):
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        G = np.array([[0.5 * dt**2, 0], [0, 0.5 * dt**2], [dt, 0], [0, dt]], dtype=float)
        a = np.array([[a_n], [a_w]], dtype=float)
        self.x = F @ self.x + G @ a
        self.P = F @ self.P @ F.T + self.Q

    def update(self, p_n_meas, p_w_meas):
        z = np.array([[p_n_meas], [p_w_meas]], dtype=float)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

    def get_position(self):
        return self.x[0, 0], self.x[1, 0]

    def get_velocity(self):
        return self.x[2, 0], self.x[3, 0]

    def get_speed(self):
        v_n, v_w = self.get_velocity()
        return np.sqrt(v_n**2 + v_w**2)


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.kf = KalmanFilter2D()
        self.initial_lat = None
        self.initial_lon = None
        self.last_imu_time = None
        self.current_yaw = None

        self.create_subscription(NavSatFix, '/navsatfix', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/autonomy/pose/robot/global', 10)
        self.velocity_publisher = self.create_publisher(Float32MultiArray, '/autonomy/velocity', 10)

        self.timer = self.create_timer(0.1, self.publish_data)

    def gps_callback(self, msg):
        if self.initial_lat is None or self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude

        meas_north, meas_west = self.get_north_west_meters(
            self.initial_lat, self.initial_lon, msg.latitude, msg.longitude
        )
        self.kf.update(meas_north, meas_west)

    def imu_callback(self, msg):
        q = msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = euler[2]

        ax_robot = msg.linear_acceleration.x
        ay_robot = msg.linear_acceleration.y
        theta = -self.current_yaw
        a_n = math.cos(theta) * ax_robot - math.sin(theta) * ay_robot
        a_w = math.sin(theta) * ax_robot + math.cos(theta) * ay_robot

        current_time = self.get_clock().now()
        dt = (current_time - self.last_imu_time).nanoseconds * 1e-9 if self.last_imu_time else 0.01
        self.last_imu_time = current_time

        self.kf.predict(dt, a_n, a_w)

    def publish_data(self):
        pose = Float32MultiArray(data=[*self.kf.get_position(), self.current_yaw or 0.0])
        velocity = Float32MultiArray(data=[*self.kf.get_velocity(), self.kf.get_speed()])
        self.pose_publisher.publish(pose)
        self.velocity_publisher.publish(velocity)

    def get_north_west_meters(self, start_lat, start_lon, target_lat, target_lon):
        R = 6378137.0
        lat1_rad, lat2_rad = map(math.radians, [start_lat, target_lat])
        lon1_rad, lon2_rad = map(math.radians, [start_lon, target_lon])
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0
        delta_north = delta_lat * R
        delta_east = delta_lon * R * math.cos(mean_lat)
        return delta_north, -delta_east


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
