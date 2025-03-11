import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from cmr_msgs.msg import IMUSensorData
from pyubx2 import llh2ecef  # assumes this returns (x, y, z) in meters

class RTKLocalization(Node):
    def __init__(self):
        super().__init__('rtk_localization_node')
        # Subscribers for GPS and IMU data
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/rtk/navsatfix_data',
            self.gps_callback,
            10
        )
        self.sub_imu = self.create_subscription(
            IMUSensorData,
            '/imu',
            self.imu_callback,
            10
        )
        # Publisher for the filtered pose estimate
        self.pub = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        
        # Timer for 10Hz updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # For GPS offset initialization (using ECEF conversion)
        self.initial_x = None
        self.initial_y = None

        # Storage for latest sensor measurements
        self.imu_data = None  # to store latest IMU measurement dict
        self.new_gps = False  # flag to indicate new GPS measurement available
        self.gps_n = 0.0  # north displacement from GPS (meters)
        self.gps_w = 0.0  # west displacement from GPS (meters)
        self.gps_noise = 1.0  # horizontal std. deviation from GPS

        # Kalman filter state initialization:
        # State vector: [n, w, yaw, v_n, v_w, omega] (column vector)
        self.kf_x = np.zeros((6, 1))
        # Initialize covariance with relatively high uncertainty for positions and velocities
        self.kf_P = np.diag([100.0, 100.0, 0.1, 10.0, 10.0, 0.1])

        # Predefined yaw measurement noise variance (typical commercial IMU)
        self.yaw_variance = 0.05 ** 2  # (rad^2)

    def gps_callback(self, msg: NavSatFix):
        """
        Process GPS data. Note:
          - The altitude field is used as the horizontal standard deviation (in m)
          - llh2ecef converts (lat, lon, height) to ECEF x,y,z (in meters)
        """
        lat = msg.latitude
        lon = msg.longitude
        # Use a known fixed height (e.g. basestation height)
        basestation_height = 245.0  
        x, y, _ = llh2ecef(lat, lon, basestation_height)
        # Initialize offset from first measurement
        if self.initial_x is None:
            self.initial_x = x
            self.initial_y = y
        # Compute displacement relative to initial position
        disp_x = x - self.initial_x  # north displacement (m)
        disp_y = y - self.initial_y  # west displacement (m)
        self.gps_n = disp_x
        self.gps_w = disp_y
        # Use the altitude field as the horizontal std deviation of the GPS measurement
        self.gps_noise = msg.altitude if msg.altitude > 0.0 else 1.0
        self.new_gps = True

    def imu_callback(self, msg: IMUSensorData):
        """
        Process IMU data.
        IMUSensorData fields:
          - accx, accy: linear accelerations in m/s^2 (body frame)
          - gyroz: angular velocity in rad/s
          - anglez: yaw angle in degrees (convert to radians)
        """
        self.imu_data = {
            'accx': msg.accx,
            'accy': msg.accy,
            'gyroz': msg.gyroz,
            'yaw_meas': math.radians(msg.anglez)
        }

    def timer_callback(self):
        dt = 0.1  # time step in seconds
        
        # Set control input u = [a_n, a_w, gyroz]^T.
        # Compute acceleration in the global (north,west) frame.
        # Here we use the latest IMU yaw measurement for the rotation.
        if self.imu_data is not None:
            # Transform accelerations from body frame to global frame:
            # a_n = accx*cos(yaw) - accy*sin(yaw)
            # a_w = accx*sin(yaw) + accy*cos(yaw)
            yaw_meas = self.imu_data['yaw_meas']
            a_n = self.imu_data['accx'] * math.cos(yaw_meas) - self.imu_data['accy'] * math.sin(yaw_meas)
            a_w = self.imu_data['accx'] * math.sin(yaw_meas) + self.imu_data['accy'] * math.cos(yaw_meas)
            gyro_z = self.imu_data['gyroz']
        else:
            a_n, a_w, gyro_z = 0.0, 0.0, 0.0

        u = np.array([[a_n],
                      [a_w],
                      [gyro_z]])

        # Define state transition matrix F and control-input matrix B.
        # Process model (prediction):
        #   n_{k+1}    = n_k + v_n*dt + 0.5*a_n*dt^2
        #   w_{k+1}    = w_k + v_w*dt + 0.5*a_w*dt^2
        #   yaw_{k+1}  = yaw_k + omega*dt
        #   v_n_{k+1}  = v_n_k + a_n*dt
        #   v_w_{k+1}  = v_w_k + a_w*dt
        #   omega_{k+1} = gyroz (taken directly from control input)
        F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        B = np.array([
            [0.5 * dt * dt, 0,               0],
            [0,              0.5 * dt * dt,   0],
            [0,              0,               dt],
            [dt,             0,               0],
            [0,              dt,              0],
            [0,              0,               1]
        ])

        # Define process noise covariance Q.
        # Assume typical noise levels: sigma_a = 0.2 m/s^2 and sigma_gyro = 0.01 rad/s.
        sigma_a = 0.2
        sigma_gyro = 0.01
        U = np.diag([sigma_a**2, sigma_a**2, sigma_gyro**2])
        Q = B @ U @ B.T

        # ---- Prediction step ----
        self.kf_x = F @ self.kf_x + B @ u
        self.kf_P = F @ self.kf_P @ F.T + Q

        # ---- Measurement update: GPS position ----
        if self.new_gps:
            # Measurement vector from GPS: position (n, w)
            z_gps = np.array([[self.gps_n],
                              [self.gps_w]])
            H_gps = np.array([
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0]
            ])
            # Use the horizontal std deviation (from msg.altitude) as the noise level.
            R_gps = np.diag([self.gps_noise**2, self.gps_noise**2])
            S = H_gps @ self.kf_P @ H_gps.T + R_gps
            K = self.kf_P @ H_gps.T @ np.linalg.inv(S)
            self.kf_x = self.kf_x + K @ (z_gps - H_gps @ self.kf_x)
            self.kf_P = (np.eye(6) - K @ H_gps) @ self.kf_P
            self.new_gps = False

        # ---- Measurement update: IMU yaw ----
        if self.imu_data is not None:
            # Measurement: yaw (from IMU anglez, converted to radians)
            z_yaw = np.array([[self.imu_data['yaw_meas']]])
            H_yaw = np.array([[0, 0, 1, 0, 0, 0]])
            R_yaw = np.array([[self.yaw_variance]])
            S_yaw = H_yaw @ self.kf_P @ H_yaw.T + R_yaw
            K_yaw = self.kf_P @ H_yaw.T @ np.linalg.inv(S_yaw)
            self.kf_x = self.kf_x + K_yaw @ (z_yaw - H_yaw @ self.kf_x)
            self.kf_P = (np.eye(6) - K_yaw @ H_yaw) @ self.kf_P

        # ---- Publish the updated state ----
        # Note: twist.linear.x = north displacement, twist.linear.y = west displacement,
        # and twist.angular.z = yaw in degrees.
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = float(self.kf_x[0, 0])
        twist_msg.twist.linear.y = float(self.kf_x[1, 0])
        twist_msg.twist.angular.z = self.kf_x[2, 0]
        self.pub.publish(twist_msg)
        x = self.kf_x
        self.get_logger().info(f"Kalman filter state:\nx: {x[0]}m\ny: {x[1]}m\nyaw: {math.degrees(x[2])}deg\nvx: {x[3]}m/s\nvy: {x[4]}m/s\ndyaw: {x[5]}rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = RTKLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
