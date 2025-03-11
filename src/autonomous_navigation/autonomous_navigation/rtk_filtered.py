import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import NavSatFix
from cmr_msgs.msg import IMUSensorData, AutonomyDrive

L = 0.83  # wheelbase of the rover (meters)

class RTKFilter(Node):
    def __init__(self):
        super().__init__('rtk_filter_node')
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
        self.sub_ackerman = self.create_subscription(
            AutonomyDrive,
            '/autonomy/move/ackerman',
            self.ackerman_callback,
            10
        )
        self.sub_turn = self.create_subscription(
            Twist,
            '/autonomy/move/point_turn',
            self.point_turn_callback,
            10
        )
        # Publisher for the filtered pose estimate
        self.pub = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        
        # Timer for 10Hz updates
        self.dt = 0.1  # timer period in seconds
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # For GPS coordinate initialization
        self.initial_lat = None
        self.initial_lon = None

        # Storage for latest sensor measurements
        self.north = 0.0  # north displacement from GPS (meters)
        self.west = 0.0   # west displacement from GPS (meters)
        self.gps_noise = 1.0  # default gps reading noise (meters)
        self.yaw = 0.0  # from IMU in radians

        # Velocities of center of rover (derived from drive commands)
        self.d_north = 0.0  # linear velocity in north direction (m/s)
        self.d_west = 0.0   # linear velocity in west direction (m/s)
        self.d_yaw = 0.0    # angular velocity about yaw axis (rad/s)

        # GPS antenna offset from center of rover in meters
        # first element is forward in frame of rover and second element is left
        self.r = [0.26, 0.08, 0.0]

        # Kalman filter initialization
        # State: [north, west, yaw, d_north, d_west, d_yaw]^T
        self.state = np.zeros((6, 1))
        # Initial covariance: set to a small value because we assume relatively accurate initial knowledge
        self.P = np.eye(6) * 1e-3

    def gps_callback(self, msg: NavSatFix):
        """
        Process GPS data.
        Note:
          - The altitude field is used as the horizontal standard deviation (in m)
          - llh2ecef converts (lat, lon, height) to ECEF x,y,z (in meters)
        """
        lat = msg.latitude
        lon = msg.longitude
        self.gps_noise = msg.altitude if msg.altitude > 0.0 else 1.0
        # Initialize coordinates if first measurement
        if self.initial_lat is None:
            self.initial_lat = lat
            self.initial_lon = lon
            return
        north, west = self.get_north_west_meters(lat, lon)
        self.north = north
        self.west = west
        
    def imu_callback(self, msg: IMUSensorData):
        """
        Process IMU data.
        IMUSensorData fields:
          - accx, accy: linear accelerations in m/s^2 (body frame)
          - gyroz: angular velocity in rad/s
          - anglez: yaw angle in degrees (convert to radians)
        """
        self.yaw = math.radians(msg.anglez)

    def point_turn_callback(self, msg: Twist):
        """
        Update omega yaw and linear velocities for center of rover from point turn commands.
        """
        self.d_north = 0
        self.d_west = 0
        omega = msg.angular.z
        n, w = self.lin_from_ang(omega)
        self.d_north = n
        self.d_west = w
        self.d_yaw = omega

    def ackerman_callback(self, msg: AutonomyDrive):
        """
        Update omega yaw and linear velocities for center of rover from Ackerman drive commands.
        """
        R = L / math.tan(math.radians(msg.fl_angle))
        omega = msg.vel / R
        n, w = self.lin_from_ang(omega)
        self.d_north = (math.cos(self.yaw) * msg.vel) + n
        self.d_west = (math.sin(self.yaw) * msg.vel) + w
        self.d_yaw = omega

    def lin_from_ang(self, omega):
        """
        Helper function to calculate the linear velocity components
        resulting from angular rotation at the reference point.
        """
        w_vec = np.array([0.0, 0.0, omega])
        R_mat = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],
                          [math.sin(self.yaw),  math.cos(self.yaw)]])
        rr = R_mat.dot(np.array([self.r[0], self.r[1]]))
        r_vec = np.array([rr[0], rr[1], self.r[2]])
        vr = np.cross(w_vec, r_vec)
        return vr[0], vr[1]

    def timer_callback(self):
        """
        Every timer tick, run the Kalman filter prediction and update steps,
        then publish the filtered pose.
        """
        # --- Kalman Filter Prediction Step ---
        # Control input from drive commands
        u = np.array([[self.d_north],
                      [self.d_west],
                      [self.d_yaw]])
        # We want to update the position using the control input and
        # then set the velocity states to the drive command values.
        # Our prediction model is:
        #   n_new   = n_old   + dt * d_north
        #   w_new   = w_old   + dt * d_west
        #   yaw_new = yaw_old + dt * d_yaw
        #   d_north_new = d_north (from command)
        #   d_west_new  = d_west  (from command)
        #   d_yaw_new   = d_yaw   (from command)
        F = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 0, 0]])
        B = np.array([[self.dt, 0,      0],
                      [0,      self.dt, 0],
                      [0,      0,      self.dt],
                      [1,      0,      0],
                      [0,      1,      0],
                      [0,      0,      1]])
        # Predicted state estimate
        x_pred = F @ self.state + B @ u

        # Process noise: assume very low noise on drive commands (minimal process noise)
        # Q_u applies to the control input part
        Q_u = np.diag([1e-6, 1e-6, 1e-6])
        # Covariance prediction
        P_pred = F @ self.P @ F.T + B @ Q_u @ B.T

        # --- Measurement Update Step ---
        # Measurements: GPS gives [north, west] and IMU gives yaw.
        z = np.array([[self.north],
                      [self.west],
                      [self.yaw]])
        # Measurement matrix: we directly measure the first three state elements.
        H = np.array([[1, 0, 0, 0, 0, 0],
                      [0, 1, 0, 0, 0, 0],
                      [0, 0, 1, 0, 0, 0]])
        # Measurement noise covariance: using gps_noise for position and an assumed value for yaw.
        r_gps = self.gps_noise**2  # variance for GPS position
        r_yaw = 0.05**2            # assumed variance for yaw measurement (rad^2)
        R_cov = np.diag([r_gps, r_gps, r_yaw])

        # Innovation or measurement residual
        y_res = z - (H @ x_pred)
        # Innovation covariance
        S = H @ P_pred @ H.T + R_cov
        # Kalman gain
        K = P_pred @ H.T @ np.linalg.inv(S)
        # Updated state estimate
        x_updated = x_pred + K @ y_res
        # Updated covariance estimate
        P_updated = (np.eye(6) - K @ H) @ P_pred

        # Save updated estimates for the next iteration
        self.state = x_updated
        self.P = P_updated

        # --- Publish the filtered pose ---
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        # Publish the filtered position and yaw from the state estimate.
        twist_msg.twist.linear.x = self.state[0, 0]  # north displacement
        twist_msg.twist.linear.y = self.state[1, 0]  # west displacement
        twist_msg.twist.angular.z = self.state[2, 0]  # yaw (in radians)
        self.pub.publish(twist_msg)

        # Log the current state (positions and velocities)
        self.get_logger().info(
            f"Filtered Pose:\n"
            f"North: {self.state[0, 0]:.3f} m, West: {self.state[1, 0]:.3f} m, Yaw: {math.degrees(self.state[2, 0]):.2f} deg"
        )
        self.get_logger().info(
            f"Filtered Velocities:\n"
            f"dNorth: {self.state[3, 0]:.3f} m/s, dWest: {self.state[4, 0]:.3f} m/s, dYaw: {self.state[5, 0]:.3f} rad/s"
        )

    def get_north_west_meters(self, lat, lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        relative to the initial GPS coordinate.
        """
        R = 6378137.0  # Earth's radius in meters
        lat1_rad = math.radians(lat)
        lat2_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(lon)
        lon2_rad = math.radians(self.initial_lon)
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        # Compute north and west distances
        north = -1.0 * delta_lat * R
        west = delta_lon * R * math.cos(mean_lat)
        return north, west

def main(args=None):
    rclpy.init(args=args)
    node = RTKFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
