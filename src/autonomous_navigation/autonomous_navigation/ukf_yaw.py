import math
import numpy as np
import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import NavSatFix, Imu
from cmr_msgs.msg import IMUSensorData, AutonomyDrive

L = 0.83  # Wheelbase of the rover (meters)

class UKFYaw(Node):

    def __init__(self):
        super().__init__('ukf_yaw_filter_node')

        self.declare_parameter('real', False) # FALSE IF RUNNING IN SIMULATION
        self.real = self.get_parameter('real').get_parameter_value().bool_value

        # Robot drive command subscribers
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

        # IMU and GPS subscriptions
        if not self.real:
            self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_sim_callback, 10)
            self.sub_gps = self.create_subscription(NavSatFix, '/gps_exact', self.gps_callback, 10)
        else:
            self.sub_imu = self.create_subscription(IMUSensorData, '/imu', self.imu_callback, 10)
            self.sub_gps = self.create_subscription(NavSatFix, '/rtk/navsatfix_data', self.gps_callback, 10)

        # Publisher for the filtered pose estimate
        self.pub = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)

        # For GPS coordinate initialization
        self.initial_lat = None
        self.initial_lon = None
        if not self.real:
            self.initial_lat = 38.161479
            self.initial_lon = -122.454630

        # Sensor offsets (not really used now, but kept for completeness)
        self.r_gps = [0.26, 0.08, 0.0]
        self.r_imu = [-0.2, 0.05, 0.0]

        # Scaling factors to correct for sim driving
        self.k_pt_turn   = -1.32
        self.k_ackerman  =  1.78

        # Last measurement times
        self.last_imu_time = None
        self.last_gps_time = None

        # Drive velocity (robot frame): [dx, dy, d_yaw], from commanded drive
        self.drive_velocity = [0.0, 0.0, 0.0]

        # GPS measurements (treated as absolute ground truth in N/W):
        self.gps_meas = [0.0, 0.0]

        # IMU measurements: [acc_x, acc_y, yaw, omega], but we’ll only fuse yaw, omega
        self.imu_meas = [0.0, 0.0, 0.0, 0.0]

        # State vector x = [north, west, yaw, dx_body, dy_body, omega]^T
        # We'll only really be filtering yaw with the UKF update, but let's keep the shape.
        self.x = np.zeros(6)

        # --- UKF parameters and matrices ---
        self.n_x = 6
        self.P = np.eye(self.n_x) * 0.1  # State covariance

        # Process noise (we will only meaningfully use the yaw portion below, but keep shape):
        self.Q = np.diag([
            0.0, 0.0,   # (north, west) will be overridden by GPS anyway
            0.0, 0.0,   # (dx_body, dy_body) not used in final filter
            0.001, 0.001  # small process noise for omega, just for stability
        ])

        # We consider GPS as absolute truth, so we do *not* fuse it. We’ll simply override x[0], x[1].
        # Meanwhile we keep the IMU measurement noise for yaw, omega:
        self.R_imu = np.diag([0.0001, 0.0001])  # [yaw, omega]

        # Unscented transform parameters
        self.alpha = 1e-3
        self.beta  = 2.0
        self.kappa = 0.0
        self.lmbda = (self.alpha**2) * (self.n_x + self.kappa) - self.n_x

        # Sigma point weights
        self.Wm = np.zeros(2*self.n_x + 1)
        self.Wc = np.zeros(2*self.n_x + 1)
        self.Wm[0] = self.lmbda / (self.n_x + self.lmbda)
        self.Wc[0] = self.Wm[0] + (1.0 - self.alpha**2 + self.beta)
        for i in range(1, 2*self.n_x + 1):
            self.Wm[i] = 1.0 / (2.0 * (self.n_x + self.lmbda))
            self.Wc[i] = self.Wm[i]

        # Create a timer to run the filter at 10 Hz
        self.timer_ukf = self.create_timer(0.1, self.ukf_loop)

    #--------------------------------------------------------------------------
    # Subscriptions / Callbacks
    #--------------------------------------------------------------------------
    def point_turn_callback(self, msg: Twist):
        """
        Update yaw rate from point turn commands. We ignore linear motion.
        """
        omega = msg.angular.z
        if not self.real:
            omega *= self.k_pt_turn
        self.drive_velocity = [0.0, 0.0, omega]

    def ackerman_callback(self, msg: AutonomyDrive):
        """
        Update yaw rate from Ackerman drive commands.
        We treat GPS as absolute for position, so we only keep angular velocity.
        """
        if abs(msg.fl_angle) < 3.0:
            # effectively going straight, no yaw rate
            v = msg.vel
            if not self.real:
                v *= self.k_ackerman
            self.drive_velocity = [v, 0.0, 0.0]  # yaw rate is zero
        else:
            R = L / math.tan(math.radians(msg.fl_angle))
            omega = msg.vel / R
            if not self.real:
                omega *= self.k_ackerman
            self.drive_velocity = [0.0, 0.0, omega]

    def gps_callback(self, msg: NavSatFix):
        """
        We treat GPS as absolute ground truth for north/west,
        so we just override state[0], state[1].
        """
        if self.initial_lat is None or self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude

        self.last_gps_time = msg.header.stamp
        n, w = self.get_north_west_meters(msg.latitude, msg.longitude)
        self.gps_meas = [n, w]

        # Override the filter's north, west with "ground truth"
        self.x[0] = n
        self.x[1] = w

    def imu_callback(self, msg: IMUSensorData):
        """
        For real system, we read yaw in degrees (anglez) -> convert to radians, plus yaw rate.
        We'll fuse yaw, omega in the UKF.
        """
        self.last_imu_time = msg.header.stamp
        self.imu_meas = [
            msg.accx,           # not used for final
            msg.accy,           # not used for final
            math.radians(msg.anglez),  # yaw
            msg.gyroz           # yaw rate
        ]

    def imu_sim_callback(self, msg: Imu):
        """
        For simulation: convert quaternion to euler, store yaw, store yaw rate.
        """
        self.last_imu_time = msg.header.stamp
        q = msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw   = euler[2]
        omega = msg.angular_velocity.z
        # Store IMU data: [acc_x_body, acc_y_body, yaw, omega] but we only use yaw, omega
        self.imu_meas = [msg.linear_acceleration.x, msg.linear_acceleration.y, yaw, omega]

    #--------------------------------------------------------------------------
    # Main Loop
    #--------------------------------------------------------------------------
    def ukf_loop(self):
        """
        We run the filter at 10 Hz for yaw only. GPS is treated as absolute for position.
        So effectively, we do:
          1) Predict step (only yaw state evolves using drive_velocity's omega)
          2) IMU update (yaw, omega)
          3) Publish
        """
        now = self.get_clock().now().to_msg()
        now_sec = now.sec + now.nanosec * 1e-9
        if not hasattr(self, 'previous_time_sec'):
            self.previous_time_sec = now_sec
            return
        dt = now_sec - self.previous_time_sec
        self.previous_time_sec = now_sec

        if dt <= 0.0:
            return

        # 1) Predict
        self.ukf_predict(self.x, self.P, dt)

        # 2) IMU update (yaw, omega)
        self.ukf_update_imu(self.x, self.P)

        # 3) Since GPS is absolute, we do not fuse it in the filter. We already override self.x[0], self.x[1] on callback.

        # 4) Publish
        self.publish_state()

    def publish_state(self):
        """
        Publish north, west, yaw in TwistStamped:
          linear.x = north
          linear.y = west
          angular.z = yaw
        """
        pose_msg = TwistStamped()
        pose_msg.twist.linear.x = float(self.x[0])  # Overridden by GPS
        pose_msg.twist.linear.y = float(self.x[1])  # Overridden by GPS
        pose_msg.twist.angular.z = float(self.x[2]) # Filtered
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(pose_msg)

    #--------------------------------------------------------------------------
    # Unscented Transform
    #--------------------------------------------------------------------------
    def sigma_points(self, x, P):
        P = 0.5 * (P + P.T)  # enforce symmetry
        U = np.linalg.cholesky((self.n_x + self.lmbda) * P)
        sigma_pts = [x]
        for i in range(self.n_x):
            sigma_pts.append(x + U[:, i])
            sigma_pts.append(x - U[:, i])
        return np.array(sigma_pts)  # shape: (2n+1, n_x)

    def unscented_mean_cov(self, S):
        """
        Weighted mean/cov of the sigma points after transformation.
        We only meaningfully filter yaw (index 2) and maybe omega (index 5).
        """
        xm = np.zeros(self.n_x)
        for i in range(2*self.n_x + 1):
            xm += self.Wm[i] * S[i]
        # wrap yaw
        xm[2] = self.wrap_angle(xm[2])

        Pc = np.zeros((self.n_x, self.n_x))
        for i in range(2*self.n_x + 1):
            dx = S[i] - xm
            dx[2] = self.wrap_angle(dx[2])
            Pc += self.Wc[i] * np.outer(dx, dx)

        return xm, Pc

    #--------------------------------------------------------------------------
    # Process Model (Yaw Only)
    #--------------------------------------------------------------------------
    def f(self, x_in, dt, drive_vel):
        """
        Simplified. 
        We trust GPS for (north, west) so do NOT model them here.
        We only incorporate the "expected" angular velocity from drive commands
        to predict yaw.

        The state is still [n, w, yaw, dx_b, dy_b, omega].
        But for the prediction we do:
            new_yaw = old_yaw + drive_vel[2] * dt
        and keep everything else the same.

        Any IMU linear acceleration logic is removed.
        """
        x_out = np.copy(x_in)

        old_yaw = x_in[2]
        old_omega = x_in[5]

        # Overwrite predicted yaw from drive commands
        om_cmd = drive_vel[2]  # from ackerman or point_turn
        new_yaw = self.wrap_angle(old_yaw + om_cmd*dt)

        # Keep other states as is
        x_out[2] = new_yaw
        x_out[5] = om_cmd  # we assume the drive command sets the "expected" yaw rate

        return x_out

    def ukf_predict(self, x, P, dt):
        """
        Generate sigma points, pass them through f, and compute new mean/cov,
        then add Q. We'll keep a small Q for yaw dimension only.
        """
        S = self.sigma_points(x, P)
        S_pred = []
        for i in range(S.shape[0]):
            S_pred.append(self.f(S[i], dt, self.drive_velocity))
        S_pred = np.array(S_pred)

        x_pred, P_pred = self.unscented_mean_cov(S_pred)

        # Add process noise (only truly relevant for yaw/omega).
        P_pred += self.Q

        self.x = x_pred
        self.P = P_pred

    #--------------------------------------------------------------------------
    # IMU Update (Yaw, Omega)
    #--------------------------------------------------------------------------
    def h_imu(self, x_in):
        """
        The measurement model for IMU yaw & omega:
          z_imu = [yaw, omega].
        We'll keep angle wrapping for yaw.
        """
        return np.array([self.wrap_angle(x_in[2]), x_in[5]])

    def ukf_update_imu(self, x, P):
        """
        Standard UKF measurement update for IMU yaw, omega.
        """
        S = self.sigma_points(x, P)
        Z = []
        for i in range(S.shape[0]):
            zi = self.h_imu(S[i])
            zi[0] = self.wrap_angle(zi[0])  # wrap each yaw
            Z.append(zi)
        Z = np.array(Z)  # shape: (2n+1, 2)

        # Mean predicted measurement
        z_dim = 2
        z_pred = np.zeros(z_dim)
        for i in range(2*self.n_x + 1):
            z_pred += self.Wm[i] * Z[i]
        z_pred[0] = self.wrap_angle(z_pred[0])  # wrap

        # Innovation covariance S_zz
        S_zz = np.zeros((z_dim, z_dim))
        for i in range(2*self.n_x + 1):
            dz = Z[i] - z_pred
            dz[0] = self.wrap_angle(dz[0])
            S_zz += self.Wc[i] * np.outer(dz, dz)
        S_zz += self.R_imu

        # Cross covariance S_xz
        S_xz = np.zeros((self.n_x, z_dim))
        for i in range(2*self.n_x + 1):
            dx = S[i] - x
            dx[2] = self.wrap_angle(dx[2])
            dz = Z[i] - z_pred
            dz[0] = self.wrap_angle(dz[0])
            S_xz += self.Wc[i] * np.outer(dx, dz)

        # Kalman gain
        K = S_xz @ np.linalg.inv(S_zz)

        # Actual measurement
        yaw_meas   = self.imu_meas[2]
        omega_meas = self.imu_meas[3]
        z_meas = np.array([yaw_meas, omega_meas])

        z_err = z_meas - z_pred
        z_err[0] = self.wrap_angle(z_err[0])

        # State update
        new_x = x + K @ z_err
        new_x[2] = self.wrap_angle(new_x[2])

        new_P = P - K @ S_zz @ K.T

        self.x = new_x
        self.P = 0.5 * (new_P + new_P.T)  # enforce symmetry

    #--------------------------------------------------------------------------
    # Helpers
    #--------------------------------------------------------------------------
    def get_north_west_meters(self, lat, lon):
        """
        Convert lat/lon to north/west displacement from the initial lat/lon.
        We treat these as absolute ground truth for position.
        """
        R = 6378137.0
        lat1_rad = math.radians(lat)
        lat0_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(lon)
        lon0_rad = math.radians(self.initial_lon)

        d_lat  = lat0_rad - lat1_rad
        d_lon  = lon0_rad - lon1_rad
        mean_lat = (lat1_rad + lat0_rad) / 2.0

        north = -1.0 * d_lat * R
        west  = d_lon * R * math.cos(mean_lat)
        return north, west

    def wrap_angle(self, angle):
        """Wrap angle into [-pi, pi]."""
        return (angle + math.pi) % (2.0*math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = UKFYaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
