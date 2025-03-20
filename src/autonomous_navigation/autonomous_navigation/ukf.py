import math
import numpy as np
import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import TwistStamped, Twist
from sensor_msgs.msg import NavSatFix, Imu
from cmr_msgs.msg import IMUSensorData, AutonomyDrive

L = 0.83  # Wheelbase of the rover (meters)

class UKF(Node):

    def __init__(self):
        super().__init__('ukf_filter_node')

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
        self.initial_lat = 42.4443013  # Real starting coords eng quad
        self.initial_lon = -76.4832399
        if not self.real:
            self.initial_lat = 38.161479
            self.initial_lon = -122.454630

        # Sensor offsets from the center of the rover (in the robot frame).
        # r_gps and r_imu are [x, y, z] offsets in the body frame.
        self.r_gps = [0.26, 0.08, 0.0]
        self.r_imu = [-0.2, 0.05, 0.0]

        # Scaling factors to correct for sim driving
        self.k_pt_turn   = -1.32
        self.k_ackerman  =  1.78

        # Last measurement times
        self.last_imu_time = None
        self.last_gps_time = None

        # Current measurements / controls
        # Drive velocity (robot frame): [dx, dy, d_yaw], from commanded drive
        self.drive_velocity = [0.0, 0.0, 0.0]

        # GPS measurements: [north, west] in meters from initial coordinate
        self.gps_meas = [0.0, 0.0]

        # IMU measurements: [acc_x, acc_y, yaw, omega], in robot frame for acc
        # yaw, omega about z
        self.imu_meas = [0.0, 0.0, 0.0, 0.0]

        # State vector x = [north, west, yaw, dx_body, dy_body, omega]^T
        self.x = np.zeros(6)

        # --- UKF parameters and matrices ---
        # State dimension
        self.n_x = 6

        # We keep one global covariance P
        self.P = np.eye(self.n_x) * 0.1

        # Process noise covariance (tune as needed).
        # Q will be added in the prediction step. 
        # Because we're integrating accelerations, you will likely need
        # to increase parts of Q to reflect real-world uncertainty.
        # Process noise covariance (tune as needed)
        self.Q = np.diag([
            0.001, 0.001,    # north, west
            0.1,  0.1,     # dx_body, dy_body
            0.001, 0.001   # omega (added missing value)
        ])

        # Measurement noise covariance for GPS: R_gps (2x2)
        # (north, west). Adjust as needed.
        self.R_gps = np.diag([0.05, 0.05])

        # Measurement noise covariance for IMU: R_imu (2x2)
        # We'll only fuse yaw and omega in the measurement update here for simplicity.
        self.R_imu = np.diag([0.001, 0.001])  # [yaw, omega]

        # Parameters for Unscented Transform
        self.alpha = 1e-3   # Small positive value
        self.beta  = 2.0    # Optimal for Gaussian distributions
        self.kappa = 0.0
        self.lmbda = (self.alpha**2) * (self.n_x + self.kappa) - self.n_x

        # Compute weights for the sigma points (constant for given n_x, alpha, beta, kappa)
        self.Wm = np.zeros(2*self.n_x + 1)  # Weights for means
        self.Wc = np.zeros(2*self.n_x + 1)  # Weights for covariance
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
        Update omega (yaw rate) and linear velocities (in robot frame) 
        for center of rover from point turn commands.
        """
        omega = msg.angular.z
        if not self.real:
            omega *= self.k_pt_turn
        # For a point turn, no linear motion in x or y in the robot frame.
        self.drive_velocity = [0.0, 0.0, omega]
        #self.get_logger().info(f"[point_turn_callback] drive_velocity: {self.drive_velocity}")

    def ackerman_callback(self, msg: AutonomyDrive):
        """
        Update omega (yaw rate) and linear velocities (in robot frame) 
        for center of rover from Ackerman drive commands.
        """
        if abs(msg.fl_angle) < 3.0:
            # Essentially going straight
            v = msg.vel
            if not self.real:
                v *= self.k_ackerman
            self.drive_velocity = [v, v * math.sin(math.radians(msg.fl_angle)), 0.0]
        else:
            # Ackerman steering
            R = L / math.tan(math.radians(msg.fl_angle))
            omega = msg.vel / R
            v     = msg.vel
            if not self.real:
                omega *= self.k_ackerman
                v     *= self.k_ackerman
            vx = v * math.cos(math.radians(msg.fl_angle))
            vy = v * math.sin(math.radians(msg.fl_angle))
            self.drive_velocity = [vx, vy, omega]

        #self.get_logger().info(f"[ackerman_callback] drive_velocity: {self.drive_velocity}")

    def gps_callback(self, msg: NavSatFix):
        """
        Updates the GPS measurement of displacement from initial position.
        """
        if self.initial_lat is None or self.initial_lon is None:
            self.initial_lat = msg.latitude
            self.initial_lon = msg.longitude

        self.last_gps_time = msg.header.stamp
        n, w = self.get_north_west_meters(msg.latitude, msg.longitude)
        self.gps_meas = [n, w]

    def imu_callback(self, msg: IMUSensorData):
        """
        Updates IMU measurement of linear accelerations in the robot frame,
        yaw, and rate of rotation.
        """
        self.last_imu_time = msg.header.stamp

        # Store IMU data: [acc_x_body, acc_y_body, yaw, omega]
        self.imu_meas = [msg.accx, msg.accy, math.radians(msg.anglez), msg.gyroz]

    def imu_sim_callback(self, msg: Imu):
        """
        Updates IMU measurement of linear accelerations in the robot frame,
        yaw, and rate of rotation. *From simulation
        """
        self.last_imu_time = msg.header.stamp

        q = msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = euler[2]

        x_acc = msg.linear_acceleration.x
        y_acc = msg.linear_acceleration.y
        omega = msg.angular_velocity.z

        # Store IMU data: [acc_x_body, acc_y_body, yaw, omega]
        self.imu_meas = [x_acc, y_acc, yaw, omega]

    #--------------------------------------------------------------------------
    # UKF Main Loop
    #--------------------------------------------------------------------------
    def ukf_loop(self):
        """
        Main loop for the UKF at 10 Hz.
        1) Determine dt
        2) Predict step (incorporate IMU acceleration in the process model)
        3) Update with IMU (yaw, omega)
        4) Update with GPS (north, west)
        5) Publish state
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

        # 2) Predict
        self.ukf_predict(self.x, self.P, dt)

        # 3) IMU update (yaw, omega)
        self.ukf_update_imu(self.x, self.P)

        # 4) GPS update (north, west)
        self.ukf_update_gps(self.x, self.P)

        # 5) Publish
        self.publish_state()

    def publish_state(self):
        """
        Publishes the pose of the rover as a TwistStamped, where
        linear.x = north (m), linear.y = west (m), angular.z = yaw (rad).
        """
        pose_msg = TwistStamped()
        pose_msg.twist.linear.x = float(self.x[0])
        pose_msg.twist.linear.y = float(self.x[1])
        pose_msg.twist.angular.z = float(self.x[2])
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(pose_msg)

    #--------------------------------------------------------------------------
    # Unscented Transform
    #--------------------------------------------------------------------------
    def sigma_points(self, x, P):
        """
        Generate 2n+1 sigma points for state x with covariance P,
        per the unscented transform.
        """
        P = 0.5 * (P + P.T)  # enforce symmetry
        U = np.linalg.cholesky((self.n_x + self.lmbda) * P)
        sigma_pts = [x]
        for i in range(self.n_x):
            sigma_pts.append(x + U[:, i])
            sigma_pts.append(x - U[:, i])
        return np.array(sigma_pts)  # shape: (2n+1, n_x)

    def unscented_mean_cov(self, S):
        """
        Given array of sigma points S (2n+1, n_x) (already transformed),
        compute the mean/cov using the unscented weights Wm, Wc.
        """
        xm = np.zeros(self.n_x)
        for i in range(2*self.n_x + 1):
            xm += self.Wm[i] * S[i]
        # wrap angle
        xm[2] = self.wrap_angle(xm[2])

        Pc = np.zeros((self.n_x, self.n_x))
        for i in range(2*self.n_x + 1):
            dx = S[i] - xm
            dx[2] = self.wrap_angle(dx[2])
            Pc += self.Wc[i] * np.outer(dx, dx)
        return xm, Pc

    #--------------------------------------------------------------------------
    # Process Model
    #--------------------------------------------------------------------------
    def f(self, x_in, dt, _unused_drive_cmd):
        """
        Our updated state-transition function, incorporating IMU linear acceleration.
        x_in = [north, west, yaw, dx_b, dy_b, omega]^T

        We'll do the following:
          1) Use x_in[5] (omega) and the known offset self.r_imu to remove
             centripetal acceleration from IMU's linear acceleration.
          2) Integrate the body's acceleration to update (dx_b, dy_b).
          3) Convert old velocity in body frame -> global frame to move (north, west).
          4) Integrate yaw using x_in[5].
          5) Return updated x_out.

        NOTE: For each sigma point we do the same IMU-based acceleration,
              which is a simplification; if you want it "fully correct"
              you'd put acceleration in the state. For now, we simply feed
              in the current measured acceleration as a known input.
        """
        x_out = np.copy(x_in)

        # Current state
        north = x_in[0]
        west  = x_in[1]
        yaw   = x_in[2]
        dx_b  = x_in[3]
        dy_b  = x_in[4]
        om    = x_in[5]  # body yaw-rate

        # IMU linear acceleration in body frame
        ax_imu_body = self.imu_meas[0]
        ay_imu_body = self.imu_meas[1]

        # Correct for centripetal acceleration, since IMU is offset by r_imu in body frame.
        # For small body tilt, alpha ~ 0.  centripetal = omega x (omega x r_imu).
        # If r_imu = (rx, ry, 0) and omega = (0,0,om), then:
        #   cross(omega, r_imu)         = ( -om*ry, om*rx, 0 )
        #   cross(omega, cross(...))   = ( -om^2 * rx, -om^2 * ry, 0 )
        # So the offset acceleration is [ -om^2*rx, -om^2*ry ].
        rx = self.r_imu[0]
        ry = self.r_imu[1]
        om_sq = om*om
        ax_offset = -om_sq * rx
        ay_offset = -om_sq * ry

        # Center acceleration in body frame
        ax_center_body = ax_imu_body - ax_offset
        ay_center_body = ay_imu_body - ay_offset

        # --- 1) Integrate velocity in body frame (forward Euler) ---
        new_dx_b = dx_b + ax_center_body * dt
        new_dy_b = dy_b + ay_center_body * dt

        # --- 2) Integrate position in world frame using old velocity (dx_b, dy_b) ---
        # Convert dx_b, dy_b to world frame. We use the old velocity
        # for a simple forward-Euler:
        vx_world = dx_b * math.cos(yaw) - dy_b * math.sin(yaw)
        vy_world = dx_b * math.sin(yaw) + dy_b * math.cos(yaw)
        new_north = north + vx_world * dt
        new_west  = west  + vy_world * dt

        # --- 3) Integrate yaw using om (the sigma-point's own yaw-rate) ---
        new_yaw = self.wrap_angle(yaw + om * dt)

        # Write back
        x_out[0] = new_north
        x_out[1] = new_west
        x_out[2] = new_yaw
        x_out[3] = new_dx_b
        x_out[4] = new_dy_b
        x_out[5] = om  # keep the same body yaw-rate for now

        return x_out

    def ukf_predict(self, x, P, dt):
        """
        Unscented Kalman Filter prediction step:
          1) Generate sigma points from (x, P).
          2) Push each sigma point through f().
          3) Recompute mean and covariance.
          4) Add process noise Q.
        """
        S = self.sigma_points(x, P)  # shape: (2n+1, 6)

        S_pred = []
        for i in range(S.shape[0]):
            # We pass _unused_drive_cmd to match the signature but do not use it
            S_pred.append(self.f(S[i], dt, self.drive_velocity))
        S_pred = np.array(S_pred)

        x_pred, P_pred = self.unscented_mean_cov(S_pred)

        # Add process noise
        P_pred += self.Q

        self.x = x_pred
        self.P = P_pred

    #--------------------------------------------------------------------------
    # Measurement Models
    #--------------------------------------------------------------------------
    def h_imu(self, x_in):
        """
        We will (for simplicity) use only yaw and omega from the IMU in the measurement.
        z_imu = [yaw, omega].
        """
        return np.array([self.wrap_angle(x_in[2]), x_in[5]])

    def h_gps(self, x_in):
        """
        We measure the GPS position of the *GPS antenna*, offset from
        the robot center by self.r_gps in the robot frame.
        Let r_gps = [rx, ry, rz]. Then in the world frame:
            p_gps = p_center + R(yaw) * [rx, ry].
        We ignore z.
        """
        north_c = x_in[0]
        west_c  = x_in[1]
        yaw     = x_in[2]
        rx = self.r_gps[0]
        ry = self.r_gps[1]

        # standard 2D rotation
        north_gps = north_c + rx*math.cos(yaw) - ry*math.sin(yaw)
        west_gps  = west_c  + rx*math.sin(yaw) + ry*math.cos(yaw)
        return np.array([north_gps, west_gps])

    #--------------------------------------------------------------------------
    # UKF Update (IMU: yaw, omega)
    #--------------------------------------------------------------------------
    def ukf_update_imu(self, x, P):
        """
        Standard UKF measurement update for IMU yaw, omega.
        z_imu = [yaw_meas, omega_meas].
        """
        S = self.sigma_points(x, P)

        # Transform each sigma point
        Z = []
        for i in range(S.shape[0]):
            zi = self.h_imu(S[i])
            # wrap the yaw
            zi[0] = self.wrap_angle(zi[0])
            Z.append(zi)
        Z = np.array(Z)  # (2n+1, 2)

        # Mean predicted measurement
        z_dim = 2
        z_pred = np.zeros(z_dim)
        for i in range(2*self.n_x + 1):
            z_pred += self.Wm[i] * Z[i]
        z_pred[0] = self.wrap_angle(z_pred[0])

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

        # Measurement residual
        yaw_meas   = self.imu_meas[2]
        omega_meas = self.imu_meas[3]
        z_meas = np.array([yaw_meas, omega_meas])
        z_err = z_meas - z_pred
        z_err[0] = self.wrap_angle(z_err[0])

        # Update
        new_x = x + K @ z_err
        new_x[2] = self.wrap_angle(new_x[2])
        new_P = P - K @ S_zz @ K.T

        self.x = new_x
        self.P = 0.5 * (new_P + new_P.T)

    #--------------------------------------------------------------------------
    # UKF Update (GPS: north, west)
    #--------------------------------------------------------------------------
    def ukf_update_gps(self, x, P):
        """
        Standard UKF measurement update for GPS position.
        z_gps = [north_meas, west_meas].
        """
        S = self.sigma_points(x, P)

        # Transform each sigma point
        Z = []
        for i in range(S.shape[0]):
            Z.append(self.h_gps(S[i]))
        Z = np.array(Z)

        # Mean predicted measurement
        z_dim = 2
        z_pred = np.zeros(z_dim)
        for i in range(2*self.n_x + 1):
            z_pred += self.Wm[i] * Z[i]

        # Innovation covariance
        S_zz = np.zeros((z_dim, z_dim))
        for i in range(2*self.n_x + 1):
            dz = Z[i] - z_pred
            S_zz += self.Wc[i] * np.outer(dz, dz)
        S_zz += self.R_gps

        # Cross-covariance
        S_xz = np.zeros((self.n_x, z_dim))
        for i in range(2*self.n_x + 1):
            dx = S[i] - x
            dx[2] = self.wrap_angle(dx[2])
            dz = Z[i] - z_pred
            S_xz += self.Wc[i] * np.outer(dx, dz)

        K = S_xz @ np.linalg.inv(S_zz)

        # Measurement residual
        north_meas = self.gps_meas[0]
        west_meas  = self.gps_meas[1]
        z_meas = np.array([north_meas, west_meas])
        z_err = z_meas - z_pred

        # Update
        new_x = x + K @ z_err
        new_x[2] = self.wrap_angle(new_x[2])
        new_P = P - K @ S_zz @ K.T

        self.x = new_x
        self.P = 0.5 * (new_P + new_P.T)

    #--------------------------------------------------------------------------
    # Helpers
    #--------------------------------------------------------------------------
    def get_north_west_meters(self, lat, lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        relative to the initial GPS coordinate.
        """
        R = 6378137.0  # Earth radius in meters
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
        """Wraps angle into [-pi, pi]."""
        return (angle + math.pi) % (2.0*math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = UKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
