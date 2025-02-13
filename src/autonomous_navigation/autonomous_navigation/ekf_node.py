#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped
from cmr_msgs.msg import IMUSensorData

import math

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        ##############################################
        #### NOTE: Coordinate system              ####
        #### Global frame - x=north, y=west, z=up ####
        #### Body frame - x=forward, y=left, z=up ####
        ##############################################

        # Initialize the state vector x (13x1):
        # [x, y, z, q_x, q_y, q_z, q_w, v_x, v_y, v_z, w_x, w_y, w_z]^T
        self.x = np.zeros(13)
        self.x[6] = 1.0
        self.p = np.array([self.x[0], self.x[1], self.x[2]])
        self.q = np.array([self.x[3], self.x[4], self.x[5], self.x[6]])
        self.velocity = np.array([self.x[7], self.x[8], self.x[9]])
        self.omega = np.array([self.x[10], self.x[11], self.x[12]])

        # Initialize the covariance matrix P (13x13)
        self.P = np.zeros((13,13))
        
        # Define process noise matrix Q (13x13) (constant for now)
        self.Q = np.diag([0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001])

        self.imu_offset = ([0.038, 2.324, -179.132])

        # Initial orientation of IMU taken as zero rotation
        self.r_imu = np.array([-0.57, -0.065, 0.02])
        self.r_zed = np.array([0.0, 0.10, 0.75])
        self.r_gps = np.array([1.0, 1.0, 1.0])
        
        # ---------------------------
        # Define Measurement Matrices
        # ---------------------------
        
        # H_imu (7x13) = [0_4x3   I_4x4   0_4x6
        #                 0_3x10  I_3x3]
        self.H_imu = np.zeros((7, 13))
        self.H_imu[0:4, 3:7] = np.eye(4)       # quaternion part (rows 0-3, cols 3-6)
        self.H_imu[4:7, 10:13] = np.eye(3)       # angular velocity part (rows 4-6, cols 10-12)
        
        # R_imu (7x7): Constant covariance matrix for IMU orientation (first 4) and omega (last 3)
        self.R_imu = np.diag([0.01, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001])
        
        # H_zed (13x13) = [I_3x3   0_3x10
        #                 0_4x3   I_4x4  0_4x6
        #                 0_3x7          I_3x3 0_3x3
        #                 0_3x10               I_3x3]
        self.H_zed = np.zeros((13, 13))
        self.H_zed[0:3, 0:3] = np.eye(3)       # position part (rows 0-2, cols 0-2)
        self.H_zed[3:7, 3:7] = np.eye(4)         # quaternion part (rows 3-6, cols 3-6)
        self.H_zed[7:10, 7:10] = np.eye(3)
        self.H_zed[10:13, 10:13] = np.eye(3)

        # H_gps (2x13) = [1  0_1x12
        #                 0  1 0_1x11]
        self.H_gps = np.zeros((2, 13))
        self.H_gps[0, 0] = 1.0                 # x position
        self.H_gps[1, 1] = 1.0                 # y position

        # Store linear acceleration processed from IMU data for prediction step
        self.ak = np.zeros(3)

        # Storing various system in time seconds for dt computation
        self.last_predict_time = None
        self.last_imu_time = None
        self.last_zed_time = None
        self.last_gps_time = None

        # Store estimated translation at time of last zed message to add to
        # change in position measure from ZED
        self.last_zed_state = None
        self.last_zed_covariance = None
        
        # --- Subscriptions ---
        # IMU data subscription
        self.create_subscription(
            IMUSensorData,
            '/imu',
            self.imu_callback,
            10)
        # ZED pose subscription
        self.create_subscription(
            TwistWithCovarianceStamped,
            '/zed/pose',
            self.zed_callback,
            10)
        # GPS data subscription
        self.create_subscription(
            NavSatFix,
            '/navsatfix_data',
            self.gps_callback,
            10)
        
        # --- Publishers ---
        # Generic pose publisher
        self.pose_publisher = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        self.pub_timer = self.create_timer(0.1, self.publish_state)
        self.last_yaw_pub = 0.0

    def publish_state(self):
        """
        Publish state with converted orientation representation
        """
        pose = TwistStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.twist.linear.x = self.x[0]
        pose.twist.linear.y = self.x[1]
        pose.twist.linear.z = self.x[2]
        eul = self.quaternion_to_euler(self.q)
        pose.twist.angular.x = eul[0]
        pose.twist.angular.y = eul[1]
        '''if math.degrees(abs(self.last_yaw_pub - eul[2])) > 10:
            pose.twist.angular.z = self.last_yaw_pub
        else:
            pose.twist.angular.z = eul[2]
            self.last_yaw_pub = eul[2]'''
        pose.twist.angular.z = eul[2]

        self.pose_publisher.publish(pose)
        #self.get_logger().info(f"Position: {self.p}")
        #self.get_logger().info(f"Orientation: {eul}")
        #self.get_logger().info(f"Linear velocity: {self.velocity}")
        #self.get_logger().info(f"Angular velocity: {self.omega}")

    def imu_callback(self, msg):
        """
        IMU data callback
        """
        a_imu = np.array([msg.accx, msg.accy, msg.accz]) + self.rotate_vector_by_quaternion(np.array([0.0, 0.0, -9.81]), self.q)
        time = self.timestamp_to_float(msg.header.stamp)

        # Try prediction with previous IMU reading instead of current
        # Can swap order of next two statements maybe
        self.predict(time)
        self.ak = self.imu_acceleration_to_global(a_imu)
        
        q = self.euler_to_quaternion(np.radians(np.array([msg.anglex, msg.angley, msg.anglez]) - self.imu_offset))
        w = np.array([msg.gyrox, msg.gyroz, msg.gyroz])
        z_imu = np.zeros(7)
        z_imu[0:4] = q
        z_imu[4:7] = w
        self.update(z_imu, self.H_imu, self.R_imu)

    def zed_callback(self, msg):
        """
        ZED data callback
        TwistWithCovarianceStampedMessage
        Linear component is change in global position relative to last ZED message
        Angular component is just absolute orientation estimate
        """
        time = self.timestamp_to_float(msg.header.stamp)
        if self.last_zed_time is None:
            self.last_zed_time = time
            self.last_zed_state = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                                            msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
            self.last_zed_covariance = msg.twist.covariance
            return
        dt = time - self.last_zed_time
        if dt < 0: return

        # Predict step
        self.predict(dt)

        # Measurement vector [same as state vector]
        z_zed = np.zeros(13)
        p = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])        
        v = (p - self.last_zed_state[0:3])/dt
        eul = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        omega = (eul - self.last_zed_state[3:6])/dt
        #z_zed[0:3] = self.last_zed_state[0:3] + self.rotate_vector_by_quaternion(dp, self.last_zed_state[3:7])
        z_zed[0:3] = p
        q = self.euler_to_quaternion(eul)
        z_zed[3:7] = q
        z_zed[7:10] = v + np.cross(self.omega, self.r_zed)
        z_zed[10:13] = omega
        # Reject measurements that imply unrealistic velocities
        if np.max(v) > 2.0:
            self.get_logger().info("\nZED measurement rejected\n")
            return
        # 13x13 covariance matrix
        R_zed = np.zeros((13,13))
        R_zed[0:7, 0:7] = self.convert_zed_covariance(msg.twist.covariance, q)
        R_zed[7:13, 7:13] = (np.array(msg.twist.covariance).reshape(6,6) + np.array(self.last_zed_covariance).reshape(6,6))/dt

        # Update step
        self.update(z_zed, self.H_zed, R_zed)
        # Cache values for next callback
        self.last_zed_time = time
        self.last_zed_state[0:3] = p
        self.last_zed_state[3:6] = eul

    def gps_callback(self, msg):
        """
        GPS data callback
        """
        return
    
    def predict(self, time):
        """
        Perform the EKF prediction step

        Parameters:
            time (float): System time (in seconds) the prediction is stepping to
        """
        if self.last_predict_time is None:
            dt = 0.01
        else:
            dt = time - self.last_predict_time
            if dt < 0: return

        # Position
        p = self.p + (dt*self.velocity) + (0.5*(dt**2)*self.ak)
        # Linear velocity
        v = self.velocity + (dt*self.ak)
        # Orientation
        q = self.predict_quaternion(dt)

        # Update state variable
        self.x[0:3] = p
        #self.x[3:7] = q
        self.x[7:10] = v
        self.last_predict_time = time

        # Jacobian F
        F = np.zeros((13,13))
        # Rows 0-2 correspond to position
        F[0:3, 0:3] = np.eye(3)
        F[0:3, 7:10] = np.eye(3) * dt
        F[0:3, 10:] = np.eye(3) * (dt**2)
        # Rows 3-6 correspond to quaternion orientation
        #F[3:7, 3:7] = self.quaternion_jacobian(dt)
        # Rows 7-9 correspond to linear velocity
        F[7:10, 7:10] = np.eye(3)
        F[7:10, 10:] = np.eye(3) * dt

        # Update state covariance
        self.P = np.dot(F, np.dot(self.P, F.T)) + self.Q
        self.update_helper_variables()

    def update(self, z, H, R):
        """
        Perform the EKF update step.
        
        Parameters:
          z (np.array): Measurement vector.
          H (np.array): Measurement matrix.
          R (np.array): Measurement covariance matrix.
        """
        # Innovation (measurement residual)
        y = z - np.dot(H, self.x)
        
        # Innovation covariance
        S = np.dot(H, np.dot(self.P, H.T)) + R
        
        # Kalman Gain
        K = np.dot(self.P, np.dot(H.T, np.linalg.inv(S)))
        
        # State update
        self.x = self.x + np.dot(K, y)
        
        # Covariance update
        I = np.eye(len(self.x))
        self.P = np.dot(I - np.dot(K, H), self.P)
        self.update_helper_variables()
    
    def imu_acceleration_to_global(self, a_imu):
        """
        Convert linear acceleration from imu to global frame
        """
        wxr = np.cross(self.omega, self.r_imu)
        centripetal = np.cross(self.omega, wxr)
        v_imu = self.rotate_vector_by_quaternion(self.velocity, self.q, inverse=True) + wxr
        coriolis = np.cross(2*self.omega, v_imu)

        # Neglecting tangential acceleration for now
        
        ak = a_imu - centripetal - coriolis

        return self.rotate_vector_by_quaternion(ak, self.q)

    def predict_quaternion(self, dt):
        """
        Predict the new quaternion orientation based on the current state and timestep.

        Parameters:
            dt (float): Timestep duration.
        Returns:
            np.ndarray: Updated quaternion (normalized) [q_w, q_x, q_y, q_z].
        """
        # Create the pure quaternion from the angular velocity
        omega_q = np.array([0.0, self.omega[0], self.omega[1], self.omega[2]])
        
        # Compute the quaternion derivative: dq/dt = 0.5 * (q ⊗ omega_q)
        q_dot = 0.5 * self.quaternion_multiply(self.q, omega_q)
        
        # Update the quaternion using simple Euler integration
        q_new = self.q + q_dot * dt
        
        # Normalize the updated quaternion to avoid drift
        q_new = q_new / np.linalg.norm(q_new)
        
        return q_new

    def rotate_vector_by_quaternion(self, vector, quaternion, inverse=False):
        """
        Rotate a 3x1 vector by a quaternion.
        
        Parameters:
          vector (array-like): 3D vector.
          quaternion (array-like): Quaternion in order [q_x, q_y, q_z, q_w].
        
        Returns:
          np.array: The rotated 3D vector.
        """
        r = Rotation.from_quat(quaternion)
        if inverse:
            r = r.inv()

        return r.apply(vector)
    
    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions.
        
        Parameters:
            q1 (array-like): First quaternion [q_w, q_x, q_y, q_z].
            q2 (array-like): Second quaternion [q_w, q_x, q_y, q_z].
            
        Returns:
            np.ndarray: The product quaternion [q_w, q_x, q_y, q_z].
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])

    def quaternion_to_euler(self, quaternion, seq='xyz'):
        """
        Convert a quaternion to Euler angles.
        
        Parameters:
          quaternion (array-like): Quaternion in order [q_x, q_y, q_z, q_w].
          seq (str): Sequence of axes for Euler angles (default 'xyz').
        
        Returns:
          np.array: Euler angles in radians.
        """
        r = Rotation.from_quat(quaternion)
        return r.as_euler(seq, degrees=False)
    
    def euler_to_quaternion(self, euler, seq='xyz'):
        """
        Convert Euler angles to a quaternion.
        
        Parameters:
          euler (array-like): Euler angles in radians.
          seq (str): Sequence of axes for Euler angles (default 'xyz').
        
        Returns:
          np.array: Quaternion in order [q_x, q_y, q_z, q_w].
        """
        r = Rotation.from_euler(seq, euler, degrees=False)
        q = r.as_quat()
        return np.array(q)
    
    def timestamp_to_float(self, timestamp):
        """
        Convert a ROS2 timestamp to a float in seconds.
        
        Assumes the timestamp has 'sec' and 'nanosec' attributes.
        
        Parameters:
          timestamp: A ROS2 timestamp message.
        
        Returns:
          float: Time in seconds.
        """
        return timestamp.sec + timestamp.nanosec * 1e-9
    
    def quaternion_jacobian(self, dt):
        """
        Compute the Jacobian matrix F for the quaternion propagation.
        
        The quaternion dynamics are given by:
            dq/dt = 0.5 * Q(omega) * q
        where q = [q_w, q_x, q_y, q_z]^T and
            Q(omega) = [ [  0   -w_x  -w_y  -w_z],
                        [ w_x    0    w_z  -w_y],
                        [ w_y  -w_z    0    w_x],
                        [ w_z   w_y  -w_x    0  ] ]
        
        Using a simple Euler integration, the discrete update is:
            q_new = (I + 0.5*dt*Q(omega)) * q
        Therefore, the Jacobian is:
            F = I + 0.5 * dt * Q(omega)
        
        Parameters:
            dt (float): The timestep for integration.
            
        Returns:
            np.ndarray: A 4x4 Jacobian matrix.
        """
        # Extract angular velocity components from the state variable.
        w_x, w_y, w_z = self.omega  # Ensure self.omega is np.array([w_x, w_y, w_z])
        
        # Construct the Q(omega) matrix.
        Q = np.array([
            [0,    -w_x, -w_y, -w_z],
            [w_x,   0,    w_z, -w_y],
            [w_y,  -w_z,  0,    w_x],
            [w_z,   w_y, -w_x,  0   ]
        ])
        
        # Compute the Jacobian for the quaternion update.
        F = np.eye(4) + 0.5 * dt * Q
        return F

    def convert_zed_covariance(self, cov_6x6, quaternion):
        """
        Converts a flattened 6x6 covariance matrix into a 7x7 covariance matrix
        with quaternion orientation covariance.

        cov_6x6: A 36-element 1D array representing a 6x6 covariance matrix.
        quaternion: (4,) numpy array representing the orientation quaternion [qx, qy, qz, qw]

        Returns:
        cov_7x7: A 7x7 numpy array where:
        - Top-left 3x3 is position covariance.
        - Bottom-right 4x4 is quaternion orientation covariance.
        """
        # Reshape the input 1D array into a 6x6 covariance matrix
        cov_6x6 = np.array(cov_6x6).reshape(6, 6)

        # Extract position covariance (3x3) (Top-left of the original matrix)
        pos_cov = cov_6x6[:3, :3]

        # Extract Euler angle covariance (3x3) (Bottom-right of the original matrix)
        euler_cov = cov_6x6[3:6, 3:6]

        # Convert Euler angle covariance to quaternion covariance
        quaternion_cov = self.euler_to_quaternion_cov(euler_cov, quaternion)

        # Construct the 7x7 covariance matrix
        cov_7x7 = np.zeros((7, 7))
        cov_7x7[:3, :3] = pos_cov  # Position covariance
        cov_7x7[3:7, 3:7] = quaternion_cov  # Quaternion covariance

        return cov_7x7

    def euler_to_quaternion_cov(self, euler_cov, quaternion):
        """
        Converts a 3x3 Euler angle covariance matrix into a 4x4 quaternion covariance matrix.

        euler_cov: (3x3) numpy array representing covariance in Euler angles.
        quaternion: (4,) numpy array [qx, qy, qz, qw]

        Returns:
        quaternion_cov: (4x4) numpy array representing covariance in quaternions.
        """
        qx, qy, qz, qw = quaternion

        # Compute the Jacobian matrix
        J = 0.5 * np.array([
            [-qx, -qy, -qz],
            [ qw, -qz,  qy],
            [ qz,  qw, -qx],
            [-qy,  qx,  qw]
        ])

        # Transform Euler covariance to quaternion covariance
        quaternion_cov = J @ euler_cov @ J.T
        
        # Ensure the 4x4 matrix remains symmetric
        quaternion_cov = (quaternion_cov + quaternion_cov.T) / 2
        
        return quaternion_cov


    def update_helper_variables(self):
        """
        Helper function to update class variables p, q, velocity, and omega
        """
        self.p = np.array([self.x[0], self.x[1], self.x[2]])
        self.q = np.array([self.x[3], self.x[4], self.x[5], self.x[6]])
        self.velocity = np.array([self.x[7], self.x[8], self.x[9]])
        self.omega = np.array([self.x[10], self.x[11], self.x[12]])

def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
