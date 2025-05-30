import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import math
import moteus
import yaml
import os
import time
import numpy as np

class RobotArmTorqueEstimator:
    def __init__(self, L1, L2, L3, L4, L5, L6, g=9.81):
        """
        Initializes the RobotArmTorqueEstimator with given link lengths and gravity.
        
        Parameters:
            L1 to L6: Lengths of the robot arm links.
            g: Gravitational acceleration (default is 9.81 m/s^2).
        """
        # Initialize lists to store joint angles and torques data
        self.joint_angles_list = []  # Each element is a list or array of joint angles [theta1, ..., theta6]
        self.joint_torques_list = []  # Each element is a list or array of joint torques [tau1, ..., tau6]
        self.masses = [0.0, 0.0, -0.0001057928629190465, -0.00010579286291904542, 
                       -0.00010579286291904535, -0.0006060767028367527] # To store the estimated masses after computation
        
        # Store link lengths and gravity
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4
        self.L5 = L5
        self.L6 = L6
        self.g = g  # Gravitational acceleration
    
    def add_data(self, joint_angles, joint_torques):
        """
        Stores a set of joint angles and corresponding joint torques.
        
        Parameters:
            joint_angles: A list or array of joint angles [theta1, ..., theta6] in radians.
            joint_torques: A list or array of joint torques [tau1, ..., tau6] in Nm.
        """
        self.joint_angles_list.append(joint_angles)
        self.joint_torques_list.append(joint_torques)
    
    def compute_coefficients(self, joint_angles):
        """
        Computes the coefficients C_ik for the torque equations, given joint angles.
        
        Parameters:
            joint_angles: A list or array of joint angles [theta1, ..., theta6] in radians.
        
        Returns:
            A 6x6 numpy array representing the coefficients matrix C.
        """
        theta1, theta2, theta3, theta4, theta5, theta6 = joint_angles
        L1, L2, L3, L4, L5, L6 = self.L1, self.L2, self.L3, self.L4, self.L5, self.L6
        g_vec = np.array([0, 0, -self.g])  # Gravity vector
        
        # Helper functions for rotation matrices
        def rotz(theta):
            c, s = np.cos(theta), np.sin(theta)
            return np.array([
                [ c, -s,  0],
                [ s,  c,  0],
                [ 0,  0,  1]
            ])

        def roty(theta):
            c, s = np.cos(theta), np.sin(theta)
            return np.array([
                [ c,  0,  s],
                [ 0,  1,  0],
                [-s,  0,  c]
            ])

        def rotx(theta):
            c, s = np.cos(theta), np.sin(theta)
            return np.array([
                [ 1,  0,  0],
                [ 0,  c, -s],
                [ 0,  s,  c]
            ])
        
        # Initialize rotation matrices and position vectors
        T = [np.eye(3) for _ in range(7)]  # Rotation matrices from base to joint i
        p = [np.zeros(3) for _ in range(7)]  # Position vectors of each joint/link mass
        
        # Base frame (identity rotation and zero position)
        T[0] = np.eye(3)
        p[0] = np.zeros(3)
        
        # Compute rotation matrices and position vectors recursively
        # Joint 1
        T[1] = T[0] @ rotz(theta1)
        p[1] = p[0] + T[0] @ np.array([0, 0, self.L1])
        
        # Joint 2
        T[2] = T[1] @ roty(theta2)
        p[2] = p[1] + T[1] @ np.array([self.L2, 0, 0])
        
        # Joint 3
        T[3] = T[2] @ roty(theta3)
        p[3] = p[2] + T[2] @ np.array([self.L3, 0, 0])
        
        # Joint 4
        T[4] = T[3] @ rotx(theta4)
        p[4] = p[3] + T[3] @ np.array([0, self.L4, 0])
        
        # Joint 5
        T[5] = T[4] @ roty(theta5)
        p[5] = p[4] + T[4] @ np.array([0, 0, -self.L5])
        
        # Joint 6
        T[6] = T[5] @ rotx(theta6)
        p[6] = p[5] + T[5] @ np.array([self.L6, 0, 0])
        
        # Compute joint axes in the base frame
        u = [np.zeros(3) for _ in range(6)]
        u[0] = T[0] @ np.array([0, 0, 1])  # Joint 1 axis (z-axis)
        u[1] = T[1] @ np.array([0, 1, 0])  # Joint 2 axis (y-axis)
        u[2] = T[2] @ np.array([0, 1, 0])  # Joint 3 axis (y-axis)
        u[3] = T[3] @ np.array([1, 0, 0])  # Joint 4 axis (x-axis)
        u[4] = T[4] @ np.array([0, 1, 0])  # Joint 5 axis (y-axis)
        u[5] = T[5] @ np.array([1, 0, 0])  # Joint 6 axis (x-axis)
        
        # Initialize the coefficients matrix C (6x6)
        C = np.zeros((6,6))
        
        # Compute coefficients for torque equations
        for i in range(6):  # For each joint i
            for k in range(i, 6):  # For each mass k >= i
                # Vector from joint i to mass k
                r_ik = p[k+1] - p[i+1]
                # Torque due to mass m_k is C_ik * m_k
                torque_vector = np.cross(r_ik, g_vec)
                C_ik = np.dot(u[i], torque_vector)
                # Store coefficient
                C[i, k] = C_ik
        
        return C
    
    def estimate_masses(self):
        """
        Estimates the point masses for each link that best match the stored joint angles and torques data.
        Uses least squares estimation to solve for masses.
        """
        num_data = len(self.joint_angles_list)
        if num_data == 0:
            print("No data to estimate masses.")
            return
        
        # Lists to collect all equations
        A_list = []
        b_list = []
        
        for idx in range(num_data):
            joint_angles = self.joint_angles_list[idx]
            joint_torques = self.joint_torques_list[idx]
            # Compute coefficients matrix C for current joint angles
            C = self.compute_coefficients(joint_angles)
            # Append to the list of equations
            A_list.append(C)
            b_list.append(joint_torques)
        
        # Stack all equations into matrices
        A = np.vstack(A_list)  # (6N x 6) coefficient matrix
        b = np.hstack(b_list)  # (6N x 1) torque vector
        
        # Solve the linear system A * masses = b using least squares
        masses, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
        self.masses = masses  # Estimated masses
        
        #print("Estimated masses (kg):", self.masses)
    
    def compute_torques(self, joint_angles):
        """
        Computes the expected torque values in each joint based on the estimated point masses and given joint angles.
        
        Parameters:
            joint_angles: A list or array of joint angles [theta1, ..., theta6] in radians.
        
        Returns:
            A numpy array of expected joint torques [tau1, ..., tau6] in Nm.
        """
        if self.masses is None:
            print("Masses have not been estimated yet. Please run estimate_masses() first.")
            return None
        
        # Compute coefficients matrix C for the given joint angles
        C = self.compute_coefficients(joint_angles)
        # Compute torques using tau = C * masses
        tau = C @ self.masses
        return tau

    def get_joint_mass(self,i):
        return self.masses[i]

class TorqueEstimator(Node):
    def __init__(self):
        super().__init__('torque_estimator')

        # Constants
        self.max_torque = 0.7
        self.max_vel = 6.0
        self.max_acc = 5.0

        # Dictionary with motor ids for arm joints
        # Pulled from armnet but last two should be swapped
        self._joint_info = {
            "ARM_BASE": 0x09, 
            "ARM_SHOULDER": 0x0B,
            "ARM_ELBOW": 0x0A,
            "WRIST_ROTATE_1": 0x0C,
            "WRIST_ROTATE_2": 0x0E,
            "WRIST_TILT": 0x0D
        }

        # Dictionary with motor controller constants (with correct ids)
        # Key: order in kinematic chain (starting at base)
        # Value: (motor id, gear ratio, direction of rotation (-1 negative if positive rotation of motor results in negative rotation about axis, 1 if not)
        self.joint_motor_info = {
            0: (0x09, 100, 1),
            1: (0x0B, 100, -1),
            2: (0x0A, 100, 1),
            3: (0x0C, 50, 1),
            4: (0x0D, 50, 1),
            5: (0x0E, 50, 1)
        }

        # Create a subscription to the cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.arm_control_callback,
            10)

        self.transport = None
        self.servos = None
        self.motors_ready = True
        self.take_data = True
        self.data_count = 0

        self.passive_mode = [False, False, False, False, False, False]
        self.passive_velocities = [0,0,0,0,0,0]
        self.torques = None
        self.estimated_torques = None
        self.commands = []
        self.curr_angles = [0,0,0,0,0,0]
        self.motor_positions = [0,0,0,0,0,0]

        # Lenths are currently very rough estimates
        self.torque_estimator = RobotArmTorqueEstimator(5,37,40,8,10,10)
        self.increment_computes = 0
        self.initialized = False
        self.loop = asyncio.get_event_loop()

        self.loop.run_until_complete(self.__async_initialize_moteus())

        self.set_joints()

        self.get_logger().info('Arm Controller Node has been started.')

    def angle_to_motor_position(self, joint_num, angle):
        ratio = self.joint_motor_info[joint_num][1]
        swap = self.joint_motor_info[joint_num][2]
        return (angle * (ratio/360) * swap)

    def arm_control_callback(self,msg):

        if self.motors_ready:
            self.determine_passive()
            self.set_joints()
            return True
        else:
            return False

    def determine_passive(self):
        for i in range(6):
            t = self.torques[i]
            et = self.estimated_torques[i]
            if et < 0:
                et -= 0.15
            else:
                et += 0.15
            pd = abs((et-t))/et
            self.get_logger().info(f"Joint {i}: pd: {pd} torque: {t}")
            if abs(t) > self.max_torque or pd > 1.5:
                self.passive_mode[i] = True
                self.passive_velocities[i] = min(1.0*(t/self.max_torque), self.max_vel)
            else:
                self.passive_mode[i] = True
                self.passive_velocities[i] = 0


    def set_joints(self):
        return self.loop.run_until_complete(self.__async_set_drive())
    
    def convert_angle(self, angle):
        return angle * (100.0/360.0)

    async def __async_initialize_moteus(self):
        self.transport = moteus.Fdcanusb()
        self.servos = {
            servo_id : moteus.Controller(id=servo_id, transport=self.transport) for servo_id in [0x09,0x0B,0x0A,0x0C,0x0E,0x0D]
        }

        # reset servo positions
        await self.transport.cycle([x.make_stop() for x in self.servos.values()])
        await self.transport.cycle([x.make_rezero() for x in self.servos.values()])


    async def __async_set_drive(self):
        self.max_vel = 6.0
        self.max_acc = 5.0
        self.max_torque = 0.7
        self.motors_ready = False

        # Initialize Moteus transport and controllers
        commands = []
        now = time.time()
        if not self.initialized:
            for i in range(6):

                id = self.joint_motor_info[i][0]
                commands.append(self.servos[id].make_position(
                    position = 0.0,
                    query=True,
                    accel_limit=self.max_acc,
                    velocity_limit=self.max_vel,
                    maximum_torque=self.max_torque
                ))
            self.initialized = True

        else:
            for i in range(6):
                id = self.joint_motor_info[i][0]
                if self.passive_mode[i]:
                    commands.append(self.servos[id].make_position(
                        position = math.nan,
                        query=True,
                        velocity = self.passive_velocities[i],
                        accel_limit=self.max_acc,
                        velocity_limit=self.max_vel,
                        maximum_torque=self.max_torque
                    ))

                else:
                    commands.append(self.servos[id].make_position(
                        position = self.motor_positions[i],
                        query=True,
                        accel_limit=self.max_acc,
                        velocity_limit=self.max_vel,
                        maximum_torque=self.max_torque 
                ))
        
        result = await self.transport.cycle(commands)
        self.motors_ready = True
        torques = []
        angles = []
        for i in range(6):
            ratio = self.joint_motor_info[i][1]
            swap = self.joint_motor_info[i][2]
            self.motor_positions[i] = result[i].values[moteus.Register.POSITION]
            angle = result[i].values[moteus.Register.POSITION] * (360/ratio) * swap
            torque = result[i].values[moteus.Register.TORQUE] * swap
            torques.append(torque)
            angles.append(math.radians(angle))

        self.angles = angles
        self.torques = torques
        self.estimated_torques = self.torque_estimator.compute_torques(angles)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
