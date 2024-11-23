import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import roboticstoolbox as rtb
from spatialmath import SE3
import rerun as rr
from scipy.spatial.transform import Rotation as R

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        # Create the robot model
        self.robot = rtb.ERobot.URDF('arm_simplified_description/urdf/arm2.urdf')

        # Initialize joint angles to zero
        self.robot.q = np.zeros(self.robot.n)
        self.joint_transforms = None
        self.joint_positions = np.zeros(self.robot.n)

        current_pose = self.robot.fkine(self.robot.q)
        self.p_start = np.array(current_pose.t)
        self.o_start = np.array(current_pose.rpy(order='xyz', unit='rad'))

        self.initialized_ee_vec = False
        self.initial_ee_vector = None
        self.current_ee_vector = None
        self.current_ee_angle = 0.0

        self.get_logger().info(f"Number of joint angles: {self.robot.n}")

        for link in self.robot.links:
            self.get_logger().info(f"Link name: {link.name}")

        # Initialize Rerun
        rr.init("robot_visualization", spawn=True)

        # Visualize the robot in its initial configuration
        self.visualize_robot()

        # Create a subscriber to /cmd_vel
        self.command_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Create a subscription to the joint increment topic
        self.increment_subscription = self.create_subscription(
            Float32MultiArray,
            '/arm/joint_increment',
            self.joint_increment_callback,
            10)
        
        # Create a publisher for joint angles
        self.publisher_ = self.create_publisher(Float32MultiArray, '/joint_angles/desired', 10)

        
        self.intended_position = np.array([0.0, 0.0, 0.0])
        self.intended_orientation = np.array([0.0, 0.0, 0.0])

    '''def actual_angles_callback(self,msg):
        self.get_logger().info("UPDATING")
        if self.switch: self.switch = False
        else: self.switch = True
        angles = []
        for i in range(6):
            angles.append(msg.data[i])

        angles.append(msg.data[-1])
        self.robot.q = np.radians(np.array(angles))
        # Visualize the robot
        self.visualize_robot()'''

    def visualize_robot(self):
        """
        Visualizes the robot given a list of SE(3) transforms using Rerun.
        """

        self.joint_transforms = self.robot.fkine_all(self.robot.q)
        self.joint_positions = np.array([transform.t for transform in self.joint_transforms])

        joint_positions = self.joint_positions
        #self.get_logger().info("Joint Angles:")
        #self.get_logger().info(str(self.robot.q))

        # Log the joint positions as points
        rr.log("robot/basejoint", rr.Points3D(joint_positions[2], radii=0.025, colors=[0, 255, 0]))
        rr.log("robot/joints", rr.Points3D(joint_positions[3:], radii=0.025, colors=[255, 0, 0]))

        # Log the links as line segments
        rr.log("robot/links", rr.LineStrips3D(joint_positions[2:], radii=0.015, colors=[0, 0, 255]))

        # Add an orthogonal green line at the end effector to represent rotation
        end_effector_transform = self.joint_transforms[-1]
        end_effector_position = end_effector_transform.t

        # Compute the direction of the green line (orthogonal to the end effector orientation)
        end_effector_orientation = end_effector_transform.R
        x_orthogonal = end_effector_orientation[:, 0]
        y_orthogonal = end_effector_orientation[:, 1]
        z_orthogonal = end_effector_orientation[:, 2]

        # Define the green line's start and end positions
        line_start = end_effector_position
        x_end = end_effector_position + 0.05 * x_orthogonal
        y_end = end_effector_position + 0.05 * y_orthogonal
        z_end = end_effector_position + 0.05 * z_orthogonal

        # Log the end effector axis
        rr.log("robot/end_effector_orientationx", rr.LineStrips3D(
            np.array([line_start, x_end]),
            radii=0.01,
            colors=[255, 0, 0]
        ))
        rr.log("robot/end_effector_orientationy", rr.LineStrips3D(
            np.array([line_start, y_end]),
            radii=0.01,
            colors=[0, 255, 0]
        ))
        rr.log("robot/end_effector_orientationz", rr.LineStrips3D(
            np.array([line_start, z_end]),
            radii=0.01,
            colors=[0, 0, 255]
        ))

    def joint_increment_callback(self,msg):
        q_new = self.robot.q
        for i in range(6):
            q_new[i] += msg.data[i]
        
        q_new[-1] = q_new[-2]
        self.publish_joint_angles()
        self.visualize_robot()

    def compute_ik(self,dp=[0,0,0],do=[0,0,0],i=1000,t=1e-3,q0=None, q_flag=False, e=None):

        p = self.intended_position + self.p_start + np.array(dp)
        o = self.intended_orientation + self.o_start + np.array(do)
        T_pos = SE3(p) * SE3.RPY(o, order='xyz', unit='rad')

        if not q_flag:
            q0 = self.robot.q
            q0[1] -= 0.001
            q0[2] += 0.001

        solution = self.robot.ikine_LM(T_pos, q0=q0, ilimit=i, tol=t, end=e)
        return solution

    def set_target_pose_ee(self, delta_p=[0,0,0], delta_o=[0, 0, 0]):

        self.current_ee_vector = np.array(self.joint_positions[-1] - self.joint_positions[-2])
        if not self.initialized_ee_vec:
            self.initial_ee_vector = np.array(self.joint_positions[-1] - self.joint_positions[-2])
            self.initialized_ee_vec = True
        dp, do = delta_p, delta_o
        if not np.array_equal(self.initial_ee_vector, self.current_ee_vector):
            self.get_logger().info(f"Initial EE vector: {self.initial_ee_vector}")
            self.get_logger().info(f"Current EE vector: {self.current_ee_vector}")
            dp, do = self.modify_deltas(np.array(delta_p),np.array(delta_o))
        return self.set_target_pose(delta_position=dp,delta_orientation=do)
    

    def set_target_position_via_second_last_link(self, delta_p):
        """
        Adjusts the end-effector position by performing inverse kinematics
        on the joint before the end effector (second-to-last joint).

        Parameters:
            delta_o (np.array): Desired delta orientation [d_roll, d_pitch, d_yaw] in radians.

        Returns:
            success (bool): True if IK solution is found and within constraints, False otherwise.
        """
        
        if np.all(delta_p == 0):
            return True
        # Get the index of the second-to-last link
        second_last_link_index = -3  # Assuming -1 is the last link
        second_last_link = self.robot.links[second_last_link_index]
        # Get the current pose of the end effector and the second-to-last link
        current_end_effector_pose = self.robot.fkine(self.robot.q)
        current_pose_second_last = self.robot.fkine(self.robot.q, end=second_last_link)
        #do_x = delta_o[0]
        #delta_o[0] = 0.0
        # Compute the transformation from the end effector to the second-to-last link
        T_end_to_second_last = current_end_effector_pose.inv() * current_pose_second_last

        # Compute the desired end effector pose
        
        # Apply delta position and orientation in end effector's frame
        delta_transform = SE3(delta_p) * SE3.RPY(np.zeros(3), order='xyz', unit='rad')
        desired_end_effector_pose = current_end_effector_pose * delta_transform

        # Compute the desired pose of the second-to-last link
        desired_pose_second_last = desired_end_effector_pose * T_end_to_second_last


        # Perform IK to compute joint angles up to the second-to-last joint
        q = self.robot.q
        q0 = q[:len(q)-2]
        q0[1] -= 0.001
        q0[2] += 0.001

        solution = self.robot.ikine_LM(desired_pose_second_last, q0=q0, ilimit=1000, tol=1e-4, end=second_last_link)

        if solution.success:
            q_new = solution.q.tolist()

            # Set the last joint angle to achieve the desired rotation about the end effector's x-axis
            # delta_o[0] is the desired rotation about x-axis
            q_new.append(self.current_ee_angle)
            q_new.append(self.current_ee_angle)
            delta_q = np.abs(q_new - self.robot.q)
            if np.all(delta_q <= 0.3):

                # Update the robot's joint angles
                self.robot.q = np.array(q_new)

                # Visualize the robot
                self.visualize_robot()

                # Update intended position and orientation
                self.intended_position += delta_p

                # Log position and orientation errors
                current_pose = self.robot.fkine(self.robot.q)
                orientation_error = self.intended_orientation - np.array(current_pose.rpy(order='xyz', unit='rad')) + self.o_start
                position_error = self.intended_position - np.array(current_pose.t) + self.p_start

                '''self.get_logger().info(f"Intended Position: {self.intended_position}")
                self.get_logger().info(f"Intended Orientation: {self.intended_orientation}")
                self.get_logger().info(f"Position Error: {position_error}")
                self.get_logger().info(f"Orientation Error: {orientation_error}")'''

            return True


        self.get_logger().warn("IK solution not found for second-to-last link.")
        return False        

    def set_target_orientation_via_second_last_link(self, delta_o):
        """
        Adjusts the end-effector orientation by performing inverse kinematics
        on the joint before the end effector (second-to-last joint).

        Parameters:
            delta_p (np.array): Desired delta position [dx, dy, dz] in the end effector's frame.
            delta_o (np.array): Desired delta orientation [d_roll, d_pitch, d_yaw] in radians.

        Returns:
            success (bool): True if IK solution is found and within constraints, False otherwise.
        """
        # Get the index of the second-to-last link
        second_last_link_index = -3  # Assuming -1 is the last link
        second_last_link = self.robot.links[second_last_link_index]
        # Get the current pose of the end effector and the second-to-last link
        current_end_effector_pose = self.robot.fkine(self.robot.q)
        current_pose_second_last = self.robot.fkine(self.robot.q, end=second_last_link)
        do_x = delta_o[0]
        self.current_ee_angle += do_x
        delta_o[0] = 0.0
        # Compute the transformation from the end effector to the second-to-last link
        T_end_to_second_last = current_end_effector_pose.inv() * current_pose_second_last

        # Only orientation change, rotate about end effector's own origin
        delta_transform = SE3.RPY(delta_o, order='zyx', unit='rad')
        desired_end_effector_pose = current_end_effector_pose * delta_transform

        # Compute the desired pose of the second-to-last link
        desired_pose_second_last = desired_end_effector_pose * T_end_to_second_last


        # Perform IK to compute joint angles up to the second-to-last joint
        q = self.robot.q
        q0 = q[:len(q)-2]
        solution = self.robot.ikine_LM(desired_pose_second_last, q0=q0, ilimit=1000, tol=1e-6, end=second_last_link)

        if solution.success:
            q_new = solution.q.tolist()

            # Set the last joint angle to achieve the desired rotation about the end effector's x-axis
            # delta_o[0] is the desired rotation about x-axis
            q_new.append(self.current_ee_angle)
            q_new.append(self.current_ee_angle)
            delta_q = np.abs(q_new - self.robot.q)
            if np.all(delta_q <= 0.3):
                # Update the robot's joint angles
                self.robot.q = np.array(q_new)

                # Visualize the robot
                self.visualize_robot()
                self.intended_orientation += delta_o

                # Log position and orientation errors
                current_pose = self.robot.fkine(self.robot.q)
                orientation_error = self.intended_orientation - np.array(current_pose.rpy(order='xyz', unit='rad')) + self.o_start
                position_error = self.intended_position - np.array(current_pose.t) + self.p_start

                '''self.get_logger().info(f"Intended Position: {self.intended_position}")
                self.get_logger().info(f"Intended Orientation: {self.intended_orientation}")
                self.get_logger().info(f"Position Error: {position_error}")
                self.get_logger().info(f"Orientation Error: {orientation_error}")'''

                return True
            
        self.get_logger().warn("IK solution not found for second-to-last link.")
        return False
        
    def set_target_pose_via_second_last_link(self, delta_p, delta_o):
        p_success = self.set_target_position_via_second_last_link(delta_p)
        o_success = self.set_target_orientation_via_second_last_link(delta_o)
        return (p_success and o_success)

    def set_target_pose(self, delta_position=[0,0,0], delta_orientation=[0, 0, 0], end=None):
        """
        Adjusts the end-effector position and orientation by a delta, ensuring
        joint angles do not change by more than 0.2 radians.

        Parameters:
            delta_position: A list or array [dx, dy, dz] representing the desired change in position.
            delta_orientation: A list or array [d_roll, d_pitch, d_yaw] representing the desired change in orientation (default is [0, 0, 0]).

        Returns:
            success (bool): True if IK solution is found and within constraints, False otherwise.
        """
        
        '''# Get the current end-effector pose
        current_pose = self.robot.fkine(self.robot.q)

        # Extract the current position and rotation
        current_position = current_pose.t
        current_orientation = current_pose.rpy(order='xyz', unit='rad')
        self.get_logger().info(f"curr p: {current_position} curr o: {current_orientation}")'''

        pos_solution = self.compute_ik(dp=delta_position,i=1000,e=end)
        if pos_solution.success:
            q_new = pos_solution.q
            delta_q = np.abs(q_new - self.robot.q)

            if np.all(delta_q <= 0.3):
                solution = self.compute_ik(dp=delta_position,do=delta_orientation,i=2000,t=1e-5,q0=q_new,q_flag=True, e=end)
                if solution.success:
                    q_new = solution.q
                    delta_q = np.abs(q_new - self.robot.q)
                    if np.all(delta_q <= 0.3):
                        self.intended_position += delta_position
                        self.intended_orientation += delta_orientation

                        q_new[-1] = self.intended_orientation[1]
                        q_new[-2] = self.intended_orientation[1]
                        self.robot.q = q_new
                        self.visualize_robot()

                        current_pose = self.robot.fkine(self.robot.q)
                        orientation_error = self.intended_orientation - np.array(current_pose.rpy(order='xyz', unit='rad')) + self.o_start
                        position_error = self.intended_position - np.array(current_pose.t) + self.p_start

                        
                        self.get_logger().info(f"Intended Position: {self.intended_position}")
                        self.get_logger().info(f"Intended Orientation: {self.intended_orientation}")
                        self.get_logger().info(f"Position Error: {position_error}")
                        self.get_logger().info(f"Orientation Error: {orientation_error}")
                        return True
                    
                self.get_logger().info("Orientation solution failed")
        self.get_logger().info("Position solution failed")
                
        return False


    def cmd_vel_callback(self, msg):
        """
        Callback function for /cmd_vel topic.

        Parameters:
            msg (Twist): The Twist message containing linear and angular velocities.
        """
        
        # Map linear values to delta positions
        d_position = np.array([msg.linear.x, msg.linear.y, msg.linear.z])

        # Map angular values to delta orientations
        d_orientation = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

        if np.all(d_orientation == 0.0) and np.all(d_position == 0.0):
            #self.get_logger().info("no change")
            return
        # Call set_target_pose
        success = self.set_target_pose_via_second_last_link(delta_p=d_position, delta_o=d_orientation)

        if success:
            self.publish_joint_angles()

        else:
            self.get_logger().warn("IK solution not found; joint angles not updated.")

    def publish_joint_angles(self):
        # Publish joint angles in degrees
        joint_angles_degrees = np.degrees(self.robot.q)

        joint_angles_msg = Float32MultiArray()
        joint_angles_msg.data = joint_angles_degrees.tolist()

        self.publisher_.publish(joint_angles_msg)


    def modify_deltas(self, delta_position, delta_euler_angles):
        """
        Modifies delta position and delta Euler angles based on the rotation between the initial and current vectors.

        Parameters:
            initial_vector (np.array): The initial vector of form [x, y, z].
            current_vector (np.array): The current vector of form [x, y, z].
            delta_position (np.array): Desired delta position of form [dx, dy, dz].
            delta_euler_angles (np.array): Desired change in Euler angles [d_roll, d_pitch, d_yaw] (in radians).

        Returns:
            modified_delta_position (np.array): Modified delta position after applying the rotation.
            modified_delta_euler_angles (np.array): Modified delta Euler angles after applying the rotation.
        """
        initial_vector = np.asarray(self.initial_ee_vector).reshape(3)
        current_vector = np.asarray(self.current_ee_vector).reshape(3)
        delta_position = np.asarray(delta_position).reshape(3)
        delta_euler_angles = np.asarray(delta_euler_angles).reshape(3)

        # Normalize the initial and current vectors
        initial_vector = initial_vector / np.linalg.norm(initial_vector)
        current_vector = current_vector / np.linalg.norm(current_vector)

        # Compute the rotation axis and angle
        rotation_axis = np.cross(initial_vector, current_vector)
        rotation_angle = np.arccos(np.clip(np.dot(initial_vector, current_vector), -1.0, 1.0))

        # Handle case where vectors are parallel (no rotation needed)
        if np.linalg.norm(rotation_axis) < 1e-6:
            rotation_matrix = np.eye(3)
        else:
            # Normalize the rotation axis
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

            # Create a rotation matrix using scipy's Rotation class
            rotation_matrix = R.from_rotvec(rotation_angle * rotation_axis).as_matrix()

        eur = R.from_matrix(rotation_matrix).as_euler('xyz')
        self.get_logger().info(f"Euler angle rotation of EE: {eur}")

        # Apply the rotation to the delta position vector
        modified_delta_position = np.dot(rotation_matrix, delta_position)

        # Convert delta Euler angles to a rotation matrix
        delta_rotation_matrix = R.from_euler('xyz', delta_euler_angles).as_matrix()

        # Apply the rotation to the delta rotation matrix
        modified_delta_rotation_matrix = np.dot(rotation_matrix, delta_rotation_matrix)

        # Convert the modified rotation matrix back to Euler angles
        modified_delta_euler_angles = R.from_matrix(modified_delta_rotation_matrix).as_euler('xyz')

        return modified_delta_position, modified_delta_euler_angles

def main(args=None):
    rclpy.init(args=args)

    robot_arm_node = IKNode()

    rclpy.spin(robot_arm_node)

    robot_arm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
