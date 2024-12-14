import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
import math
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from collections import Counter
import time

class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')

        self.subscription = self.create_subscription(Image, '/zed/image_left', self.image_callback, 10)
        self.plane_subscription = self.create_subscription(
            Float32MultiArray, '/zed/plane/angles', self.plane_callback, 10
        )
        self.bridge = CvBridge()
        self.plane_requester = self.create_publisher(
            Int32MultiArray, '/zed/plane/pixel', 10
        )
        self.detected_publisher = self.create_publisher(
            Image, '/zed/aruco_annotation', qos_profile_sensor_data)
        
        self.offset_publisher = self.create_publisher(Float32MultiArray, '/joint_angles/offsets', 10)
        # Intrinsic parameters (from left camera calibration)
        self.camera_matrix = np.array([[657.70933821, 0.0, 605.53598505],
                                       [0.0, 657.27652417, 343.9524918],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.array([0.34688736, 0.07662388, 0.14965771, 0.01600403])

        # Joint info log (computed from relavent aruco tag/angle)
        # Key - joint index (0 is base joint, 5 is last joint)
        # Value - (relevant fiducial id, value of depth reading to try and maximize zero position,
        # , position to move to after zeroing)
        self.joint_offsets = {
            'base': 0.0,
            'shoulder': -2.0,
            'elbow': 0.0,
            'wrist_rotate': 0.0,
            'wrist_rotate2': 8.0,
            'end_effector': -5.0
        }

        self.required_count = 50

        self.arm_homing = True


        # Dictionary to store the detected aruco markers and their (x,y) coordinates in the current ZED image
        self.current_detections = {}

        # Storing the detected rotation angles received from ZED plane detection
        self.aruco_rotations = {0: {'yaw': [], 'pitch': [], 'roll': []},
                                1: {'yaw': [], 'pitch': [], 'roll': []},
                                2: {'yaw': [], 'pitch': [], 'roll': []},
                                3: {'yaw': [], 'pitch': [], 'roll': []}}

        self.count = 0
        self.rotations_cache = {0: {'yaw': [], 'pitch': [], 'roll': []},
                                1: {'yaw': [], 'pitch': [], 'roll': []},
                                2: {'yaw': [], 'pitch': [], 'roll': []},
                                3: {'yaw': [], 'pitch': [], 'roll': []}}

        # Publisher to send joint angle increments to ArmControllerNode
        self.joint_increment_publisher = self.create_publisher(
            Float32MultiArray, '/arm/joint_increment', 10)

        self.get_logger().info("Arm Aruco Detection Node has started.")

    def image_callback(self, msg):
        
        #try:

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        annotated_image = self.detect_aruco_markers(cv_image, self.camera_matrix, self.dist_coeffs)
        
        if (annotated_image is not None):
            self.detected_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8'))

        #except Exception as e:
            #self.get_logger().error(f"Failed to process image: {e}")

    def detect_aruco_markers(self, image, camera_matrix, dist_coeffs, dictionary=cv2.aruco.DICT_6X6_50):
        if image is None or image.size == 0:
            print("Invalid image. Skipping marker detection.")
            return []

        if image.shape[-1] == 4:  # Convert RGBA to BGR
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

        
        aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary)
        parameters = cv2.aruco.DetectorParameters()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        camera_matrix = self.camera_matrix
        dist_coeffs = self.dist_coeffs
        angles = {}
        ids_in_frame = []
        if ids is not None:
            
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, camera_matrix, dist_coeffs)

            for i in range(len(ids)):
                if ids[i][0] not in self.aruco_rotations.keys():
                    continue

                ids_in_frame.append(ids[i][0])
                # Calculate centerpoint of marker
                marker_corners = corners[i][0]  # Corners are stored as a (1, 4, 2) array
                center_x = int(np.mean(marker_corners[:, 0]))  # Average of x-coordinates
                center_y = int(np.mean(marker_corners[:, 1]))  # Average of y-coordinates
                self.current_detections[ids[i][0]] = (center_x, center_y)
                '''rotation_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                yaw, pitch, roll = self.rotation_matrix_to_euler_angles(rotation_matrix)
                angles[ids[i][0]] = {}
                angles[ids[i][0]]['yaw'] = yaw
                angles[ids[i][0]]['pitch'] = pitch
                angles[ids[i][0]]['roll'] = roll
                # Extract unit vectors for X, Y, Z axes
                x_axis = rotation_matrix[:, 0]  # X-axis unit vector
                y_axis = rotation_matrix[:, 1]  # Y-axis unit vector
                z_axis = rotation_matrix[:, 2]  # Z-axis unit vector
                f_desired = (np.array([1, 0, 0]), np.array([0, 0, 1]), np.array([0, -1, 0]))
                f_current = (np.array(x_axis), np.array(y_axis), np.array(z_axis))

                yaw, _, _ = self.compute_euler_angles(f_desired, f_current)
                f_desired = self.transform_coordinate_frame(f_desired, (yaw, 0, 0))
                _, _, roll = self.compute_euler_angles(f_desired, f_current)

                self.get_logger().info(f"Z axis: {z_axis}")
                self.get_logger().info(f"Base rotation: {yaw}    Shoulder rotation: {roll}")'''
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvecs[i][0], tvecs[i][0], 0.1)
            
            if len(ids_in_frame) != 4:
                self.get_logger().info(f"Not all arucos in frame. Arucos currently in frame: {ids_in_frame}")

            elif self.arm_homing:
                self.request_planes()
            
        return image
    
    def request_planes(self):
        """
        Function to request the rotation angles of the current aruco markers detected
        request format is array of length 3N where each detected marker is requested sequentially
        with ID, x_centerpoint, y_centerpoint
        """
        
        request_data = []
        request_msg = Int32MultiArray()
        
        for marker in self.current_detections.keys():
            (x,y) = self.current_detections[marker]
            #self.get_logger().info(f"Marker ID: {marker}    x: {x}  y: {y}")
            request_data.extend([int(marker),int(x),int(y)])
        
        if request_data != []:
            request_msg.data = request_data
            self.plane_requester.publish(request_msg)

    '''
    def request_plane(self, x, y):
        pixels = [x, y, self.current_joint]
        pixel_msg = Int32MultiArray()
        pixel_msg.data = pixels
        self.plane_requester.publish(pixel_msg)
    '''

    def plane_callback(self,msg):
        """
        Store the aruco rotations requested from the ZED
        """
        angles = msg.data
        
        for i in range(0, len(angles), 4):
            id = int(angles[i])
            yaw = angles[i+1]
            pitch = angles[i+2]
            roll = angles[i+3]
            self.aruco_rotations[id]['yaw'].append(yaw)
            self.aruco_rotations[id]['pitch'].append(pitch)
            self.aruco_rotations[id]['roll'].append(roll)
        
        home = True
        if self.arm_homing:
            for marker in self.aruco_rotations.keys():

                if len(self.aruco_rotations[marker]['yaw']) < self.required_count:
                    home = False
                    self.get_logger().info(f"Marker {marker}, angles collected: {len(self.aruco_rotations[marker]['yaw'])}")
                    break

            if home:
                self.arm_homing = False
                self.execute_arm_homing()


    def execute_arm_homing(self):
        
        self.get_logger().info("HOMING ARM")
        # Get the joint correction angles
        corrections = self.filter_detections()
        
        for marker in corrections.keys():
            for angle in corrections[marker].keys():
                self.rotations_cache[marker][angle].append(corrections[marker][angle])
                if angle == 'roll' or len(self.rotations_cache[marker][angle]) < 2:
                    continue
                diff = abs(max(self.rotations_cache[marker][angle])-min(self.rotations_cache[marker][angle]))
                self.get_logger().info(f"Marker {marker} {angle} value range: {diff}")
        
        # Get the necessary corrections for each joint, corrected for difference in coordinate axis 
        base_rotate = (corrections[0]['pitch']) + self.joint_offsets['base']
        shoulder_rotate = (-1.0 * corrections[0]['yaw']) + self.joint_offsets['shoulder']
        elbow_rotate = (corrections[0]['yaw'] - corrections[1]['yaw']) + self.joint_offsets['elbow']
        wrist_rotate = (corrections[2]['pitch'] - corrections[1]['pitch']) + self.joint_offsets['wrist_rotate']
        wrist_rotate2 = (corrections[1]['yaw'] - corrections[2]['yaw']) + self.joint_offsets['wrist_rotate2']
        end_effector = (corrections[3]['pitch'] - corrections[2]['pitch']) + self.joint_offsets['end_effector']

        self.rotate_joint(5, end_effector)
        self.rotate_joint(4, wrist_rotate2)
        self.rotate_joint(3, wrist_rotate)
        self.rotate_joint(2, elbow_rotate)
        self.rotate_joint(1, shoulder_rotate)
        self.rotate_joint(0, base_rotate)
        '''self.get_logger().info(f"Joint 1: {self.aruco_rotations[1]}")
        self.get_logger().info(f"Joint 2: {self.aruco_rotations[2]}")
        pitch_avg1 = self.filtered_average(self.aruco_rotations[1]['pitch'])
        pitch_avg2 = self.filtered_average(self.aruco_rotations[2]['pitch'])
        base_rotate = (pitch_avg1+pitch_avg2)/2
        joint1 = self.filtered_average(self.aruco_rotations[1]['yaw'])
        joint2 = self.filtered_average(self.aruco_rotations[2]['yaw']) - joint1'''
        #self.get_logger().info(f"Base correction: {base_rotate}")
        #self.get_logger().info(f"Joint 1 correction: {shoulder_rotate}")
        #self.get_logger().info(f"Joint 2 correction: {joint2}")
        #self.get_logger().info(f"Aruco 2: {corrections[2]}")
        
        '''time.sleep(5)
        base = [0.0, base_rotate]
        base_msg = Float32MultiArray()
        base_msg.data = base
        shoulder = [1.0, -1.0*joint1]
        shoulder_msg = Float32MultiArray()
        shoulder_msg.data = shoulder
        elbow = [2.0, -1.0*joint2]
        elbow_msg = Float32MultiArray()
        elbow_msg.data = elbow
        self.offset_publisher.publish(base_msg)
        time.sleep(5.0)
        self.offset_publisher.publish(shoulder_msg)
        time.sleep(5.0)
        self.offset_publisher.publish(elbow_msg)
        time.sleep(5.0)'''

        # Clear the detection cache
        for id in self.aruco_rotations.keys():
            self.aruco_rotations[id] = {'yaw': [], 'pitch': [], 'roll': []}

        

    def rotate_joint(self, joint_index, angle):
        joint_names = ['base','shoulder','elbow','wrist_rotate','wrist_rotate2','end_effector']
        self.get_logger().info(f"Rotating {joint_names[joint_index]} joint by {angle} degrees")
        rotation_msg = Float32MultiArray()
        rotation = [float(joint_index), float(angle)]
        rotation_msg.data = rotation
        self.offset_publisher.publish(rotation_msg)
        # include sleep for now to make sure nothing goes crazy
        time.sleep(2.0)


    def filter_detections(self):
        """
        Removes bad detections from the current aruco detections and returns 
        a dictionary containing the current estimate of marker rotations
        """
        
        averages = {}
        for marker in self.aruco_rotations.keys():
            
            averages[marker] = {}
            for angle in self.aruco_rotations[marker].keys():

                if (len(self.aruco_rotations[marker][angle]) == 0):
                    continue

                averages[marker][angle] = np.mean(self.filter_within_std(self.aruco_rotations[marker][angle]))

        return averages

    def filter_within_std(self, data):
        """
        Filters the values in the data that lie within one standard deviation from the mean.

        Args:
            data (list of float): The input list of floats.

        Returns:
            list of float: A new list containing only the values within one standard deviation.
        """
        if not data:
            return []

        # Calculate the mean
        mean = sum(data) / len(data)
        
        # Calculate the standard deviation
        variance = sum((x - mean) ** 2 for x in data) / len(data)
        std_dev = variance ** 0.5
        
        # Determine the range within one standard deviation
        lower_bound = mean - std_dev
        upper_bound = mean + std_dev
        
        # Filter and return values within the range
        return [x for x in data if lower_bound <= x <= upper_bound]

    def filtered_average(self, values):
        """
        Filters a list of floats, removing elements not within 50% of the average,
        and computes the new average.

        Parameters:
            values (list of float): The list of numerical values.

        Returns:
            float: The new average after filtering.
        """
        if not values:
            raise ValueError("The input list cannot be empty.")
        
        # Step 1: Compute the initial average
        initial_average = sum(values) / len(values)

        # Step 2: Filter out elements outside bounds
        filtered_values = []
        for value in values:
            if initial_average - 10.0 <= value <= initial_average + 10.0:
                filtered_values.append(value)

        if not filtered_values:
            raise ValueError("All values were filtered out; no valid elements remain.")
        
        # Step 3: Compute and return the new average
        new_average = sum(filtered_values) / len(filtered_values)
        return new_average

    def rotation_matrix_to_euler_angles(self, R):
        """
        Convert a rotation matrix to Euler angles (yaw, pitch, roll).
        Assumes ZYX rotation order.

        Parameters:
            R (np.ndarray): 3x3 rotation matrix.

        Returns:
            tuple: (yaw, pitch, roll) in radians.
        """
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            yaw = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            roll = np.arctan2(R[1, 0], R[0, 0])
        else:
            yaw = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            roll = 0

        return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)
    
            
    def move_joint_to_pos(self, joint, pos):
        """
        Moves joint to position relative to current (in degrees)
        """
        
        joint_increment_msg = Float32MultiArray()
        # Assuming joint index 0 is the base joint
        swap = self.joint_info[joint][3]
        increments = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        increments[joint] = math.radians(pos*swap)

        joint_increment_msg.data = increments
        self.joint_increment_publisher.publish(joint_increment_msg)

    def increment_joint(self):
        """
        Publishes a message to increment joint angles.

        :param joint: Which joint to increment.
        """

        joint = self.current_joint
        joint_increment_msg = Float32MultiArray()
        # Assuming joint index 0 is the base joint
        swap = self.joint_info[joint][3]
        increments = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if self.curr_offset == self.search_start:
            increments[joint] = math.radians(swap * self.search_start)
            self.curr_offset += 1.0
        elif self.curr_offset >= self.search_end:
            self.get_best_offset()
            return
        else:
            self.curr_offset += 1.0
            increments[joint] = math.radians(swap)

        joint_increment_msg.data = increments
        self.joint_increment_publisher.publish(joint_increment_msg)

    def round_and_find_most_common(self, array):
        """
        Rounds all elements of a numpy array to the nearest ones place
        and returns the most commonly appearing element.
        
        Parameters:
            array (np.ndarray): Input numpy array.
        
        Returns:
            int: The most commonly appearing element after rounding.
        """
        # Round the elements of the array
        rounded_array = np.round(array).astype(int)
        
        # Flatten the array to 1D for easier processing
        flattened = rounded_array.flatten()
        
        # Find the most common element
        counter = Counter(flattened)
        most_common_element, _ = counter.most_common(1)[0]
        
        return most_common_element
    
    def compute_euler_angles(self, f1, f2):
        """
        Computes the Euler angles (ZYX order) to transform the first coordinate frame (f1)
        to the second coordinate frame (f2).

        Parameters:
            f1: tuple of np.ndarray
                Three unit vectors representing the x, y, z axes of the first frame.
            f2: tuple of np.ndarray
                Three unit vectors representing the x, y, z axes of the second frame.

        Returns:
            tuple: Euler angles (yaw, pitch, roll) in radians.
        """
        x1, y1, z1 = f1
        x2, y2, z2 = f2

        # Compute the rotation matrix
        R = np.array([
            [np.dot(x2, x1), np.dot(x2, y1), np.dot(x2, z1)],
            [np.dot(y2, x1), np.dot(y2, y1), np.dot(y2, z1)],
            [np.dot(z2, x1), np.dot(z2, y1), np.dot(z2, z1)]
        ])

        # Extract Euler angles
        # Check for gimbal lock
        if abs(R[2, 0]) != 1:
            yaw = np.arctan2(R[1, 0], R[0, 0])       # psi
            pitch = np.arcsin(-R[2, 0])              # theta
            roll = np.arctan2(R[2, 1], R[2, 2])      # phi
        else:
            # Gimbal lock case
            if R[2, 0] == -1:
                pitch = np.pi / 2
                yaw = 0
                roll = np.arctan2(R[0, 1], R[0, 2])
            else:
                pitch = -np.pi / 2
                yaw = 0
                roll = np.arctan2(-R[0, 1], -R[0, 2])

        return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


    def transform_coordinate_frame(self, frame, euler_angles):
        """
        Transforms a coordinate frame using ZYX Euler angles.

        Parameters:
            frame: tuple of np.ndarray
                Three unit vectors (x, y, z) representing the coordinate frame.
            euler_angles: tuple of float
                Euler angles (yaw, pitch, roll) in degrees, in ZYX order.

        Returns:
            tuple: Transformed coordinate frame as three unit vectors (x', y', z').
        """
        yaw, pitch, roll = np.radians(euler_angles)  # Convert degrees to radians

        # Rotation matrices for ZYX order
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        # Combined rotation matrix (ZYX order)
        R = R_z @ R_y @ R_x

        # Transform the coordinate frame
        x, y, z = frame
        x_new = R @ x
        y_new = R @ y
        z_new = R @ z

        return x_new, y_new, z_new


    def rotation_matrix_to_euler_angles(self, R):
        """
        Convert a rotation matrix to Euler angles (yaw, pitch, roll).
        Assumes ZYX rotation order.

        Parameters:
            R (np.ndarray): 3x3 rotation matrix.

        Returns:
            tuple: (yaw, pitch, roll) in radians.
        """
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            yaw = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            roll = np.arctan2(R[1, 0], R[0, 0])
        else:
            yaw = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            roll = 0

        return yaw, pitch, roll

    def rotate_v2_to_plane(self, v1, v2, v3):
        """
        Rotate v2 about v1 until v2 lies in the plane defined by v1 and v3.

        Parameters:
            v1, v2, v3: np.ndarray
                Unit vectors representing the directions.

        Returns:
            theta: float
                The angle of rotation in radians to align v2 into the plane of v1 and v3.
        """
        # Normalize input vectors
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)
        v3 = v3 / np.linalg.norm(v3)

        # Normal vector to the plane defined by v1 and v3
        n = np.cross(v1, v3)
        n = n / np.linalg.norm(n)  # Normalize the normal vector

        # Project v2 onto the plane
        v2_perp = np.dot(v2, n) * n
        v2_proj = v2 - v2_perp

        # Compute the angle between v2 and its projection
        cos_theta = np.dot(v2, v2_proj) / (np.linalg.norm(v2) * np.linalg.norm(v2_proj))
        sin_theta = np.linalg.norm(np.cross(v2, v2_proj)) / (np.linalg.norm(v2) * np.linalg.norm(v2_proj))

        # Use atan2 for a signed angle
        theta = np.arctan2(sin_theta, cos_theta)

        return theta


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
