import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from ultralytics import YOLO
import os
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from datetime import datetime
import math

'''
Helper class to store end effector camera parameters
and keyboard dimensions
'''
class EndEffectorCamera():
    def __init__(self):
        # Calibration to undistort fisheye image from EE camera
        # Values computed from calibration script
        self.K = np.array([[1.10231280e+03, 0, 1.23931013e+03],
                           [0, 8.80524776e+02, 5.76711558e+02],
                           [0, 0, 1]])

        self.D = np.array([-0.19872862,
                           1.00295506,
                           -2.21112719,
                           1.71318476])

        # End effector bounding box (example values)
        self.ee_box = ([1190, 400], (1210, 420))  # Modify as needed

        # Distance between center of first and last key on each row (mm)
        self.row1_length = 171.0
        self.row2_length = 152.0
        self.row3_length = 114.0
        # Distance between center of keys on each row
        self.row13_sep = 36.5
        # Key spacings
        #self.key_sep_x = 18.0
        #self.key_sep_y = 19.0
        # Distance between starting keys of each row
        self.row_12_offset = 5.0
        self.row_13_offset = 13.0
        self.row1 = ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P']
        self.row2 = ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L']
        self.row3 = ['Z', 'X', 'C', 'V', 'B', 'N', 'M']
        self.key_sep_x = ((self.row1_length/len(self.row1)) + (self.row2_length/len(self.row2)) + (self.row3_length/len(self.row3)))/3
        self.key_sep_y = self.row13_sep/2


        self.true_key_pos = {}
        self.true_key_vecs = {}

        # Create a dictionary of keys storing positions on the actual keyboard
        # Assuming Q is (0,0)
        for i, k in enumerate(self.row1):
            self.true_key_pos[k] = (i * self.key_sep_x, 0, 0)
        for i, k in enumerate(self.row2):
            self.true_key_pos[k] = (self.row_12_offset + (i * self.key_sep_x), self.key_sep_y, 1)
        for i, k in enumerate(self.row3):
            self.true_key_pos[k] = (self.row_13_offset + (i * self.key_sep_x), 2.0 * self.key_sep_y, 2)

        # Compute true relative vectors for keys
        for k1 in self.true_key_pos.keys():
            self.true_key_vecs[k1] = {}
            (x1, y1, _) = self.true_key_pos[k1]
            for k2 in self.true_key_pos.keys():
                if k1 == k2:
                    continue
                (x2, y2, _) = self.true_key_pos[k2]
                self.true_key_vecs[k1][k2] = (x2 - x1, y2 - y1)




class KeyboardEdges(Node):
    def __init__(self):
        super().__init__('zed_keyboard')
        
        # Publisher to send joint angle increments to ArmControllerNode
        self.joint_increment_publisher = self.create_publisher(
            Float32MultiArray, '/arm/joint_increment', 10)
        
        # Create a subscriber to zed left cam image
        self.subscription = self.create_subscription(Image, '/zed/image_left', self.image_callback, 10)

        # Create a subscription to the detected aruco markers to track end effector
        self.aruco_subscription = self.create_subscription(Int32MultiArray, '/zed/aruco_points', self.aruco_callback, 10)

        # Publisher for camera images with keyboard bounding
        self.detected_publisher = self.create_publisher(
            Image, '/zed/keyboard_annotation', qos_profile_sensor_data)
        
        # Publisher to request depth and plane data from a pixel coordinate
        self.depth_requester = self.create_publisher(Int32MultiArray, '/zed/depth/pixel', 10)

        # Subscriber to get the coordinate and plane normal from the requested pixel
        self.depth_listener = self.create_subscription(
            Float32MultiArray,
            '/zed/depth/coordinate',
            self.coordinate_callback,
            10
        )

        # Publisher for cropped search images
        self.cropped_publisher = self.create_publisher(
            Image, '/zed/cropped_search', qos_profile_sensor_data)
        
        self.twist_publisher = self.create_publisher(
            Twist, '/cmd_vel', 5
        )

        self.bridge = CvBridge()

        # Load the YOLOv8 model
        model_path = os.path.join("/home/cmr/cmr/terra/src/cmr_keyboard/config", 'yolov8n.pt')

        # Check if model file exists
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at: {model_path}")
            return

        # Load the YOLOv8 model
        self.model = YOLO(model_path)

        # Get class names (COCO dataset)
        self.class_names = self.model.model.names  # Dict {class_id: class_name}

        # Find the class ID for 'keyboard'
        self.keyboard_class_id = 66

        if self.keyboard_class_id not in self.class_names:
            self.get_logger().error("Keyboard class not found in model class names")
        else:
            self.get_logger().info(f"Keyboard class ID: {self.keyboard_class_id} corresponds to '{self.class_names[self.keyboard_class_id]}'")

        self.K = np.array([[657.70933821, 0.0, 605.53598505],
                           [0.0, 657.27652417, 343.9524918],
                           [0.0, 0.0, 1.0]])

        self.D = np.array([-0.00271428, -0.0540361, 0.05926518, -0.0230707])

        self.best_keyboard_detection = None #(700,860,1100,1000)
        self.best_keyboard_confidence = None #50.0

        # Store poses
        # keyboard_pos and end_effector_pos: np.array([x, y, z])
        # keyboard_normal and end_effector_normal: np.array([nx, ny, nz])
        self.keyboard_pos = None
        self.keyboard_normal = None
        self.end_effector_pos = None
        self.end_effector_normal = None
        self.end_effector_offset = [-8.0, 120.0, -225.0]
        self.desired_depth_from_keyboard = 500.0
        # Thresholds to determine "significant" changes
        self.position_threshold = 0.01  # 1 cm threshold for position change
        self.orientation_threshold = np.deg2rad(5)  # 5 degrees for orientation change

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
        
        h, w = cv_image.shape[:2]

        self.get_best_keyboard_detection(cv_image)

        if self.best_keyboard_detection is None:
            self.get_logger().info("No Keyboard Detected")
        else:
            (x1, y1, x2, y2) = self.best_keyboard_detection
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Request coordinate and plane at keyboard midpoint with id of 1
            self.request_coordinate(1, (x1+x2)//2, (y1+y2)//2)
        
        resized_image = cv2.resize(cv_image, (w // 2, h // 2), interpolation=cv2.INTER_LINEAR)
        annotated_image = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
        self.detected_publisher.publish(annotated_image)

    def aruco_callback(self, msg):
        """
        Request end effector coordinate and plane with ID of 0
        if marker with ID=5 is found.
        """
        markers = msg.data
        i = 0
        while i < len(markers)-2:
            if markers[i] == 5:
                # End effector marker detected, request pose data
                self.request_coordinate(0, markers[i+1], markers[i+2])
                return
            i += 3

    def coordinate_callback(self, msg):
        # Expecting data format: [id, x, y, z, nx, ny, nz]
        data = msg.data
        if data is None or len(data) < 4:
            return

        point_id = data[0]
        x, y, z = data[1], data[2], data[3]

        # If normals are provided
        if len(data) == 7:
            nx, ny, nz = data[4], -1.0*data[5], data[6]
        else:
            return

        if point_id == 0:
            # End effector
            old_pos = self.end_effector_pos
            old_norm = self.end_effector_normal
            self.end_effector_normal = self.normalize_vector(np.array([nx, ny, nz]))
            ee_offset = self.rotate_vector_to_match(self.end_effector_offset,self.end_effector_normal)
            self.end_effector_pos = np.array([x + ee_offset[0], y+ee_offset[1], z+ee_offset[2]])

            '''if self.keyboard_pos is not None and self.keyboard_normal is not None:
                self.check_and_compute_transformation(old_pos, old_norm,
                                                      self.end_effector_pos, self.end_effector_normal,
                                                      self.keyboard_pos, self.keyboard_normal)

            self.get_logger().info(f"End effector aruco coordinates: ({x}, {y}, {z}) normal: ({nx}, {ny}, {nz})")'''

        elif point_id == 1:
            # Keyboard
            old_pos = self.keyboard_pos
            old_norm = self.keyboard_normal
            self.keyboard_pos = np.array([x, y, z])
            self.keyboard_normal = self.normalize_vector(np.array([nx, ny, nz]))

            '''if self.end_effector_pos is not None and self.end_effector_normal is not None:
                self.check_and_compute_transformation(old_pos, old_norm,
                                                      self.end_effector_pos, self.end_effector_normal,
                                                      self.keyboard_pos, self.keyboard_normal)

            self.get_logger().info(f"Keyboard midpoint coordinates: ({x}, {y}, {z}) normal: ({nx}, {ny}, {nz})")'''

    def check_and_compute_transformation(self, old_pos, old_norm, ee_pos, ee_norm, kb_pos, kb_norm):
        """
        Check if there's a significant change in end effector or keyboard pose
        and if so, compute the required translation and rotation and print it.
        """
        if old_pos is not None and old_norm is not None:
            # Check if position or orientation changed significantly
            pos_changed = np.linalg.norm((ee_pos - kb_pos) - (old_pos - kb_pos)) > self.position_threshold
            angle_changed = self.angle_between_vectors(ee_norm, kb_norm) > self.orientation_threshold

            # Also check if the keyboard moved significantly:
            if kb_pos is not None and old_pos is not None:
                pos_changed = pos_changed or (old_pos is not None and np.linalg.norm(kb_pos - old_pos) > self.position_threshold)

            if not pos_changed and not angle_changed:
                return
        
        # Compute translation
        translation = kb_pos - ee_pos  # Vector from end effector to keyboard center
        translation[2] -= self.desired_depth_from_keyboard
        # Compute rotation to align normals
        # We want to rotate end_effector_normal to match keyboard_normal
        R_axis, R_angle = self.compute_rotation_to_align(ee_norm, kb_norm)

        # Log the transformation
        self.get_logger().info("Transformation needed:")
        self.get_logger().info(f"   Translation (x, y, z): {translation}")
        self.get_logger().info(f"   Rotation axis (nx, ny, nz): {R_axis}, angle (radians): {R_angle}, angle (degrees): {math.degrees(R_angle)}")

    def request_coordinate(self, id, x, y):
        """
        Request cartesian coordinate and plane normal of a point in the image
        """
        request_data = [int(id), int(x), int(y)]
        request = Int32MultiArray()
        request.data = request_data
        self.depth_requester.publish(request)  

    def get_best_keyboard_detection(self, image, left_crop=0.25, right_crop=0.4, top_crop=0.5, bottom_crop=0.0):
        """
        Perform keyboard detection using YOLO within a specified cropped region of the image.
        """
        # Calculate crop margins
        height, width, _ = image.shape
        x1_margin = int(width * left_crop)
        x2_margin = int(width * right_crop)
        y1_margin = int(height * top_crop)
        y2_margin = int(height * bottom_crop)

        # Define the region of interest (ROI)
        roi_x1, roi_y1, roi_x2, roi_y2 = x1_margin, y1_margin, width - x2_margin, height - y2_margin
        
        # Crop the image to the ROI
        cropped_image = image[roi_y1:roi_y2, roi_x1:roi_x2]
        self.cropped_publisher.publish(self.bridge.cv2_to_imgmsg(cropped_image, encoding='bgr8'))

        # Perform YOLO detection on the cropped image
        results = self.model(cropped_image, verbose=False)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                if class_id == self.keyboard_class_id and confidence > self.best_keyboard_confidence:
                    # Adjust coordinates back to the original image
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x1 += roi_x1
                    y1 += roi_y1
                    x2 += roi_x1
                    y2 += roi_y1

                    self.best_keyboard_detection = (x1, y1, x2, y2)
                    self.best_keyboard_confidence = confidence

    @staticmethod
    def normalize_vector(v):
        norm = np.linalg.norm(v)
        if norm < 1e-9:
            return v
        return v / norm

    @staticmethod
    def angle_between_vectors(a, b):
        a_n = a / (np.linalg.norm(a) + 1e-9)
        b_n = b / (np.linalg.norm(b) + 1e-9)
        dot = np.clip(np.dot(a_n, b_n), -1.0, 1.0)
        return math.acos(dot)
    
    @staticmethod
    def rotate_vector_axis_angle(vector, axis, angle):
        """
        Rotate a vector around a given axis by an angle using Rodrigues' formula.

        Args:
            vector (np.array): The input vector to be rotated, shape (3,).
            axis (np.array): The axis of rotation, shape (3,). Must be a unit vector.
            angle (float): The rotation angle in radians.

        Returns:
            np.array: The rotated vector, shape (3,).
        """
        # Ensure axis is normalized
        axis = axis / np.linalg.norm(axis)

        # Rodrigues' rotation formula
        vector_rotated = (vector * np.cos(angle) +
                          np.cross(axis, vector) * np.sin(angle) +
                          axis * np.dot(axis, vector) * (1 - np.cos(angle)))
        
        return vector_rotated

    @staticmethod
    def rotate_vector_to_match(v1, v2):
        """
        Rotate vector v2 using the same rotation required to align [0, 1, 0] to v1.

        Args:
            v1 (np.array): The target vector to align [0, 1, 0] to. Shape (3,).
            v2 (np.array): The vector to be rotated. Shape (3,).

        Returns:
            np.array: The rotated vector.
        """
        # Reference vector [0, 1, 0]
        ref_vector = np.array([0.0, 1.0, 0.0])

        # Normalize v1 and the reference vector
        v1_normalized = v1 / np.linalg.norm(v1)
        ref_normalized = ref_vector / np.linalg.norm(ref_vector)

        # Compute the rotation axis (cross product of ref_vector and v1)
        rotation_axis = np.cross(ref_normalized, v1_normalized)

        # Compute the rotation angle (arccos of the dot product)
        dot_product = np.dot(ref_normalized, v1_normalized)
        dot_product = np.clip(dot_product, -1.0, 1.0)  # Ensure numerical stability
        rotation_angle = np.arccos(dot_product)

        # If the angle is zero, no rotation is needed
        if np.linalg.norm(rotation_axis) < 1e-6:
            return v2  # No rotation needed, return v2 as-is

        # Normalize the rotation axis
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

        # Rotate v2 using Rodrigues' rotation formula
        rotated_vector = KeyboardEdges.rotate_vector_axis_angle(v2, rotation_axis, rotation_angle)
        return rotated_vector

    @staticmethod
    def rotate_vector_axis_angle(vector, axis, angle):
        """
        Rotate a vector around a given axis by an angle using Rodrigues' formula.

        Args:
            vector (np.array): The input vector to be rotated, shape (3,).
            axis (np.array): The axis of rotation, shape (3,). Must be a unit vector.
            angle (float): The rotation angle in radians.

        Returns:
            np.array: The rotated vector, shape (3,).
        """
        # Ensure axis is normalized
        axis = axis / np.linalg.norm(axis)

        # Rodrigues' rotation formula
        vector_rotated = (vector * np.cos(angle) +
                          np.cross(axis, vector) * np.sin(angle) +
                          axis * np.dot(axis, vector) * (1 - np.cos(angle)))
        return vector_rotated

    def compute_rotation_to_align(self, source_vec, target_vec):
        """
        Compute axis and angle to rotate source_vec to target_vec.
        Returns:
            axis (np.array): Axis of rotation (unit vector)
            angle (float): Angle in radians
        """
        source_n = self.normalize_vector(source_vec)
        target_n = self.normalize_vector(target_vec)

        dot = np.dot(source_n, target_n)
        dot = np.clip(dot, -1.0, 1.0)
        angle = math.acos(dot)

        if angle < 1e-9:
            # No rotation needed
            return np.array([0.0, 0.0, 1.0]), 0.0

        # Axis is cross product
        axis = np.cross(source_n, target_n)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-9:
            # source and target are opposite
            # pick an arbitrary perpendicular axis
            # For example, if source is close to [0,0,1], choose [0,1,0].
            arbitrary = np.array([1.0, 0.0, 0.0])
            axis = np.cross(source_n, arbitrary)
            axis = axis / (np.linalg.norm(axis) + 1e-9)
        else:
            axis = axis / axis_norm

        return axis, angle


def main(args=None):
    rclpy.init(args=args)

    keyboard_detector = KeyboardEdges()

    try:
        rclpy.spin(keyboard_detector)
    except KeyboardInterrupt:
        keyboard_detector.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        keyboard_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
