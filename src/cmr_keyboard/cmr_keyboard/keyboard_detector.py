import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from ultralytics import YOLO
import os
from easyocr import Reader
import math
from geometry_msgs.msg import Twist

class KeyboardDetector(Node):
    def __init__(self):
        super().__init__('keyboard_detector')

        # Steps for autonomous typing
        self.modes = [
            'Looking for keyboard',
            'Rotate until Keyboard in Frame',
            'Aligning z-axis so planes are parallel',
            'Reposition until Keyboard in Frame',
            'Mapping Key Layout',
            'Aligning End Effector',
            'Typing'
        ]
        self.mode_index = 0

        # Create a subscriber to '/camA/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/camA/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        self.bridge = CvBridge()

        # Load the YOLOv8 model
        model_path = os.path.join("/home/cmr/cmr/terra/src/cmr_keyboard/config", 'yolov8n.pt')

        # Check if model file exists
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at: {model_path}")
            return

        # Load the YOLOv8 model
        self.model = YOLO(model_path)  # Load the YOLOv8 model

        # Get class names (COCO dataset)
        self.class_names = self.model.model.names  # Dict {class_id: class_name}

        # Find the class ID for 'keyboard'
        # Update this ID based on your model's class definitions
        self.keyboard_class_id = 66  # Example class ID for 'keyboard'

        if self.keyboard_class_id not in self.class_names:
            self.get_logger().error("Keyboard class not found in model class names")
        else:
            self.get_logger().info(f"Keyboard class ID: {self.keyboard_class_id} corresponds to '{self.class_names[self.keyboard_class_id]}'")

        # Initialize easyOCR reader
        self.reader = Reader(['en'], gpu=True)

        # Publisher for camera images with keyboard bounding
        self.detected_publisher = self.create_publisher(
            Image, '/camA/annotated_image', qos_profile_sensor_data)

        # Publisher to send joint angle increments to ArmControllerNode
        self.joint_increment_publisher = self.create_publisher(
            Float32MultiArray, '/arm/joint_increment', 10)
        
        # Publisher for /cmd_vel
        self.ik_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize state variables
        self.keyboard_bounding = None

        self.current_base_angle = 0.0
        self.current_last_joint_angle = 0.0

        # Angles to adjust (in radians)
        self.base_joint_increment = 0.05  # Adjust as needed
        self.last_joint_increment = 0.05  # Adjust as needed

        # Width/height ratio for alignment
        self.prev_ratio = None
        self.search_direction = 1  # 1 for increasing angle, -1 for decreasing

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

        # Dictionaries to store the currently detected keys and bad detections
        self.detected_keys = {}
        self.bad_detections = set()
        self.key_guesses = {}
        self.angle_error_threshold = 60.0  # Degrees

        self.best_detection = {}
        self.best_accuracy = 0.0

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        h, w = image.shape[:2]

        # Undistort the image
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), self.K, (w, h), cv2.CV_16SC2)
        cv_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # Draw bounding box for end effector (example, adjust as needed)
        cv2.rectangle(cv_image, tuple(self.ee_box[0]), tuple(self.ee_box[1]), (0, 255, 0), 2)



        if self.mode_index == 0:
            keyboard_bbox, ratio, cv_image, detected = self.get_best_keyboard_detection(cv_image)
            if detected:
                self.mode_index += 1
                self.get_logger().info("Keyboard detected")
            else:
                self.rotate_base_joint(self.base_joint_increment)

        elif self.mode_index == 1:
            keyboard_bbox, ratio, cv_image, detected = self.get_best_keyboard_detection(cv_image)
            if not detected:
                return
            
            
            (x1,y1,x2,y2) = keyboard_bbox
            self.get_logger().info(f"Bounding box coordinates: x1={x1} y1={y1} x2={x2} y2={y2}")

            ik_move = Twist()
            if x1 == 0:
                self.rotate_base_joint(self.base_joint_increment/5)
            elif x2 == w:
                self.rotate_base_joint(-1.0*self.base_joint_increment/5)
            elif y1 == 0:
                ik_move.linear.z = 0.01
            elif y2 == h:
                ik_move.linear.z = -0.01
            else:
                self.mode_index += 1
                self.get_logger().info("Keyboard in frame")
        
        elif self.mode_index == 2:
            

        # Clear the cached detected keys
        self.detected_keys = {}

        # Perform OCR using easyOCR
        ocr_results = self.reader.readtext(cv_image)

        for (bbox, text, prob) in ocr_results:
            if not text.isalpha() or len(text) != 1:
                continue

            # Check if the detected text matches the desired key
            detected_text = text.strip().upper()

            # Extract bounding box coordinates
            [x1, y1], [x2, y2] = bbox[0], bbox[2]

            self.detected_keys[detected_text] = ((x1 + x2) // 2, (y1 + y2) // 2)

        # If enough keys are detected, remove incorrect detections
        if len(self.detected_keys.keys()) > 3:
            self.bad_detections = self.pop_bad_detections()

        for key in self.detected_keys.keys():
            (x, y) = self.detected_keys[key]
            x1, x2, y1, y2 = x - 10, x + 10, y - 10, y + 10
            color = (255, 0, 0)
            if key in self.bad_detections:
                color = (0, 0, 255)
            # Draw bounding box around the text
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            try:
                cv2.putText(cv_image, key, (int(x1), int(y2) + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            except Exception as e:
                self.get_logger().error(f"Unable to put text on image: {e}")

        # Extrapolate unknown keys
        self.key_guesses = self.extrapolate_keys()
        for key in self.key_guesses.keys():
            (x, y) = self.key_guesses[key]
            x1, x2, y1, y2 = x - 10, x + 10, y - 10, y + 10
            color = (0, 255, 0)
            # Draw bounding box around the guessed key
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            try:
                cv2.putText(cv_image, key, (int(x1), int(y2) + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            except Exception as e:
                self.get_logger().error(f"Unable to put text on image: {e}")

        # Publish the annotated image
        resized_image = cv2.resize(cv_image, (w // 2, h // 2), interpolation=cv2.INTER_LINEAR)
        annotated_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        # Image resizing so it will load in foxglove
        #annotated_image = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
        self.detected_publisher.publish(annotated_image)

    def get_best_keyboard_detection(self, image):

        # Perform keyboard detection using YOLO
        results = self.model(image, verbose=False)

        # Initialize variables for the highest confidence keyboard detection
        highest_confidence = 0.0
        keyboard_bbox = None

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                # Check for 'keyboard' class with confidence >= 0.3
                if class_id == self.keyboard_class_id and confidence > highest_confidence:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    highest_confidence = confidence
                    keyboard_bbox = (x1, y1, x2, y2)

        if highest_confidence < 0.5:
            return None, None, False
        
        
        # Draw bounding box around the keyboard
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        bbox_width = x2 - x1
        bbox_height = y2 - y1
        ratio = bbox_width / bbox_height if bbox_height != 0 else 0
        return keyboard_bbox, ratio, image, True


    def rotate_base_joint(self, delta_angle):
        """
        Publishes a message to increment the base joint angle.

        :param delta_angle: Angle in radians to increment.
        """
        joint_increment_msg = Float32MultiArray()
        # Assuming joint index 0 is the base joint
        delta_angle *= -1.0
        increments = [delta_angle, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_increment_msg.data = increments
        self.joint_increment_publisher.publish(joint_increment_msg)
        self.current_base_angle += delta_angle
        self.get_logger().info(f"Rotating base joint by {math.degrees(delta_angle):.2f} degrees.")

    def rotate_last_joint(self, delta_angle):
        """
        Publishes a message to increment the last joint angle.

        :param delta_angle: Angle in radians to increment.
        """
        joint_increment_msg = Float32MultiArray()
        # Assuming joint index 5 is the last joint
        increments = [0.0, 0.0, 0.0, 0.0, 0.0, delta_angle]
        joint_increment_msg.data = increments
        self.joint_increment_publisher.publish(joint_increment_msg)
        self.current_last_joint_angle += delta_angle
        self.get_logger().info(f"Rotating last joint by {math.degrees(delta_angle):.2f} degrees.")

    def pop_bad_detections(self):
        """
        Remove detections that don't match the layout of the keyboard
        based on angle error threshold.

        :return: Set of bad detection keys.
        """
        bad_detections = set()

        for k1 in self.detected_keys.keys():
            count = 0
            angle_error = 0.0
            (x1, y1) = self.detected_keys[k1]
            for k2 in self.detected_keys.keys():
                if k1 == k2:
                    continue
                (x2, y2) = self.detected_keys[k2]
                curr_vec = (x2 - x1, y2 - y1)
                true_vec = self.true_key_vecs[k1].get(k2)
                if not true_vec:
                    continue
                angle_error += self.angle_between_vectors(curr_vec, true_vec)
                count += 1

            if count > 0:
                angle_error /= count
                self.get_logger().info(f"Average angle error for '{k1}': {angle_error:.2f} degrees")
                if angle_error >= self.angle_error_threshold:
                    self.get_logger().info(f"Bad detection detected for key: '{k1}'")
                    bad_detections.add(k1)

        return bad_detections

    def extrapolate_keys(self):
        """
        Extrapolate the positions of unknown keys based on detected keys.

        :return: Dictionary of guessed key positions.
        """
        key_guesses = {}

        scale_count = 0
        scale_factor = 0.0
        checked = set()
        for k1 in self.detected_keys.keys():
            if k1 in self.bad_detections:
                continue
            (x1, y1) = self.detected_keys[k1]
            checked.add(k1)
            for k2 in self.detected_keys.keys():
                if k2 in checked or k2 in self.bad_detections:
                    continue
                (x2, y2) = self.detected_keys[k2]
                v1 = self.true_key_vecs[k1].get(k2)
                if not v1:
                    continue
                v2 = (x2 - x1, y2 - y1)
                # Magnitudes of the vectors
                mag1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
                mag2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)
                if mag1 == 0:
                    continue
                factor = mag2 / mag1
                scale_factor += factor
                scale_count += 1

        if scale_count == 0:
            self.get_logger().info("Not enough detected keys to extrapolate keyboard.")
            return key_guesses

        scale_factor /= scale_count

        for k1 in self.true_key_vecs.keys():
            if k1 in self.detected_keys:
                continue
            r1 = self.true_key_pos[k1][2]
            y_true = 0.0
            true_row = 0
            x_guess = 0.0
            y_guess = 0.0
            count = 0
            for k2 in self.detected_keys.keys():
                if k2 in self.bad_detections:
                    continue
                (x, y) = self.detected_keys[k2]
                r2 = self.true_key_pos[k2][2]
                if r1 == r2:
                    y_true += y
                    true_row += 1
                vec = self.true_key_vecs[k2].get(k1)
                if not vec:
                    continue
                x_guess += x + (vec[0] * scale_factor)
                y_guess += y + (vec[1] * scale_factor)
                count += 1

            if true_row > 0:
                key_guesses[k1] = (x_guess // count, y_true // true_row)
            elif count > 0:
                key_guesses[k1] = (x_guess // count, y_guess // count)

        x_sep = key_guesses['W'][0] - key_guesses['Q'][0] 
        y_sep = key_guesses['A'][1] - key_guesses['Q'][1]
        ratio = x_sep/y_sep
        true_ratio = self.key_sep_x/self.key_sep_y
        error = (abs(true_ratio-ratio)/ratio) * 100.0
        self.get_logger().info(f"Detection accuracy: {100.0-error}%")

        return key_guesses

    def angle_between_vectors(self, v1, v2):
        """
        Calculate the angle between two vectors in degrees.

        :param v1: Tuple or list representing the first vector (x1, y1).
        :param v2: Tuple or list representing the second vector (x2, y2).
        :return: Angle in degrees.
        """
        # Dot product of the vectors
        dot_product = sum(a * b for a, b in zip(v1, v2))

        # Magnitudes of the vectors
        magnitude_v1 = math.sqrt(sum(a ** 2 for a in v1))
        magnitude_v2 = math.sqrt(sum(b ** 2 for b in v2))

        # Prevent division by zero
        if magnitude_v1 == 0 or magnitude_v2 == 0:
            return 0.0

        # Cosine of the angle
        cos_theta = dot_product / (magnitude_v1 * magnitude_v2)

        # Clamp the cosine value to the range [-1, 1] to avoid numerical issues
        cos_theta = max(-1.0, min(1.0, cos_theta))

        # Angle in degrees
        angle = math.degrees(math.acos(cos_theta))
        return angle

def main(args=None):
    rclpy.init(args=args)

    keyboard_detector = KeyboardDetector()

    try:
        rclpy.spin(keyboard_detector)
    except KeyboardInterrupt:
        keyboard_detector.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        keyboard_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
