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
from std_msgs.msg import Bool, Float32MultiArray
from datetime import datetime

# RANSAC for robust line fitting
from sklearn.linear_model import RANSACRegressor

class KeyboardDimensions:
    def __init__(self):
        # Distance between center of first and last key on each row (mm)
        self.row1_length = 171.0
        self.row2_length = 152.0
        self.row3_length = 114.0
        # Distance between center of keys on each row
        self.row13_sep = 36.5

        # Distance between starting keys of each row
        self.row_12_offset = 5.0
        self.row_13_offset = 13.0
        self.row1 = ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P']
        self.row2 = ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L']
        self.row3 = ['Z', 'X', 'C', 'V', 'B', 'N', 'M']
        self.key_sep_x = ((self.row1_length/len(self.row1)) + 
                          (self.row2_length/len(self.row2)) + 
                          (self.row3_length/len(self.row3))) / 3
        self.key_sep_y = self.row13_sep / 2

        self.true_key_pos = {}
        self.true_key_vecs = {}

        # Create a dictionary of keys storing positions on the actual keyboard
        for i, k in enumerate(self.row1):
            self.true_key_pos[k] = (i * self.key_sep_x, 0, 0)
        for i, k in enumerate(self.row2):
            self.true_key_pos[k] = (self.row_12_offset + (i * self.key_sep_x),
                                    self.key_sep_y, 1)
        for i, k in enumerate(self.row3):
            self.true_key_pos[k] = (self.row_13_offset + (i * self.key_sep_x),
                                    2.0 * self.key_sep_y, 2)

        # Compute true relative vectors for keys
        for k1 in self.true_key_pos.keys():
            self.true_key_vecs[k1] = {}
            (x1, y1, _) = self.true_key_pos[k1]
            for k2 in self.true_key_pos.keys():
                if k1 == k2:
                    continue
                (x2, y2, _) = self.true_key_pos[k2]
                self.true_key_vecs[k1][k2] = (x2 - x1, y2 - y1)

class KeyboardLights(Node):

    def __init__(self):
        super().__init__('keyboard_lights')
        
        # Create a subscriber to '/camA/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/camA/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        # Publisher for camera images with keyboard bounding
        self.detected_publisher = self.create_publisher(
            Image, '/camA/annotated_image', qos_profile_sensor_data)

        self.bridge = CvBridge()

        # Load the YOLOv8 model
        model_path = os.path.join("/home/cmr/cmr/terra/src/cmr_keyboard/config", 'yolov8n.pt')
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at: {model_path}")
            return
        self.model = YOLO(model_path)

        # Get class names (COCO dataset or custom)
        self.class_names = self.model.model.names  # Dict {class_id: class_name}

        # Update this ID based on your model's classes
        self.keyboard_class_id = 66

        if self.keyboard_class_id not in self.class_names:
            self.get_logger().error("Keyboard class not found in model class names")
        else:
            self.get_logger().info(
                f"Keyboard class ID: {self.keyboard_class_id} "
                f"corresponds to '{self.class_names[self.keyboard_class_id]}'"
            )

        # Calibration to undistort fisheye image
        self.K = np.array([[716.54878883, 0,         839.40919223],
                           [0,         713.97702137, 595.35771042],
                           [0,           0,           1         ]])
        self.D = np.array([-0.00271428, -0.0540361, 0.05926518, -0.0230707])

        self.keyboard_detection = None
        self.best_confidence = 0.0

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
        cv_image = cv2.remap(
            image, map1, map2, interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT)

        # Get the keyboard bounding box
        detection, confidence = self.get_keyboard_detections(cv_image)

        if detection is not None:
            (x1, y1, x2, y2) = detection
            cv_image = self.highlight_contours(cv_image, x1, y1, x2, y2)
            # Draw the bounding box for keyboard
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

        # Resize annotated image for easier visualization
        resized_image = cv2.resize(cv_image, (w // 2, h // 2), interpolation=cv2.INTER_LINEAR)
        annotated_image = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
        self.detected_publisher.publish(annotated_image)

    def get_keyboard_detections(self, image):
        """
        Perform keyboard detection using YOLO and return the best keyboard bounding box.
        """
        results = self.model(image, verbose=False)
        keyboard_detection = None
        best_confidence = 0.0

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                if class_id == self.keyboard_class_id and confidence >= best_confidence:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    keyboard_detection = (x1, y1, x2, y2)
                    best_confidence = confidence
        return keyboard_detection, best_confidence

    def highlight_contours(self, image, x1, y1, x2, y2):
        """
        1. Find all contours within the bounding box.
        2. Compute center points of each contour.
        3. Remove duplicate center points (closer than 1% or 2% of bounding box width).
        4. Attempt to fit up to 6 near-horizontal lines via iterative RANSAC.
        5. Annotate the image.
        """
        self.get_logger().info(f"Keyboard bounding box: {(x1, y1, x2, y2)}")

        # Step 1: Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Step 2: Apply Gaussian Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Step 3: Threshold the image to create a binary mask
        _, binary = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
        
        # Step 4: Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        annotated_image = image.copy()

        # Collect all center points inside the bounding box
        center_points = []
        box_width = x2 - x1  # For distance threshold
        dist_threshold = 0.02 * box_width  # e.g., 2% of bounding box width

        for contour in contours:
            # Get bounding box for the contour
            cx, cy, cw, ch = cv2.boundingRect(contour)
            # Check if the contour is within the keyboard bounding box
            if cx < x1 or (cx + cw) > x2 or cy < y1 or (cy + ch) > y2:
                continue
            
            # Calculate the moments of the contour to find the center point
            M = cv2.moments(contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
                center_points.append((center_x, center_y))

        # ---------------------------
        # 1) Remove close-duplicates
        # ---------------------------
        unique_centers = []
        for c in center_points:
            # Check if 'c' is too close to any point in 'unique_centers'
            if not any(self.euclidean_distance(c, uc) < dist_threshold for uc in unique_centers):
                unique_centers.append(c)

        self.get_logger().info(f"Contour center points before filtering: {len(center_points)}, "
                               f"after filtering duplicates: {len(unique_centers)}")

        # ------------
        # 2) Fit lines
        # ------------
        # We'll fit up to 6 near-horizontal lines using iterative RANSAC:
        max_lines = 6
        all_lines = []
        points_array = np.array(unique_centers, dtype=np.float32)

        # We iterate until we have 6 lines or fewer than 3 points remain
        for _ in range(max_lines):
            if len(points_array) < 3:
                break

            # Prepare data for RANSAC: X is the x-coords, y is the y-coords
            X = points_array[:, 0].reshape(-1, 1)  # shape (N, 1)
            y = points_array[:, 1]  # shape (N,)

            ransac = RANSACRegressor(random_state=42)
            ransac.fit(X, y)

            slope = ransac.estimator_.coef_[0]
            intercept = ransac.estimator_.intercept_
            inlier_mask = ransac.inlier_mask_

            inlier_indices = np.where(inlier_mask)[0]
            outlier_indices = np.where(~inlier_mask)[0]

            # Store the fitted line parameters and inliers
            line_info = {
                'slope': slope,
                'intercept': intercept,
                'inliers': points_array[inlier_indices],
                'outliers': points_array[outlier_indices]
            }
            all_lines.append(line_info)

            # Remove the inliers from our points, so we can fit another line
            points_array = points_array[outlier_indices]

            self.get_logger().info(
                f"Line {len(all_lines)}: slope={slope:.5f}, intercept={intercept:.2f}, "
                f"inliers={len(inlier_indices)}, remaining={len(points_array)}"
            )

            # (Optional) If you only want lines that are near-horizontal,
            # you can do a slope check here and stop if slope is large.

        # 3) Draw the results
        # -------------------
        color_palette = [
            (255, 0, 0),   # Blue
            (0, 255, 0),   # Green
            (0, 0, 255),   # Red
            (255, 255, 0), # Cyan
            (255, 0, 255), # Magenta
            (0, 255, 255), # Yellow
        ]

        for i, line_info in enumerate(all_lines):
            slope = line_info['slope']
            intercept = line_info['intercept']
            inliers = line_info['inliers']
            outliers = line_info['outliers']

            # Draw inliers
            for (xx, yy) in inliers:
                cv2.circle(
                    annotated_image, (int(xx), int(yy)), 5,
                    color_palette[i % len(color_palette)], -1
                )
            # Draw outliers as well (optional)
            for (xx, yy) in outliers:
                cv2.circle(
                    annotated_image, (int(xx), int(yy)), 3,
                    (128, 128, 128), -1  # Gray for outliers
                )

            # Draw the fitted line
            min_x = int(min(inliers[:, 0])) if len(inliers) else 0
            max_x = int(max(inliers[:, 0])) if len(inliers) else 0
            # If there's only one point, min_x == max_x; so skip drawing
            if min_x != max_x:
                min_y = int(slope * min_x + intercept)
                max_y = int(slope * max_x + intercept)
                cv2.line(
                    annotated_image,
                    (min_x, min_y), (max_x, max_y),
                    color_palette[i % len(color_palette)], 2
                )

        return annotated_image

    @staticmethod
    def euclidean_distance(p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def main(args=None):
    rclpy.init(args=args)

    keyboard_detector = KeyboardLights()

    try:
        rclpy.spin(keyboard_detector)
    except KeyboardInterrupt:
        keyboard_detector.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        keyboard_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
