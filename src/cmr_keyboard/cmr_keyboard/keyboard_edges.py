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
from sklearn.linear_model import RANSACRegressor  # Import RANSAC for line fitting


class KeyboardEdges(Node):
    def __init__(self):
        super().__init__('keyboard_edges')

        self.modes = [
            'Get Keyboard In Frame',
            'Detect Key Rows',
            'Detect All Key Positoins'
        ]
        self.mode_index = 0
        
        # Publisher to send joint angle increments to ArmControllerNode
        self.joint_increment_publisher = self.create_publisher(
            Float32MultiArray, '/arm/joint_increment', 10)
        
        # Create a subscriber to '/camA/image_raw'
        self.subscription = self.create_subscription(
            Image,
            '/camA/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        # Publisher for camera images with keyboard bounding
        self.detected_publisher = self.create_publisher(
            Image, '/camA/annotated_image', qos_profile_sensor_data)
        
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
        self.keyboard_class_id = 66  # Update this ID based on your model's class definitions

        if self.keyboard_class_id not in self.class_names:
            self.get_logger().error("Keyboard class not found in model class names")
        else:
            self.get_logger().info(f"Keyboard class ID: {self.keyboard_class_id} corresponds to '{self.class_names[self.keyboard_class_id]}'")

        # Calibration to undistort fisheye image from EE camera
        self.K = np.array([[716.54878883, 0, 839.40919223],
                           [0, 713.97702137, 595.35771042],
                           [0, 0, 1]])

        self.D = np.array([-0.00271428, -0.0540361, 0.05926518, -0.0230707])

        # Define cropping fractions as class variables
        self.crop_left_fraction = 1/10
        self.crop_right_fraction = 1/5
        self.crop_top_fraction = 1/6
        self.crop_bottom_fraction = 1/8

        # Define area threshold multipliers
        self.area_threshold_multiplier_min = 7000
        self.area_threshold_multiplier_max = 1000

        # Initialize variables to store contours and current image
        self.detected_contours = []
        self.current_image = None

        # Subscriber to '/save_contours' topic
        self.save_contours_subscriber = self.create_subscription(
            Bool,
            '/save_contours',
            self.save_contours_callback,
            10)

        self.best_detection = None
        self.best_heuristic_score = float('inf')  # Initialize best heuristic score
        self.key_spacing = 0  # Initialize key spacing

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
        resized_image = cv2.resize(cv_image, (w // 2, h // 2), interpolation=cv2.INTER_LINEAR)
        annotated_image = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
        self.detected_publisher.publish(annotated_image)
        keyboard_detections = self.get_keyboard_detections(cv_image)
        
        if not keyboard_detections:
            self.get_logger().info("No Keyboard Detected")
            if self.mode_index == 0:
                self.rotate_base_joint(0.015)
                
            # We should still draw the stored best detection if available
        else:


            for keyboard_bbox in keyboard_detections:
                (x1, y1, x2, y2) = keyboard_bbox

                mid_x = (x2+x1) / 2
                if self.mode_index == 0:
                    if mid_x > (w/2) * 1.1:
                        self.rotate_base_joint(0.005)
                        return
                    else:
                        self.mode_index += 1

                # Compute the vertical midpoint of the keyboard bounding box
                mid_y = (y1 + y2) / 2

                # Create the new ROI by cropping specified portions
                roi, roi_coords = self.get_cropped_roi(cv_image, x1, y1, x2, y2)
                (cropped_x1, cropped_y1, cropped_x2, cropped_y2) = roi_coords

                # Compute area of original keyboard bbox
                bbox_width = x2 - x1
                bbox_height = y2 - y1
                keyboard_area = bbox_width * bbox_height

                # Set area threshold
                area_threshold_min = keyboard_area / self.area_threshold_multiplier_min
                area_threshold_max = keyboard_area / self.area_threshold_multiplier_max

                # Detect contours within the new ROI
                contours = self.detect_contours(roi)

                # Initialize list to store offset contours and centerpoints
                offset_contours = []
                centerpoints = []

                # Iterate through each contour and perform edge detection
                for contour in contours:
                    # Filter contours by area
                    area = cv2.contourArea(contour)

                    # Approximate the contour to reduce the number of points
                    epsilon = 0.01 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)

                    # Offset the contour coordinates by cropped_x1 and cropped_y1 to match the original image
                    approx[:, 0, 0] += cropped_x1
                    approx[:, 0, 1] += cropped_y1

                    # Append the offset contour to the list
                    offset_contours.append(approx)

                    if area < area_threshold_min:
                        continue  # Skip small contours

                    # Calculate the center point
                    M = cv2.moments(approx)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        centerpoints.append([cX, cY])

                # Detect lines among centerpoints using the modified function
                lines = self.detect_lines(centerpoints, roi_coords)

                if len(lines) == 3:
                    # Compute average slope and check variance
                    slopes = [line['slope'] for line in lines]
                    slope_variance = np.var(slopes)
                    avg_slope = np.mean(slopes)

                    # Compute vertical positions of the lines
                    intercepts = [line['intercept'] for line in lines]
                    intercepts_sorted = sorted(intercepts)
                    dy1 = intercepts_sorted[1] - intercepts_sorted[0]
                    dy2 = intercepts_sorted[2] - intercepts_sorted[1]
                    y_variance = np.var([dy1, dy2])

                    # Define heuristic score
                    heuristic_score = y_variance + slope_variance

                    # Store data for this detection
                    detection_data = {
                        'keyboard_bbox': keyboard_bbox,
                        'roi_coords': roi_coords,
                        'offset_contours': offset_contours,
                        'centerpoints': centerpoints,
                        'lines': lines,
                        'y_variance': y_variance,
                        'slope_variance': slope_variance,
                        'heuristic_score': heuristic_score
                    }
                    #self.get_logger().info(f"lines: {lines}")
                    self.get_logger().info(f"Detection heuristic score: {heuristic_score}")

                    if heuristic_score < self.best_heuristic_score:
                        self.best_detection = detection_data
                        self.best_heuristic_score = heuristic_score
                        self.get_logger().info("Found better detection with lower heuristic score.")

                    # Check if heuristic score is less than 0.02 to adjust ROI
                    if heuristic_score < 1:
                        self.get_logger().info("Heuristic score < 0.02 achieved. Adjusting ROI.")
                        # Extract top and bottom lines
                        top_line = lines[0]      # Assuming lines[0] is top, lines[1] is middle, lines[2] is bottom
                        bottom_line = lines[2]

                        # Get the uppermost y_value of the top line
                        top_y1 = top_line['line'][1]
                        top_y2 = top_line['line'][3]
                        upper_y = min(top_y1, top_y2)

                        # Get the lowermost y_value of the bottom line
                        bottom_y1 = bottom_line['line'][1]
                        bottom_y2 = bottom_line['line'][3]
                        lower_y = max(bottom_y1, bottom_y2)

                        # Update the ROI coordinates
                        new_roi_coords = (roi_coords[0], upper_y, roi_coords[2], lower_y)
                        self.get_logger().info(f"New ROI coordinates: {new_roi_coords}")

                        # Update detection_data with new ROI
                        self.best_detection['roi_coords'] = new_roi_coords

                        # Optionally, you can re-crop the ROI and redraw, but to keep it simple, we'll just update the ROI
                        # If you want to reprocess with the new ROI, additional logic is required

                else:
                    self.get_logger().info("Not enough lines detected.")

        # Draw the best detection if available
        if self.best_detection:
            #self.get_logger().info(f"best lines: {self.best_detection['lines']}")
            # Draw keyboard bounding box
            (x1, y1, x2, y2) = self.best_detection['keyboard_bbox']
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw ROI bounding box
            (cropped_x1, cropped_y1, cropped_x2, cropped_y2) = self.best_detection['roi_coords']
            cv2.rectangle(cv_image, (cropped_x1, cropped_y1), (cropped_x2, cropped_y2), (0, 0, 255), 2)

            # Draw contours
            for approx in self.best_detection['offset_contours']:
                cv2.drawContours(cv_image, [approx], -1, (255, 0, 0), 2)

            # Draw centerpoints
            for cX, cY in self.best_detection['centerpoints']:
                cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)  # Red dot

            # Draw lines
            self.draw_lines(cv_image, self.best_detection['lines'])

        # Draw yellow circles for contours near lines if heuristic_score < 0.02
        if self.best_heuristic_score < 1:
            self.get_logger().info("Drawing yellow circles for contours near lines.")
            eligible_points = []
            
            for approx in self.best_detection['offset_contours']:
                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    min_distance = float('inf')
                    closest_line = None
                    closest_point_on_line = None

                    # Find the closest line and the projection point on that line
                    for line in self.best_detection['lines']:
                        x1_line, y1_line, x2_line, y2_line = line['line']
                        
                        # Calculate the projection of (cX, cY) onto the line segment
                        line_dx = x2_line - x1_line
                        line_dy = y2_line - y1_line
                        line_length_sq = line_dx**2 + line_dy**2

                        if line_length_sq == 0:
                            continue  # Avoid division by zero

                        # Parameterize the line
                        t = ((cX - x1_line) * line_dx + (cY - y1_line) * line_dy) / line_length_sq
                        t = max(0, min(1, t))  # Clamp t to [0,1]

                        # Compute the projection point
                        proj_x = x1_line + t * line_dx
                        proj_y = y1_line + t * line_dy

                        # Compute the distance from the contour center to the projection point
                        distance = np.hypot(cX - proj_x, cY - proj_y)

                        if distance < min_distance:
                            min_distance = distance
                            closest_line = line
                            closest_point_on_line = (int(round(proj_x)), int(round(proj_y)))

                    # If the projection point is within 10 pixels of the contour center, consider it eligible
                    if min_distance <= 15 and closest_point_on_line is not None:
                        eligible_points.append({
                            'line': closest_line,
                            'proj_point': closest_point_on_line,
                            'distance': min_distance
                        })

            # Group eligible points by their corresponding lines
            lines_selected_points = {idx: [] for idx in range(len(self.best_detection['lines']))}
            for point in eligible_points:
                try:
                    line_idx = self.best_detection['lines'].index(point['line'])
                    lines_selected_points[line_idx].append(point['proj_point'])
                except ValueError:
                    self.get_logger().warning("Detected line not found in best_detection['lines'] list.")

            # Initialize a list to store final selected points
            final_selected_points = []
            desired = [10,11,12]
            di = 0
            # Process each line individually
            for line_idx, points in lines_selected_points.items():
                if not points:
                    continue

                desired_num_points = desired[di]
                di += 1
                # Prevent overlapping yellow circles within 15 pixels
                non_overlapping_points = []

                for p in points:
                    too_close = False
                    for sel_p in points:
                        if sel_p == p:
                            continue
                        dist = np.hypot(p[0] - sel_p[0], p[1] - sel_p[1])
                        if dist <= 20:
                            # Compute distances to their respective lines
                            # Find the corresponding line for both points
                            current_line = None
                            selected_line = None
                            for line in self.best_detection['lines']:
                                if p in [pt for pt in lines_selected_points[self.best_detection['lines'].index(line)]]:
                                    current_line = line
                                    break
                            for line in self.best_detection['lines']:
                                if sel_p in [pt for pt in lines_selected_points[self.best_detection['lines'].index(line)]]:
                                    selected_line = line
                                    break

                            if current_line is None or selected_line is None:
                                continue  # Cannot determine lines, skip

                            # Compute distances to their respective lines
                            distance_p = self.point_to_line_distance(
                                p[0], p[1],
                                current_line['line'][0], current_line['line'][1],
                                current_line['line'][2], current_line['line'][3]
                            )

                            distance_sel_p = self.point_to_line_distance(
                                sel_p[0], sel_p[1],
                                selected_line['line'][0], selected_line['line'][1],
                                selected_line['line'][2], selected_line['line'][3]
                            )

                            if distance_p < distance_sel_p:
                                
                                points.remove(sel_p)
                            else:
                                points.remove(p)
                                too_close = True
                                break

                    if not too_close:
                        non_overlapping_points.append(p)

                    # Sort the points from left to right based on x-coordinate
                    sorted_points = sorted(non_overlapping_points, key=lambda p: p[0])

                    # Split the sorted points into left and right halves
                    mid_index = len(sorted_points) // 2
                    left_half = sorted_points[:mid_index]
                    right_half = sorted_points[mid_index:]

                    # Function to compute average distance between adjacent points
                    def compute_avg_distance(half):
                        if len(half) < 2:
                            return None
                        distances = [np.hypot(half[i][0] - half[i-1][0], half[i][1] - half[i-1][1]) for i in range(1, len(half))]
                        return np.mean(distances)

                    # Compute average distances for left and right halves
                    avg_distance_left = compute_avg_distance(left_half)
                    avg_distance_right = compute_avg_distance(right_half)

                    
                    # Function to filter points based on average distance and 10% tolerance
                    def filter_points(points_sorted, avg_distance_left, avg_distance_right):
                        if not points_sorted or avg_distance_left is None:
                            return points_sorted
                        filtered = []
                        dleft = np.hypot(points_sorted[0][0] - points_sorted[1][0], points_sorted[0][1] - points_sorted[1][1])
                        i = len(points_sorted)-1
                        dright = np.hypot(points_sorted[i][0] - points_sorted[i-1][0], points_sorted[i][1] - points_sorted[i-1][1])
                        dl = abs(dleft-avg_distance_left)/avg_distance_left
                        dr = abs(dright-avg_distance_right)/avg_distance_right
                        if dr > dl:
                            filtered = points_sorted[:i-1]
                        else:
                            filtered = points_sorted[1:]

                        return filtered

                    filtered_points = sorted_points
                    #while len(filtered_points) > desired_num_points:
                        # Filter left and right halves
                        #filtered_points = filter_points(filtered_points, avg_distance_left, avg_distance_right)

                    # Add the filtered points for this line to the final list
                    final_selected_points.extend(filtered_points)



            # Draw yellow circles at the non-overlapping selected projection points
            for p in final_selected_points:
                cv2.circle(cv_image, p, 5, (0, 255, 255), -1)  # Yellow dot

            # Publish the annotated image
            resized_image = cv2.resize(cv_image, (w // 2, h // 2), interpolation=cv2.INTER_LINEAR)
            annotated_image = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            self.detected_publisher.publish(annotated_image)
        else:
            self.get_logger().info("No valid detection found.")

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
        #self.get_logger().info(f"Rotating base joint by {math.degrees(delta_angle):.2f} degrees.")

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

    def detect_lines(self, centerpoints, keyboard_bbox):
        """
        Detect lines among centerpoints using the approach:
        - First, find the middle row by looking for points near the vertical center of the keyboard bounding box.
        - Compute orthogonal distances of all points from the middle line.
        - Ensure top and bottom lines are at least 5% of the keyboard height away from the middle line.
        - Select the 5 closest points above and below the middle line that satisfy the distance constraint.
        - Fit the top and bottom lines using only these selected points.
        - Compute the median distance between consecutive points on all three lines.
        - Round each distance to the nearest tens place before computing the median.
        - Store the median distance in self.key_spacing.
        
        Args:
            centerpoints: List of [x, y] centerpoints.
            keyboard_bbox: (x1, y1, x2, y2) coordinates of the keyboard bounding box.
        
        Returns:
            List of lines, each line is represented as a dictionary containing line parameters.
        """
        lines = []
        points = np.array(centerpoints)

        if len(points) < 6:
            self.get_logger().warning("Not enough center points to detect lines.")
            return lines  # Not enough points to detect lines

        x1, y1, x2, y2 = keyboard_bbox
        mid_y = (y1 + y2) / 2
        bbox_height = y2 - y1
        min_distance = 0.05 * bbox_height  # 5% of the keyboard height

        # First, find points near the middle y-coordinate (15% threshold)
        middle_points = points[np.abs(points[:, 1] - mid_y) < (0.15 * bbox_height)]

        if len(middle_points) < 2:
            self.get_logger().warning("Not enough middle points to fit the middle line.")
            return lines

        # Fit line to middle_points using RANSAC
        X_middle = middle_points[:, 0].reshape(-1, 1)  # x-coordinates
        y_middle = middle_points[:, 1]  # y-coordinates

        ransac_middle = RANSACRegressor(
            min_samples=2,
            residual_threshold=5,
            max_trials=100
        )
        try:
            ransac_middle.fit(X_middle, y_middle)
        except Exception as e:
            self.get_logger().error(f"RANSAC fitting for middle row failed: {e}")
            return lines

        inlier_mask_middle = ransac_middle.inlier_mask_
        if np.sum(inlier_mask_middle) < 2:
            self.get_logger().warning("Not enough inliers for the middle line.")
            return lines

        # Compute slope and intercept of the middle line
        slope_middle = ransac_middle.estimator_.coef_[0]
        intercept_middle = ransac_middle.estimator_.intercept_

        # Store the middle line data
        x1_line = x1
        x2_line = x2
        y1_line = int(ransac_middle.predict([[x1_line]])[0])
        y2_line = int(ransac_middle.predict([[x2_line]])[0])

        lines.append({
            'line': (x1_line, y1_line, x2_line, y2_line),
            'slope': slope_middle,
            'intercept': intercept_middle,
            'inliers': np.sum(inlier_mask_middle),
            'inlier_points': middle_points[inlier_mask_middle]
        })

        self.get_logger().debug(f"Middle line fitted: (x1={x1_line}, y1={y1_line}) to (x2={x2_line}, y2={y2_line})")

        # Calculate orthogonal distances of all points from the middle line
        # Formula: |m*x - y + c| / sqrt(m^2 + 1)
        m = slope_middle
        c = intercept_middle
        if m == 0:
            # Avoid division by zero for horizontal lines
            orthogonal_distances = np.abs(points[:, 1] - c)  # |y - c|
            vertical_residuals = points[:, 1] - c
        else:
            orthogonal_distances = np.abs(m * points[:, 0] - points[:, 1] + c) / np.sqrt(m**2 + 1)
            vertical_residuals = points[:, 1] - (m * points[:, 0] + c)

        # Separate points above and below the middle line
        above_indices = np.where(vertical_residuals < 0)[0]
        below_indices = np.where(vertical_residuals > 0)[0]

        # Filter points based on the minimum orthogonal distance
        above_filtered = above_indices[orthogonal_distances[above_indices] >= min_distance]
        below_filtered = below_indices[orthogonal_distances[below_indices] >= min_distance]

        self.get_logger().debug(f"Total points above middle line: {len(above_indices)}, after filtering: {len(above_filtered)}")
        self.get_logger().debug(f"Total points below middle line: {len(below_indices)}, after filtering: {len(below_filtered)}")

        # Select the 5 closest points above the middle line
        if len(above_filtered) >= 5:
            above_sorted = above_filtered[np.argsort(orthogonal_distances[above_filtered])]
            selected_above = above_sorted[:5]
        else:
            selected_above = above_filtered  # Take all available if less than 5

        # Select the 5 closest points below the middle line
        if len(below_filtered) >= 5:
            below_sorted = below_filtered[np.argsort(orthogonal_distances[below_filtered])]
            selected_below = below_sorted[:5]
        else:
            selected_below = below_filtered  # Take all available if less than 5

        self.get_logger().debug(f"Selected above points indices: {selected_above}")
        self.get_logger().debug(f"Selected below points indices: {selected_below}")

        # Extract the selected points
        top_points = points[selected_above]
        bottom_points = points[selected_below]

        # Fit the top line using the selected top points
        if len(top_points) >= 2:
            X_top = top_points[:, 0].reshape(-1, 1)
            y_top = top_points[:, 1]
            ransac_top = RANSACRegressor(
                min_samples=2,
                residual_threshold=5,
                max_trials=100
            )
            try:
                ransac_top.fit(X_top, y_top)
                inlier_mask_top = ransac_top.inlier_mask_
                if np.sum(inlier_mask_top) >= 2:
                    slope_top = ransac_top.estimator_.coef_[0]
                    intercept_top = ransac_top.estimator_.intercept_
                    y1_top = int(ransac_top.predict([[x1_line]])[0])
                    y2_top = int(ransac_top.predict([[x2_line]])[0])
                    lines.append({
                        'line': (x1_line, y1_top, x2_line, y2_top),
                        'slope': slope_top,
                        'intercept': intercept_top,
                        'inliers': np.sum(inlier_mask_top),
                        'inlier_points': top_points[inlier_mask_top]
                    })
                    self.get_logger().debug(f"Top line fitted: (x1={x1_line}, y1={y1_top}) to (x2={x2_line}, y2={y2_top})")
            except Exception as e:
                self.get_logger().error(f"RANSAC fitting for top line failed: {e}")

        # Fit the bottom line using the selected bottom points
        if len(bottom_points) >= 2:
            X_bottom = bottom_points[:, 0].reshape(-1, 1)
            y_bottom = bottom_points[:, 1]
            ransac_bottom = RANSACRegressor(
                min_samples=2,
                residual_threshold=5,
                max_trials=100
            )
            try:
                ransac_bottom.fit(X_bottom, y_bottom)
                inlier_mask_bottom = ransac_bottom.inlier_mask_
                if np.sum(inlier_mask_bottom) >= 2:
                    slope_bottom = ransac_bottom.estimator_.coef_[0]
                    intercept_bottom = ransac_bottom.estimator_.intercept_
                    y1_bottom = int(ransac_bottom.predict([[x1_line]])[0])
                    y2_bottom = int(ransac_bottom.predict([[x2_line]])[0])
                    lines.append({
                        'line': (x1_line, y1_bottom, x2_line, y2_bottom),
                        'slope': slope_bottom,
                        'intercept': intercept_bottom,
                        'inliers': np.sum(inlier_mask_bottom),
                        'inlier_points': bottom_points[inlier_mask_bottom]
                    })
                    self.get_logger().debug(f"Bottom line fitted: (x1={x1_line}, y1={y1_bottom}) to (x2={x2_line}, y2={y2_bottom})")
            except Exception as e:
                self.get_logger().error(f"RANSAC fitting for bottom line failed: {e}")

        # Verify that all three lines have been detected
        if len(lines) == 3:
            # Compute median distance between consecutive points on all lines
            all_distances = []
            for idx, line in enumerate(lines):
                inlier_points = line['inlier_points']
                if len(inlier_points) < 2:
                    self.get_logger().warning(f"Line {idx} has less than 2 inlier points. Skipping distance calculation.")
                    continue
                # Sort the inlier points by x-coordinate to ensure consecutive points are along the line
                sorted_points = inlier_points[inlier_points[:, 0].argsort()]
                # Compute distances between consecutive points
                distances = np.linalg.norm(np.diff(sorted_points, axis=0), axis=1)
                # Round each distance to the nearest tens place
                rounded_distances = [int(round(d / 10.0) * 10) for d in distances]
                all_distances.extend(rounded_distances)
                self.get_logger().debug(f"Line {idx} distances (rounded to nearest tens): {rounded_distances}")

            if len(all_distances) == 0:
                self.get_logger().warning("No distances computed from lines. Cannot determine key spacing.")
                return lines

            # Compute the median distance from the rounded distances
            median_distance = np.median(all_distances)
            self.key_spacing = int(round(median_distance / 10.0) * 10)  # Ensuring it's an integer multiple of 10
            self.get_logger().info(f"Computed key spacing (median distance rounded to nearest tens): {self.key_spacing}")

            return lines
        else:
            self.get_logger().warning(f"Expected 3 lines, but detected {len(lines)}.")
            return []

    def draw_lines(self, image, lines):
        """
        Draw lines on the image.
        Args:
            image: The image to draw on.
            lines: List of line data dictionaries.
        """
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255)]  # Colors for the lines: Green, Blue, Red
        for idx, line_data in enumerate(lines):
            x1_line, y1_line, x2_line, y2_line = line_data['line']
            color = colors[idx % len(colors)]
            cv2.line(image, (x1_line, y1_line), (x2_line, y2_line), color, 3)
            #self.get_logger().info(f"Drew line {idx}: ({x1_line}, {y1_line}) to ({x2_line}, {y2_line}) with color {color}")

    def get_keyboard_detections(self, image):
        """
        Perform keyboard detection using YOLO and return all keyboard bounding boxes.
        """
        results = self.model(image, verbose=False)
        keyboard_detections = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                if class_id == self.keyboard_class_id and confidence >= 0.5:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    keyboard_detections.append((x1, y1, x2, y2))

        return keyboard_detections

    def get_cropped_roi(self, image, x1, y1, x2, y2):
        """
        Crop the keyboard bounding box by removing specified portions.
        Returns the cropped ROI and its coordinates relative to the original image.
        """
        # Calculate the dimensions
        bbox_width = x2 - x1
        bbox_height = y2 - y1

        # Calculate new boundaries using class variables
        new_x1 = x1 + int(bbox_width * self.crop_left_fraction)  # Remove left fraction
        new_x2 = x2 - int(bbox_width * self.crop_right_fraction)  # Remove right fraction
        new_y1 = y1 + int(bbox_height * self.crop_top_fraction)  # Remove top fraction
        new_y2 = y2 - int(bbox_height * self.crop_bottom_fraction)  # Remove bottom fraction

        # Ensure coordinates are within image boundaries
        new_x1 = max(0, new_x1)
        new_y1 = max(0, new_y1)
        new_x2 = min(image.shape[1], new_x2)
        new_y2 = min(image.shape[0], new_y2)

        # Crop the ROI
        roi = image[new_y1:new_y2, new_x1:new_x2]

        return roi, (new_x1, new_y1, new_x2, new_y2)

    def detect_contours(self, roi):
        """
        Detect contours within the given ROI.
        """
        # Convert the ROI to grayscale
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise and preserve edges
        blurred_roi = cv2.GaussianBlur(gray_roi, (5, 5), 0)

        # Perform edge detection using Canny
        edges = cv2.Canny(blurred_roi, threshold1=50, threshold2=150)

        # Find contours in the edge map
        contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    def save_contours_callback(self, msg):
        """
        Callback function for '/save_contours' topic.
        """
        if msg.data:  # If True, save contours
            self.save_contour_templates()

    def save_contour_templates(self):
        """
        Save all existing detected contours as contour image templates.
        """
        if not self.detected_contours or self.current_image is None:
            self.get_logger().info("No contours to save.")
            return

        # Create directory 'key_contours' in the home directory
        home_dir = os.path.expanduser('~')
        save_dir = os.path.join(home_dir, 'key_contours')
        os.makedirs(save_dir, exist_ok=True)

        # Use timestamp for unique filenames
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Iterate over contours and save them
        for idx, contour in enumerate(self.detected_contours):
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)

            # Ensure coordinates are within image boundaries
            x = max(0, x)
            y = max(0, y)
            x_end = min(self.current_image.shape[1], x + w)
            y_end = min(self.current_image.shape[0], y + h)

            # Crop the contour region from the image
            contour_roi = self.current_image[y:y_end, x:x_end]

            # Create a mask for the contour
            mask = np.zeros((h, w), dtype=np.uint8)
            shifted_contour = contour - [x, y]  # Shift contour coordinates to (0,0)
            cv2.drawContours(mask, [shifted_contour], -1, 255, thickness=-1)

            # Apply the mask to the contour region
            contour_image = cv2.bitwise_and(contour_roi, contour_roi, mask=mask)

            # Save the image
            filename = f"contour_{timestamp}_{idx}.png"
            filepath = os.path.join(save_dir, filename)
            cv2.imwrite(filepath, contour_image)

        self.get_logger().info(f"Saved {len(self.detected_contours)} contour templates to {save_dir}")

    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        """
        Compute the minimum distance from a point (px, py) to a line segment defined by (x1, y1) to (x2, y2).
        """
        line_mag = np.hypot(x2 - x1, y2 - y1)

        if line_mag < 1e-8:
            # The line segment is a point
            return np.hypot(px - x1, py - y1)

        # Parameter u indicates the position on the line segment closest to the point
        u = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (line_mag ** 2)
        u = max(0, min(1, u))  # Clamp u to the [0,1] range

        # Compute the closest point on the line segment to the point
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)

        # Compute the distance from the point to the closest point on the line segment
        distance = np.hypot(px - ix, py - iy)

        return distance


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
