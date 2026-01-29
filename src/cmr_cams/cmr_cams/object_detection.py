#!/usr/bin/env python3
"""
YOLOv8 Object Detection Node for Cornell Mars Rover

Detects objects (mallet, bottle) from ZED camera images and publishes
their 3D world coordinates to the autonomous navigation system.

Pipeline:
1. Subscribe to /zed/image for camera frames
2. Run YOLOv8 inference to detect mallet/bottle
3. Send bounding box center pixel to ZED depth system (/zed/point/pixel)
4. Receive 3D coordinates back (/zed/point/coordinate)
5. Transform camera coords → global coords using robot pose
6. Publish target position to /autonomy/target_object/position for navigation
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory


class YOLOv8DetectionNode(Node):
    def __init__(self):
        super().__init__('yolov8_detection_node')

        # Parameters
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('max_det', 100)
        self.declare_parameter('model_file', 'best.pt')

        conf_threshold = self.get_parameter('conf_threshold').value
        max_det = self.get_parameter('max_det').value
        model_file = self.get_parameter('model_file').value

        # Load model from cmr_cams package config directory
        package_share = get_package_share_directory('cmr_cams')
        model_path = os.path.join(package_share, 'config', model_file)

        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            raise FileNotFoundError(f'YOLO model not found at {model_path}')

        # Load the YOLOv8 model
        self.model = YOLO(model_path)
        self.model.conf = conf_threshold
        self.model.max_det = max_det

        # Setup CV bridge
        self.bridge = CvBridge()

        # Map YOLO class names to navigation object IDs
        self.target_class_ids = {
            'mallet': 5,
            'hammer': 5,
            'bottle': 6,
        }

        # Robot pose for coordinate transforms
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # Pending depth requests: {index: (cls_name, cx, cy)}
        self.pending_requests = {}
        self.request_index = 0

        # Subscriber: raw ZED images
        self.image_sub = self.create_subscription(
            Image, '/zed/image', self.image_callback, 10)

        # Subscriber: robot pose
        self.pose_sub = self.create_subscription(
            TwistStamped, '/autonomy/pose/robot/global', self.pose_callback, 10)

        # Subscriber: 3D coordinates from ZED depth
        self.coord_sub = self.create_subscription(
            Float32MultiArray, '/zed/point/coordinate', self.coordinate_callback, 10)

        # Publisher: request depth at pixel
        self.depth_request_pub = self.create_publisher(
            Int32MultiArray, '/zed/point/pixel', 10)

        # Publisher: target position for navigation
        self.target_pub = self.create_publisher(
            Twist, '/autonomy/target_object/position', 10)

        # Publisher: annotated image for debugging
        self.image_pub = self.create_publisher(
            Image, '/object_detection/annotated_image', 10)

        self.get_logger().info(f'Loaded YOLOv8 model from "{model_path}"')
        self.get_logger().info('YOLO detection node with ZED depth integration started')

    def pose_callback(self, msg: TwistStamped):
        """Update robot pose for coordinate transformation."""
        self.robot_x = msg.twist.linear.x
        self.robot_y = msg.twist.linear.y
        self.robot_theta = msg.twist.angular.z

    def image_callback(self, msg: Image):
        """Process incoming camera frame with YOLO detection."""
        # Convert to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Run inference
        results = self.model(frame, verbose=False)[0]

        # Process detections
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]

            self.get_logger().debug(f'Detected {cls_name} ({conf:.2f})')

            # Draw rectangle + label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{cls_name} {conf:.2f}', (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # If this is a target class, request 3D coordinates from ZED
            if cls_name.lower() in self.target_class_ids:
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                self.request_depth(cls_name.lower(), cx, cy)
                self.get_logger().info(f'Target detected: {cls_name} at pixel ({cx}, {cy})')

        # Publish annotated image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')

    def request_depth(self, cls_name: str, cx: int, cy: int):
        """Request 3D coordinate from ZED at bounding box center pixel."""
        self.request_index += 1
        idx = self.request_index

        # Store pending request
        self.pending_requests[idx] = (cls_name, cx, cy)

        # Send pixel coordinate request to ZED
        request = Int32MultiArray()
        request.data = [idx, cx, cy]
        self.depth_request_pub.publish(request)

    def coordinate_callback(self, msg: Float32MultiArray):
        """Receive 3D coordinates from ZED and publish to navigation."""
        data = msg.data
        if len(data) < 4:
            return

        idx = int(data[0])
        x_cam, y_cam, z_cam = data[1], data[2], data[3]

        # Check if this is a response to our request
        if idx not in self.pending_requests:
            return

        cls_name, _, _ = self.pending_requests.pop(idx)
        obj_id = self.target_class_ids.get(cls_name, 0)

        # Skip invalid depth readings
        if not np.isfinite(x_cam) or not np.isfinite(y_cam) or not np.isfinite(z_cam):
            self.get_logger().warn(f'Invalid depth for {cls_name}')
            return

        # Camera to robot frame (ZED: X forward, Y left, Z up)
        x_r = x_cam
        y_r = y_cam

        # Robot to global frame
        theta = self.robot_theta
        dx = x_r * np.cos(theta) - y_r * np.sin(theta)
        dy = x_r * np.sin(theta) + y_r * np.cos(theta)
        global_x = self.robot_x + dx
        global_y = self.robot_y + dy

        # Publish target position for navigation
        target = Twist()
        target.linear.x = float(global_x)
        target.linear.y = float(global_y)
        target.linear.z = float(obj_id)
        target.angular.z = 0.0
        self.target_pub.publish(target)

        self.get_logger().info(
            f'{cls_name} (id={obj_id}) detected at global ({global_x:.2f}, {global_y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
