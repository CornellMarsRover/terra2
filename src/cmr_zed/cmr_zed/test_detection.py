#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def _default_model_path() -> str:
    """Resolve the bundled URC YOLO model from cmr_cams' share dir, falling back
    to a string the user can override via the model_path parameter."""
    try:
        share = get_package_share_directory('cmr_cams')
        candidate = os.path.join(share, 'config', 'urc_objects_v9.pt')
        if os.path.exists(candidate):
            return candidate
    except PackageNotFoundError:
        pass
    return 'urc_objects_v9.pt'


class YOLOv8DetectionNode(Node):
    def __init__(self):
        super().__init__('yolov8_detection_node')

        # --- parameters ---
        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('max_det', 100)
        self.declare_parameter('image_topic', '/zed/image_left')
        self.declare_parameter('output_topic', '/detection_test/image')

        model_path     = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('conf_threshold').value
        max_det        = self.get_parameter('max_det').value
        image_topic    = self.get_parameter('image_topic').value
        output_topic   = self.get_parameter('output_topic').value

        # --- load the YOLOv8 model ---
        self.model = YOLO(model_path)
        self.model.conf = conf_threshold
        self.model.max_det = max_det

        # --- setup CV bridge ---
        self.bridge = CvBridge()

        # --- subscriber: raw ZED images ---
        self.sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)

        # --- publisher: annotated image ---
        self.pub = self.create_publisher(
            Image,
            output_topic,
            10)

        self.get_logger().info(
            f'Loaded YOLO model from "{model_path}" '
            f'(sub: {image_topic} -> pub: {output_topic})')

    def image_callback(self, msg: Image):
        # 1) convert to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # 2) run inference
        results = self.model(frame)[0]  # only one image → take first Results

        # 3) draw & print detections
        for box in results.boxes:
            # box.xyxy = tensor([[x1, y1, x2, y2]]), box.conf, box.cls
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]

            # print to console
            self.get_logger().info(f'Detected {cls_name} ({conf:.2f})')

            # draw rectangle + label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame,
                f'{cls_name} {conf:.2f}',
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)

        # 4) publish annotated image
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_msg.header = msg.header  # preserve timestamp, frame_id, etc.
            self.pub.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8DetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
