#!/usr/bin/env python3
"""
Base-station node for the URC object-detection subsystem (overview:
cmr_cams/DETECTION_TESTING.md), part of the URC 2026 Autonomous Navigation mission.

This node runs on the BASE STATION LAPTOP (not the Jetson). It taps the live
ZED feed that crosses the radio link as a ROS 2 topic, runs the Ultralytics
YOLO model ``urc_objects_v9.pt`` at a reduced sample rate, and makes the
"object detected -> stop" decision. The decision and the bounding box are sent
back to the rover / GUI as small ROS 2 messages:

  IN   /zed/image_left                  (sensor_msgs/Image)        ZED feed
  IN   /autonomy/detection/active_target(std_msgs/String)          armed class | "none"
  OUT  /autonomy/detection/result       (cmr_msgs/Detection)       single best box
  OUT  /autonomy/motion/stop            (std_msgs/Bool)            latched STOP -> controller

Design points (URC rules):
  * Only ONE object is ever reported -- the single highest-confidence valid
    detection (preferring the currently-armed class).
  * The STOP decision is autonomous and latched: once the armed object is seen
    above ``conf_threshold`` the stop holds until the rover disarms / advances
    (state machine changes the active target). No human is in the loop.
  * The rover may stop at ANY distance -- we never command it to drive closer.

The model is loaded from the cmr_cams ``config`` share directory, the same
place the existing Jetson-side detection node loads from.
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO

from cmr_msgs.msg import Detection


# YOLO class id -> name for urc_objects_v9.pt. The three URC mission objects:
#   mallet   = orange rubber mallet
#   bottle   = water bottle
#   ice_pick = rock pick hammer
MISSION_CLASSES = {'mallet', 'bottle', 'ice_pick'}
NO_TARGET = 'none'


class BaseDetectionNode(Node):
    def __init__(self):
        super().__init__('base_detection_node')

        # --- Parameters ------------------------------------------------------
        # Stop / "detected" confidence: the armed object must beat this to
        # latch the STOP. Kept higher than the display threshold so we only
        # halt the rover on a confident detection.
        self.declare_parameter('conf_threshold', 0.50)
        # Lower threshold used purely for drawing a box on the operator panel
        # (so weak detections are still visible without triggering a stop).
        self.declare_parameter('display_conf_threshold', 0.25)
        # Sample 1 in every N frames. The ZED feed is ~10 Hz, so N=5 -> ~2 Hz,
        # which keeps base-station compute modest while reliably catching the
        # objects. Tune for the laptop's GPU/CPU.
        self.declare_parameter('process_every_n_frames', 5)
        self.declare_parameter('model_file', 'urc_objects_v9.pt')
        # Inference image side (YOLO letterboxes to this). Model trained at 640.
        self.declare_parameter('imgsz', 640)
        # Feed topic + whether it is a CompressedImage (use a compressed/throttled
        # topic if raw HD1080 saturates the radio link).
        self.declare_parameter('image_topic', '/zed/image_left')
        self.declare_parameter('use_compressed', False)
        # Topics for the stop path / GUI overlay / arming.
        self.declare_parameter('active_target_topic', '/autonomy/detection/active_target')
        self.declare_parameter('result_topic', '/autonomy/detection/result')
        self.declare_parameter('stop_topic', '/autonomy/motion/stop')

        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.display_conf_threshold = self.get_parameter('display_conf_threshold').value
        self.process_every_n_frames = max(1, int(self.get_parameter('process_every_n_frames').value))
        self.imgsz = int(self.get_parameter('imgsz').value)
        model_file = self.get_parameter('model_file').value
        image_topic = self.get_parameter('image_topic').value
        self.use_compressed = self.get_parameter('use_compressed').value
        active_target_topic = self.get_parameter('active_target_topic').value
        result_topic = self.get_parameter('result_topic').value
        stop_topic = self.get_parameter('stop_topic').value

        # --- Model -----------------------------------------------------------
        package_share = get_package_share_directory('cmr_cams')
        model_path = os.path.join(package_share, 'config', model_file)
        if not os.path.exists(model_path):
            self.get_logger().error(f'Model file not found: {model_path}')
            raise FileNotFoundError(f'YOLO model not found at {model_path}')
        self.model = YOLO(model_path)

        # --- State -----------------------------------------------------------
        self.bridge = CvBridge()
        self.frame_count = 0
        # Currently-armed target class (set by the state machine when the rover
        # is within vicinity of the object's GNSS coordinate). "none" => idle.
        self.armed_target = NO_TARGET
        # STOP latch: True once the armed object is seen confidently. Held until
        # the armed target changes (rover advances / disarms).
        self.stop_latched = False

        # --- ROS I/O ---------------------------------------------------------
        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, image_topic, self.compressed_image_callback, 10)
        else:
            self.image_sub = self.create_subscription(
                Image, image_topic, self.image_callback, 10)

        self.active_target_sub = self.create_subscription(
            String, active_target_topic, self.active_target_callback, 10)

        self.result_pub = self.create_publisher(Detection, result_topic, 10)
        self.stop_pub = self.create_publisher(Bool, stop_topic, 10)

        # Re-publish the current stop state at a steady rate so the controller
        # always has a fresh value (and a freshly-started controller latches the
        # last command) even between inference samples.
        self.stop_timer = self.create_timer(0.2, self.publish_stop)

        self.get_logger().info(
            f'Base detection node up. model="{model_path}" '
            f'feed="{image_topic}"{" (compressed)" if self.use_compressed else ""} '
            f'sample=1/{self.process_every_n_frames} frames '
            f'stop_conf={self.conf_threshold:.2f}')

    # --- Callbacks -----------------------------------------------------------
    def active_target_callback(self, msg: String):
        new_target = (msg.data or NO_TARGET).strip().lower() or NO_TARGET
        if new_target != self.armed_target:
            # Arming a new target (or disarming) always clears any latched stop:
            # the rover has advanced past the previous object.
            self.get_logger().info(
                f'Armed target: {self.armed_target} -> {new_target} (stop latch cleared)')
            self.armed_target = new_target
            self.stop_latched = False
            self.publish_stop()

    def compressed_image_callback(self, msg: CompressedImage):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode CompressedImage frame')
            return
        self.process_frame(frame)

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return
        if frame.ndim == 3 and frame.shape[2] == 4:  # ZED publishes bgra8
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        self.process_frame(frame, header=msg.header)

    # --- Core ----------------------------------------------------------------
    def process_frame(self, frame, header=None):
        """Run YOLO, choose exactly one box, publish result + stop decision."""
        h, w = frame.shape[:2]
        results = self.model(
            frame, imgsz=self.imgsz, conf=self.display_conf_threshold, verbose=False)[0]

        # Collect valid (mission-class) candidate boxes.
        candidates = []  # (conf, label, x1, y1, x2, y2)
        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model.names.get(cls_id, str(cls_id)).lower()
            if label not in MISSION_CLASSES:
                continue
            conf = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            candidates.append((conf, label, x1, y1, x2, y2))

        chosen = self.choose_one(candidates)

        detected = False
        msg = Detection()
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

        if chosen is not None:
            conf, label, x1, y1, x2, y2 = chosen
            msg.label = label
            msg.class_id = next(
                (cid for cid, n in self.model.names.items() if n.lower() == label), -1)
            msg.confidence = float(conf)
            # Normalize to [0, 1] so the GUI overlay is resolution-independent.
            msg.x1 = float(np.clip(x1 / w, 0.0, 1.0))
            msg.y1 = float(np.clip(y1 / h, 0.0, 1.0))
            msg.x2 = float(np.clip(x2 / w, 0.0, 1.0))
            msg.y2 = float(np.clip(y2 / h, 0.0, 1.0))
            # The autonomous stop fires only when the chosen box IS the armed
            # object and clears the stop confidence threshold.
            detected = (
                self.armed_target in MISSION_CLASSES
                and label == self.armed_target
                and conf >= self.conf_threshold
            )
        else:
            msg.class_id = -1

        msg.detected = detected
        self.result_pub.publish(msg)

        if detected and not self.stop_latched:
            self.stop_latched = True
            self.get_logger().info(
                f'STOP: {chosen[1]} detected ({chosen[0]:.2f}) -- latching stop')
            self.publish_stop()

    def choose_one(self, candidates):
        """Pick exactly ONE detection to report.

        Prefer the currently-armed class (best proof + correct stop). If the
        armed class isn't present, fall back to the overall highest-confidence
        valid detection so the operator still sees what the model sees.
        """
        if not candidates:
            return None
        if self.armed_target in MISSION_CLASSES:
            armed = [c for c in candidates if c[1] == self.armed_target]
            if armed:
                return max(armed, key=lambda c: c[0])
        return max(candidates, key=lambda c: c[0])

    def publish_stop(self):
        msg = Bool()
        msg.data = self.stop_latched
        self.stop_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
