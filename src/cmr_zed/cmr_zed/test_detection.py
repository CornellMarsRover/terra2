#!/usr/bin/env python3
"""
Minimal YOLO viz node for testing models against the live ZED feed.

Subscribes to /zed/image_left (bgra8 from zed_autonomy), runs inference, and
republishes annotated frames on /detection_test/image. Defaults to
urc_objects_v9.pt found in the workspace's src/cmr_cams/config dir (so this
node works even when cmr_cams isn't built). Override via the model_path or
model_file parameter.
"""
import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
from ultralytics import YOLO

try:
    from ament_index_python.packages import (
        get_package_share_directory,
        PackageNotFoundError,
    )
except ImportError:  # pragma: no cover
    get_package_share_directory = None
    PackageNotFoundError = Exception


def _candidate_model_paths(model_file: str):
    """Yield plausible locations for the model file, in priority order."""
    # 1) cmr_cams' installed share dir (only works when cmr_cams is built)
    if get_package_share_directory is not None:
        try:
            share = get_package_share_directory('cmr_cams')
            yield os.path.join(share, 'config', model_file)
        except PackageNotFoundError:
            pass

    # 2) Walk up from this file looking for src/cmr_cams/config/<model_file>.
    #    Handles both source-tree runs and symlink-installed builds (since the
    #    installed test_detection.py is symlinked back into the repo).
    here = os.path.realpath(__file__)
    cur = os.path.dirname(here)
    for _ in range(8):
        candidate = os.path.join(cur, 'src', 'cmr_cams', 'config', model_file)
        yield candidate
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent

    # 3) Common terra2 workspace location as a last guess
    yield os.path.expanduser(f'~/terra2/src/cmr_cams/config/{model_file}')


def _resolve_model_path(model_path: str, model_file: str) -> str:
    """Prefer absolute model_path if it exists, else search candidate locations."""
    if model_path and os.path.exists(model_path):
        return model_path
    for candidate in _candidate_model_paths(model_file):
        if os.path.exists(candidate):
            return candidate
    # Fallback: return whatever was requested so YOLO raises a clear error.
    return model_path or model_file


class YOLOTestDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_test_detection_node')

        self.declare_parameter('model_path', '')
        self.declare_parameter('model_file', 'urc_objects_v9.pt')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('max_det', 100)
        self.declare_parameter('image_topic', '/zed/image_left')
        self.declare_parameter('output_topic', '/detection_test/image')
        self.declare_parameter('process_every_n_frames', 1)

        model_path     = self.get_parameter('model_path').value
        model_file     = self.get_parameter('model_file').value
        conf_threshold = self.get_parameter('conf_threshold').value
        max_det        = self.get_parameter('max_det').value
        image_topic    = self.get_parameter('image_topic').value
        output_topic   = self.get_parameter('output_topic').value
        self.process_every_n_frames = max(1, int(
            self.get_parameter('process_every_n_frames').value))

        resolved = _resolve_model_path(model_path, model_file)
        self.model = YOLO(resolved)
        self.model.conf = conf_threshold
        self.model.max_det = max_det

        self.bridge = CvBridge()
        self.frame_count = 0

        self.sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f'Loaded YOLO model from "{resolved}" '
            f'(sub: {image_topic} -> pub: {output_topic}, '
            f'process_every_n_frames={self.process_every_n_frames})')

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if frame.ndim == 3 and frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        results = self.model(frame, verbose=False)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]

            self.get_logger().info(f'Detected {cls_name} ({conf:.2f})')

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{cls_name} {conf:.2f}', (x1, max(0, y1 - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLOTestDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
