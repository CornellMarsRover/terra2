#!/usr/bin/env python3
"""ROS 2 human-gesture detector using an Ultralytics pose model."""

from __future__ import annotations

import math

import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from ultralytics import YOLO

from cmr_zed.gesture_core import GestureResult, GestureStabilizer, classify_pose


class HumanGestureDetectionNode(Node):
    def __init__(self):
        super().__init__("human_gesture_detection")

        self.declare_parameter("model_path", "yolo26n-pose.pt")
        self.declare_parameter("image_topic", "/zed/image")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("keypoint_threshold", 0.35)
        self.declare_parameter("confirmation_frames", 5)
        self.declare_parameter("clear_frames", 5)
        self.declare_parameter("publish_debug_image", True)

        model_path = str(self.get_parameter("model_path").value)
        image_topic = str(self.get_parameter("image_topic").value)
        self.confidence_threshold = float(
            self.get_parameter("confidence_threshold").value
        )
        self.keypoint_threshold = float(
            self.get_parameter("keypoint_threshold").value
        )
        confirmation_frames = int(
            self.get_parameter("confirmation_frames").value
        )
        clear_frames = int(self.get_parameter("clear_frames").value)
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.stabilizer = GestureStabilizer(
            confirmation_frames=confirmation_frames,
            clear_frames=clear_frames,
        )
        self.last_logged_label = None

        self.gesture_publisher = self.create_publisher(
            String,
            "/autonomy/human_gesture",
            10,
        )
        self.confidence_publisher = self.create_publisher(
            Float32,
            "/autonomy/human_gesture/confidence",
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            "/autonomy/human_gesture/debug_image",
            2,
        )
        self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Human gesture detection ready: model={model_path!r}, "
            f"image_topic={image_topic!r}"
        )

    def image_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"CvBridge input error: {exc}")
            return

        try:
            results = self.model.predict(
                source=frame,
                conf=self.confidence_threshold,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().error(
                f"Gesture inference failed: {exc!r}",
                throttle_duration_sec=2.0,
            )
            return

        observation = self._best_gesture(results)
        stable = self.stabilizer.update(observation)
        self.gesture_publisher.publish(String(data=stable.label))
        self.confidence_publisher.publish(Float32(data=float(stable.confidence)))

        if stable.label != self.last_logged_label:
            self.get_logger().info(
                f"Confirmed gesture: {stable.label} "
                f"(confidence={stable.confidence:.2f})"
            )
            self.last_logged_label = stable.label

        if self.publish_debug_image and results:
            self._publish_debug_image(results[0].plot(), message)

    def _best_gesture(self, results) -> GestureResult:
        best = GestureResult("none", 0.0)
        for result in results:
            if result.keypoints is None or result.keypoints.data is None:
                continue
            poses = result.keypoints.data.detach().cpu().numpy()
            for pose in poses:
                candidate = classify_pose(
                    pose,
                    min_keypoint_confidence=self.keypoint_threshold,
                )
                if (
                    candidate.label != "none"
                    and math.isfinite(candidate.confidence)
                    and candidate.confidence > best.confidence
                ):
                    best = candidate
        return best

    def _publish_debug_image(self, frame, source_message: Image) -> None:
        try:
            output = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            output.header = source_message.header
            self.debug_publisher.publish(output)
        except CvBridgeError as exc:
            self.get_logger().error(f"CvBridge output error: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = HumanGestureDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
