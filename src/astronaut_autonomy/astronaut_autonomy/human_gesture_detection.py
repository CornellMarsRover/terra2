#!/usr/bin/env python3
"""ROS 2 astronaut command and ZED range perception node."""

from __future__ import annotations

import math

import cv2
import message_filters
import numpy as np
import rclpy
from cmr_msgs.msg import AstronautCommand
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32, String
from ultralytics import YOLO

from astronaut_autonomy.depth_core import (
    TargetTracker,
    command_requires_distance,
    estimate_target_from_xyz,
    organized_xyz_from_buffer,
    safe_command_for,
)
from astronaut_autonomy.gesture_core import (
    LEFT_HIP,
    LEFT_SHOULDER,
    RIGHT_HIP,
    RIGHT_SHOULDER,
    GestureResult,
    GestureStabilizer,
    classify_pose,
)


class HumanGestureDetectionNode(Node):
    def __init__(self):
        super().__init__("human_gesture_detection")

        self.declare_parameter("model_path", "yolo26n-pose.pt")
        self.declare_parameter("image_topic", "/zed/image")
        self.declare_parameter("pointcloud_topic", "/camera/points")
        self.declare_parameter("confidence_threshold", 0.50)
        self.declare_parameter("keypoint_threshold", 0.35)
        self.declare_parameter("confirmation_frames", 5)
        self.declare_parameter("clear_frames", 5)
        self.declare_parameter("sync_queue_size", 5)
        self.declare_parameter("sync_slop_seconds", 0.12)
        self.declare_parameter("minimum_distance_m", 0.30)
        self.declare_parameter("maximum_distance_m", 15.0)
        self.declare_parameter("depth_smoothing_alpha", 0.35)
        self.declare_parameter("depth_stale_after_seconds", 0.60)
        self.declare_parameter("publish_debug_image", True)

        model_path = str(self.get_parameter("model_path").value)
        image_topic = str(self.get_parameter("image_topic").value)
        pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
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
        sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        sync_slop_seconds = float(
            self.get_parameter("sync_slop_seconds").value
        )
        self.minimum_distance_m = float(
            self.get_parameter("minimum_distance_m").value
        )
        self.maximum_distance_m = float(
            self.get_parameter("maximum_distance_m").value
        )
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.stabilizer = GestureStabilizer(
            confirmation_frames=confirmation_frames,
            clear_frames=clear_frames,
        )
        self.target_tracker = TargetTracker(
            smoothing_alpha=float(
                self.get_parameter("depth_smoothing_alpha").value
            ),
            stale_after_s=float(
                self.get_parameter("depth_stale_after_seconds").value
            ),
        )
        self.last_logged_state = None

        self.gesture_publisher = self.create_publisher(
            String, "/autonomy/human_gesture", 10
        )
        self.confidence_publisher = self.create_publisher(
            Float32, "/autonomy/human_gesture/confidence", 10
        )
        self.command_publisher = self.create_publisher(
            AstronautCommand, "/autonomy/astronaut/perception", 10
        )
        self.distance_publisher = self.create_publisher(
            Float32, "/autonomy/astronaut/distance", 10
        )
        self.target_publisher = self.create_publisher(
            PointStamped, "/autonomy/astronaut/target", 10
        )
        self.debug_publisher = self.create_publisher(
            Image, "/autonomy/human_gesture/debug_image", 2
        )

        self.image_subscriber = message_filters.Subscriber(
            self,
            Image,
            image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.pointcloud_subscriber = message_filters.Subscriber(
            self,
            PointCloud2,
            pointcloud_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.image_subscriber, self.pointcloud_subscriber],
            queue_size=sync_queue_size,
            slop=sync_slop_seconds,
        )
        self.synchronizer.registerCallback(self.sensor_callback)

        self.get_logger().info(
            "Astronaut perception ready: "
            f"model={model_path!r}, image={image_topic!r}, "
            f"pointcloud={pointcloud_topic!r}"
        )

    def sensor_callback(
        self,
        image_message: Image,
        pointcloud_message: PointCloud2,
    ) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8"
            )
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

        observation, target_pose = self._best_pose_and_gesture(results)
        stable = self.stabilizer.update(observation)
        astronaut_detected = target_pose is not None

        estimate = None
        if target_pose is not None:
            try:
                xyz = self._pointcloud_xyz(pointcloud_message)
                estimate = estimate_target_from_xyz(
                    target_pose,
                    xyz,
                    image_size=(frame.shape[1], frame.shape[0]),
                    minimum_keypoint_confidence=self.keypoint_threshold,
                    minimum_distance_m=self.minimum_distance_m,
                    maximum_distance_m=self.maximum_distance_m,
                )
            except (TypeError, ValueError) as exc:
                self.get_logger().error(
                    f"Cannot decode ZED point cloud: {exc}",
                    throttle_duration_sec=2.0,
                )

        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        tracked = self.target_tracker.update(
            astronaut_detected=astronaut_detected,
            estimate=estimate,
            now=now,
        )
        distance_valid = tracked.depth_status == "VALID"
        # A held stable label can outlive one bad pose frame. Never pair a
        # lingering FOLLOW/FETCH label with another visible person's depth.
        depth_matches_gesture = observation.label == stable.label
        command = safe_command_for(
            stable.label,
            distance_valid and depth_matches_gesture,
        )

        self._publish_outputs(
            image_message,
            pointcloud_message,
            stable,
            command,
            tracked,
        )

        state = (stable.label, command, tracked.depth_status)
        if state != self.last_logged_state:
            distance_text = (
                f"{tracked.estimate.distance_m:.2f} m"
                if tracked.estimate is not None
                else "unavailable"
            )
            self.get_logger().info(
                f"Gesture={stable.label}, command={command}, "
                f"confidence={stable.confidence:.2f}, "
                f"distance={distance_text} ({tracked.depth_status})"
            )
            self.last_logged_state = state

        if self.publish_debug_image and results:
            debug_frame = results[0].plot()
            self._annotate_debug(
                debug_frame,
                stable,
                command,
                tracked.depth_status,
                tracked.estimate,
            )
            self._publish_debug_image(debug_frame, image_message)

    def _best_pose_and_gesture(self, results):
        best_gesture = GestureResult("NONE", 0.0)
        best_gesture_pose = None
        best_visible_pose = None
        best_visibility = 0.0

        for result in results:
            if result.keypoints is None or result.keypoints.data is None:
                continue
            poses = result.keypoints.data.detach().cpu().numpy()
            for pose in poses:
                visibility = self._torso_visibility(pose)
                if visibility > best_visibility:
                    best_visibility = visibility
                    best_visible_pose = pose
                candidate = classify_pose(
                    pose,
                    min_keypoint_confidence=self.keypoint_threshold,
                )
                if (
                    candidate.label != "NONE"
                    and math.isfinite(candidate.confidence)
                    and candidate.confidence > best_gesture.confidence
                ):
                    best_gesture = candidate
                    best_gesture_pose = pose

        if best_gesture_pose is not None:
            return best_gesture, best_gesture_pose
        if best_visibility >= self.keypoint_threshold:
            return best_gesture, best_visible_pose
        return best_gesture, None

    @staticmethod
    def _torso_visibility(pose) -> float:
        indices = (LEFT_SHOULDER, RIGHT_SHOULDER, LEFT_HIP, RIGHT_HIP)
        if pose is None or len(pose) <= max(indices):
            return 0.0
        return min(float(pose[index][2]) for index in indices)

    @staticmethod
    def _pointcloud_xyz(message: PointCloud2) -> np.ndarray:
        fields = {field.name: field for field in message.fields}
        missing = {"x", "y", "z"} - set(fields)
        if missing:
            raise ValueError(
                "missing PointCloud2 fields: " + ", ".join(sorted(missing))
            )
        for name in ("x", "y", "z"):
            field = fields[name]
            if field.datatype != PointField.FLOAT32 or field.count != 1:
                raise TypeError(f"{name} must be one FLOAT32 field")
        if message.height <= 1:
            raise ValueError("distance requires an organized point cloud")
        return organized_xyz_from_buffer(
            message.data,
            height=message.height,
            width=message.width,
            point_step=message.point_step,
            row_step=message.row_step,
            x_offset=fields["x"].offset,
            y_offset=fields["y"].offset,
            z_offset=fields["z"].offset,
            is_bigendian=message.is_bigendian,
        )

    def _publish_outputs(
        self,
        image_message,
        pointcloud_message,
        stable,
        command,
        tracked,
    ) -> None:
        self.gesture_publisher.publish(String(data=stable.label))
        self.confidence_publisher.publish(
            Float32(data=float(stable.confidence))
        )

        estimate = tracked.estimate
        perception = AstronautCommand()
        perception.header = image_message.header
        perception.gesture = stable.label
        perception.command = command
        perception.confidence = float(stable.confidence)
        perception.astronaut_detected = bool(tracked.astronaut_detected)
        perception.distance_required = command_requires_distance(stable.label)
        perception.distance_valid = tracked.depth_status == "VALID"
        perception.depth_status = tracked.depth_status
        perception.distance_m = (
            float(estimate.distance_m) if estimate is not None else math.nan
        )
        perception.direction = estimate.direction if estimate else "UNKNOWN"
        perception.horizontal_angle_deg = (
            float(estimate.horizontal_angle_deg) if estimate else math.nan
        )
        perception.depth_samples = estimate.sample_count if estimate else 0
        if estimate is not None:
            perception.position.x = float(estimate.position_m[0])
            perception.position.y = float(estimate.position_m[1])
            perception.position.z = float(estimate.position_m[2])
        self.command_publisher.publish(perception)
        self.distance_publisher.publish(
            Float32(data=perception.distance_m)
        )

        if estimate is not None and tracked.depth_status == "VALID":
            target = PointStamped()
            target.header = pointcloud_message.header
            target.point = perception.position
            self.target_publisher.publish(target)

    @staticmethod
    def _annotate_debug(frame, stable, command, depth_status, estimate) -> None:
        color = (0, 220, 0) if command != "NONE" else (0, 80, 255)
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 86), (0, 0, 0), -1)
        cv2.putText(
            frame,
            f"Gesture: {stable.label}  Command: {command}",
            (18, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.82,
            color,
            2,
            cv2.LINE_AA,
        )
        if estimate is None:
            depth_text = f"ZED distance: unavailable ({depth_status})"
        else:
            depth_text = (
                f"ZED distance: {estimate.distance_m:.2f} m  "
                f"bearing: {estimate.direction} "
                f"{estimate.horizontal_angle_deg:+.1f} deg ({depth_status})"
            )
        cv2.putText(
            frame,
            depth_text,
            (18, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (230, 230, 230),
            2,
            cv2.LINE_AA,
        )

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
