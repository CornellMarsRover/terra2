#!/usr/bin/env python3

import math
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from autonomous_navigation.sim_vision_obstacle_core import (
    expand_box_cluster,
    global_to_local,
    visible_in_fov,
)

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

class SimVisionObstacleDetector(Node):
    """Simulation obstacle detector using image CV when available and synthetic vision otherwise."""

    def __init__(self) -> None:
        super().__init__("sim_vision_obstacle_detector")

        self.declare_parameter("mode", "auto")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("obstacle_height_m", 1.0)
        self.declare_parameter("focal_length_px", 554.0)
        self.declare_parameter("min_contour_area_px", 600.0)
        self.declare_parameter("min_distance_m", 0.6)
        self.declare_parameter("max_distance_m", 6.0)
        self.declare_parameter("cost_value", 25.0)
        self.declare_parameter("cell_size", 0.25)
        self.declare_parameter("obstacle_x", 2.5)
        self.declare_parameter("obstacle_y", 0.0)
        self.declare_parameter("obstacle_size_x", 0.8)
        self.declare_parameter("obstacle_size_y", 0.8)
        self.declare_parameter("fov_deg", 72.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("image_fallback_timeout_s", 1.0)

        self.bridge = CvBridge() if CvBridge is not None else None
        self.robot_north = 0.0
        self.robot_west = 0.0
        self.robot_yaw = 0.0
        self.last_image_time = None
        self.reported_synthetic_fallback = False

        self.mode = str(self.get_parameter("mode").value).lower()
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.obstacle_height_m = float(self.get_parameter("obstacle_height_m").value)
        self.focal_length_px = float(self.get_parameter("focal_length_px").value)
        self.min_contour_area_px = float(self.get_parameter("min_contour_area_px").value)
        self.min_distance_m = float(self.get_parameter("min_distance_m").value)
        self.max_distance_m = float(self.get_parameter("max_distance_m").value)
        self.cost_value = float(self.get_parameter("cost_value").value)
        self.cell_size = float(self.get_parameter("cell_size").value)
        self.obstacle_north = float(self.get_parameter("obstacle_x").value)
        self.obstacle_west = float(self.get_parameter("obstacle_y").value)
        self.obstacle_size_north = float(self.get_parameter("obstacle_size_x").value)
        self.obstacle_size_west = float(self.get_parameter("obstacle_size_y").value)
        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.image_fallback_timeout_s = float(self.get_parameter("image_fallback_timeout_s").value)

        self.costmap_pub = self.create_publisher(Float32MultiArray, "/autonomy/costmap", 10)
        self.create_subscription(TwistStamped, "/autonomy/pose/robot/global", self.pose_callback, 10)

        if self.mode in {"auto", "image"}:
            self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.timer_callback)

        self.get_logger().info(
            f"Sim vision obstacle detector mode={self.mode} image_topic={self.image_topic} "
            f"obstacle=({self.obstacle_north:.2f}, {self.obstacle_west:.2f})"
        )

    def pose_callback(self, msg: TwistStamped) -> None:
        self.robot_north = msg.twist.linear.x
        self.robot_west = msg.twist.linear.y
        self.robot_yaw = msg.twist.angular.z

    def timer_callback(self) -> None:
        if self.should_use_synthetic_mode():
            if not self.reported_synthetic_fallback:
                self.reported_synthetic_fallback = True
                self.get_logger().warn(
                    "No ROS camera frames available; using synthetic obstacle visibility for sim.",
                    throttle_duration_sec=5.0,
                )
            self.publish_synthetic_detection()

    def should_use_synthetic_mode(self) -> bool:
        if cv2 is None or self.bridge is None:
            return True
        if self.mode == "synthetic":
            return True
        if self.mode == "image":
            return False
        if self.count_publishers(self.image_topic) == 0:
            return True
        if self.last_image_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
        return elapsed > self.image_fallback_timeout_s

    def image_callback(self, msg: Image) -> None:
        if cv2 is None or self.bridge is None:
            return
        self.last_image_time = self.get_clock().now()
        if self.mode == "synthetic":
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([35, 80, 50], dtype=np.uint8)
        upper = np.array([95, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.publish_costmap([])
            return

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < self.min_contour_area_px:
            self.publish_costmap([])
            return

        x, _, w, h = cv2.boundingRect(contour)
        if h <= 0:
            self.publish_costmap([])
            return

        distance = (self.obstacle_height_m * self.focal_length_px) / float(h)
        distance = max(self.min_distance_m, min(self.max_distance_m, distance))

        image_center_x = image.shape[1] / 2.0
        centroid_x = x + (w / 2.0)
        normalized_x = (centroid_x - image_center_x) / self.focal_length_px
        local_forward = distance
        local_left = -normalized_x * distance

        global_north = self.robot_north + (
            math.cos(self.robot_yaw) * local_forward - math.sin(self.robot_yaw) * local_left
        )
        global_west = self.robot_west + (
            math.sin(self.robot_yaw) * local_forward + math.cos(self.robot_yaw) * local_left
        )

        cluster = expand_box_cluster(
            global_north,
            global_west,
            self.obstacle_size_north,
            self.obstacle_size_west,
            self.cell_size,
            self.cost_value,
        )
        self.publish_costmap(cluster)
        self.get_logger().info(
            f"image_obstacle distance={distance:.2f}m lateral={local_left:+.2f}m "
            f"global=({global_north:.2f}, {global_west:.2f}) pixels={int(area)}",
            throttle_duration_sec=1.0,
        )

    def publish_synthetic_detection(self) -> None:
        local_forward, local_left = global_to_local(
            self.robot_north,
            self.robot_west,
            self.robot_yaw,
            self.obstacle_north,
            self.obstacle_west,
        )
        if not visible_in_fov(
            local_forward,
            local_left,
            self.fov_deg,
            self.min_distance_m,
            self.max_distance_m,
        ):
            self.publish_costmap([])
            return

        cluster = expand_box_cluster(
            self.obstacle_north,
            self.obstacle_west,
            self.obstacle_size_north,
            self.obstacle_size_west,
            self.cell_size,
            self.cost_value,
        )
        self.publish_costmap(cluster)
        self.get_logger().info(
            f"synthetic_obstacle visible forward={local_forward:.2f}m left={local_left:+.2f}m "
            f"global=({self.obstacle_north:.2f}, {self.obstacle_west:.2f})",
            throttle_duration_sec=1.0,
        )

    def publish_costmap(self, data: List[float]) -> None:
        msg = Float32MultiArray()
        msg.data = data
        self.costmap_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimVisionObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
