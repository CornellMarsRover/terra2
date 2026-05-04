#!/usr/bin/env python3

import math
from typing import List, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header

from autonomous_navigation.sim_vision_obstacle_core import (
    filter_visible_obstacles,
    global_to_local,
    sample_box_pointcloud,
    visible_in_fov,
)


class SimObstaclePointCloud(Node):
    """Simulation adapter that publishes /camera/points for the real autonomy costmap."""

    def __init__(self) -> None:
        super().__init__("sim_obstacle_pointcloud")

        self.declare_parameter("point_topic", "/camera/points")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("min_distance_m", 0.3)
        self.declare_parameter("max_distance_m", 8.0)
        self.declare_parameter("fov_deg", 40.0)
        self.declare_parameter("synthetic_occlusion_overlap_deg", 10.0)
        self.declare_parameter("lateral_step_m", 0.2)
        self.declare_parameter("depth_step_m", 0.2)
        self.declare_parameter("vertical_levels_m", [0.25, 0.65, 1.05])
        self.declare_parameter("obstacle_norths", [2.5])
        self.declare_parameter("obstacle_wests", [0.0])
        self.declare_parameter("obstacle_size_norths", [0.8])
        self.declare_parameter("obstacle_size_wests", [0.8])
        self.declare_parameter("obstacle_north", float("nan"))
        self.declare_parameter("obstacle_west", float("nan"))
        self.declare_parameter("obstacle_size_north", float("nan"))
        self.declare_parameter("obstacle_size_west", float("nan"))

        self.point_topic = str(self.get_parameter("point_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.min_distance_m = float(self.get_parameter("min_distance_m").value)
        self.max_distance_m = float(self.get_parameter("max_distance_m").value)
        self.fov_deg = float(self.get_parameter("fov_deg").value)
        self.synthetic_occlusion_overlap_deg = float(
            self.get_parameter("synthetic_occlusion_overlap_deg").value
        )
        self.lateral_step_m = float(self.get_parameter("lateral_step_m").value)
        self.depth_step_m = float(self.get_parameter("depth_step_m").value)
        self.vertical_levels = [float(v) for v in self.get_parameter("vertical_levels_m").value]
        self.obstacles = self._load_obstacles()

        self.robot_north = 0.0
        self.robot_west = 0.0
        self.robot_yaw = 0.0

        self.pointcloud_pub = self.create_publisher(PointCloud2, self.point_topic, 10)
        self.all_obstacles_pub = self.create_publisher(
            Float32MultiArray, "/autonomy/sim_obstacles/all", 10
        )
        self.visible_obstacles_pub = self.create_publisher(
            Float32MultiArray, "/autonomy/sim_obstacles/visible", 10
        )
        self.create_subscription(TwistStamped, "/autonomy/pose/robot/global", self.pose_callback, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_points)

        self.get_logger().info(
            f"Publishing simulated obstacle point clouds on {self.point_topic} "
            f"for {len(self.obstacles)} obstacle(s)"
        )

    def _load_obstacles(self) -> List[Tuple[float, float, float, float]]:
        norths = [float(value) for value in self.get_parameter("obstacle_norths").value]
        wests = [float(value) for value in self.get_parameter("obstacle_wests").value]
        size_norths = [float(value) for value in self.get_parameter("obstacle_size_norths").value]
        size_wests = [float(value) for value in self.get_parameter("obstacle_size_wests").value]
        north_scalar = float(self.get_parameter("obstacle_north").value)
        west_scalar = float(self.get_parameter("obstacle_west").value)
        size_north_scalar = float(self.get_parameter("obstacle_size_north").value)
        size_west_scalar = float(self.get_parameter("obstacle_size_west").value)
        if not math.isnan(north_scalar) and not math.isnan(west_scalar):
            norths = [north_scalar]
            wests = [west_scalar]
            size_norths = [0.8 if math.isnan(size_north_scalar) else size_north_scalar]
            size_wests = [0.8 if math.isnan(size_west_scalar) else size_west_scalar]
        count = min(len(norths), len(wests), len(size_norths), len(size_wests))
        return [
            (norths[idx], wests[idx], size_norths[idx], size_wests[idx])
            for idx in range(count)
        ]

    def pose_callback(self, msg: TwistStamped) -> None:
        self.robot_north = float(msg.twist.linear.x)
        self.robot_west = float(msg.twist.linear.y)
        self.robot_yaw = float(msg.twist.angular.z)

    def publish_points(self) -> None:
        self.publish_all_obstacles()

        candidates: List[Tuple[float, float, float, float, float, float]] = []
        for obstacle_north, obstacle_west, size_north, size_west in self.obstacles:
            local_forward, local_left = global_to_local(
                self.robot_north,
                self.robot_west,
                self.robot_yaw,
                obstacle_north,
                obstacle_west,
            )
            if not visible_in_fov(
                local_forward,
                local_left,
                self.fov_deg,
                self.min_distance_m,
                self.max_distance_m,
            ):
                continue
            candidates.append(
                (
                    obstacle_north,
                    obstacle_west,
                    size_north,
                    size_west,
                    local_forward,
                    local_left,
                )
            )

        visible = filter_visible_obstacles(
            candidates, occlusion_overlap_deg=self.synthetic_occlusion_overlap_deg
        )
        self.publish_visible_obstacles(visible)

        point_list: List[Tuple[float, float, float]] = []
        for _, _, size_north, size_west, local_forward, local_left in visible:
            point_list.extend(
                sample_box_pointcloud(
                    local_forward,
                    local_left,
                    size_north,
                    size_west,
                    lateral_step_m=self.lateral_step_m,
                    depth_step_m=self.depth_step_m,
                    vertical_levels=self.vertical_levels,
                )
            )

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, point_list)
        self.pointcloud_pub.publish(cloud)

        if visible:
            summary = ", ".join(
                f"({north:.2f},{west:.2f})"
                for north, west, _, _, _, _ in visible
            )
        else:
            summary = "none"
        self.get_logger().info(
            f"sim_points visible={summary} points={len(point_list)}",
            throttle_duration_sec=1.0,
        )

    def publish_all_obstacles(self) -> None:
        msg = Float32MultiArray()
        data: List[float] = []
        for north, west, size_north, size_west in self.obstacles:
            data.extend([north, west, size_north, size_west])
        msg.data = data
        self.all_obstacles_pub.publish(msg)

    def publish_visible_obstacles(
        self, visible: Sequence[Tuple[float, float, float, float, float, float]]
    ) -> None:
        msg = Float32MultiArray()
        data: List[float] = []
        for north, west, size_north, size_west, _, _ in visible:
            data.extend([north, west, size_north, size_west])
        msg.data = data
        self.visible_obstacles_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimObstaclePointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
