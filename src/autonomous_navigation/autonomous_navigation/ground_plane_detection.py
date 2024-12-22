#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import numpy as np
import random
import rerun as rr
import math


class GroundPlaneGridNode(Node):
    def __init__(self):
        super().__init__('ground_plane_grid_node')

        # Initialize Rerun visualization
        rr.init("ground_plane_grid", spawn=True)

        # Subscribe to the point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )

        # Parameters for the grid
        self.grid_size = 200  # Number of cells in one dimension
        self.grid_extent = 20.0  # Grid extent in meters
        self.grid = {} # Grid where True represents traversable and False represents not traversable
        x_index = 0.0
        for x in range(self.grid_size):
            y_index = -0.5 * self.grid_extent
            for y in range(self.grid_size):
                self.grid[(f'{round(x_index, 1):g}', f'{round(y_index, 1):g}')] = True
                y_index += self.grid_extent / self.grid_size
            x_index += self.grid_extent / self.grid_size

        # Ground detection thresholds
        self.height_threshold = 0.1
        self.gradient_threshold = 0.2
        self.expected_height = -1.0
        self.max_depth = 19.5

    def convert_xy_to_grid_key(self, x, y):
        new_x = f'{round(x, 1):g}'
        new_y = f'{round(y, 1):g}'
        #self.get_logger().info(f"original grid coordinate: {x},{y}")
        #self.get_logger().info(f"discretized grid coordinate: {new_x},{new_y}")
        if (new_x, new_y) in self.grid:
            return new_x, new_y
        return None, None

    def clear_grid(self):
        """Clear the point grid each new timestep"""
        for key in self.grid.keys():
            self.grid[key] = True


    def plot_grid_rerun(self, path=None):
        """Visualize the grid"""
        points = []
        colors = []
        for key, traversable in self.grid.items():

            points.append([float(key[0]), float(key[1]), 0])
            if traversable:
                colors.append([0, 255, 0])
            else:
                colors.append([255, 0, 0])

        rr.log("traversability_estimate", rr.Points3D(np.array(points, dtype=np.float32), colors=colors, radii=0.01))
        rr.log("current_position", rr.Points3D(np.array([[0,0,0]]), colors=[0,0,255], radii=0.05))

        if path is not None:
            """Visualize the planned path"""
            rr.log("path", rr.Points3D(
                np.array([[x, y, self.expected_height] for x, y in path], dtype=np.float32),
                colors=[0, 0, 255],
                radii=0.05
            ))

    def pointcloud_callback(self, msg):
        """Process incoming PointCloud2 messages and visualize ground detection."""
        self.clear_grid()

        for pt in point_cloud2.read_points(msg, skip_nans=True):
            x, y, z = pt[0], pt[1], pt[2]
            if z > self.max_depth:
                continue
            new_x, new_y = self.convert_xy_to_grid_key(z, x)
            if new_x is not None:
                if not (self.expected_height - self.height_threshold < (-1.0 * y) < self.expected_height + self.height_threshold):
                    self.grid[(new_x, new_y)] = False

        #self.get_logger().info(f"{self.grid}")
        self.plot_grid_rerun()


def main(args=None):
    rclpy.init(args=args)
    node = GroundPlaneGridNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
