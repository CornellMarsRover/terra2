#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge

import numpy as np
import math


class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')

        self.declare_parameter('real', False) # FALSE IF RUNNING IN SIMULATION
        self.real = self.get_parameter('real').get_parameter_value().bool_value 

        # Create synchronized subscribers
        self.pointcloud_sub = Subscriber(self, PointCloud2, '/camera/points')
        self.pose_sub = Subscriber(self, TwistStamped, '/autonomy/pose/robot/global')

        # Synchronizer with a time tolerance (slop) of 0.1 seconds
        self.sync = ApproximateTimeSynchronizer(
            [self.pointcloud_sub, self.pose_sub],
            queue_size=30,
            slop=0.1
        )
        self.sync.registerCallback(self.synchronized_callback)

        # Subscriber to know last movement
        self.movement_sub = self.create_subscription(
            String,
            '/autonomy/move/move_type',
            self.update_last_movement,
            10
        )

        self.last_movement = "ackerman"

        # Publisher for publishing costmap values
        self.new_obstacle_publisher = self.create_publisher(Float32MultiArray, '/autonomy/costmap', 10)

        # Grid with discretized coordinates
        self.grid_dict = dict()

        # Store all detected obstacles
        self.obstacles_global = set()

        # Maximum cost for occupied cells in the costmap
        self.max_cost = 10

        # Ground detection thresholds
        self.height_threshold = 0.2
        self.gradient_threshold = 0.2
        self.expected_height = -1.0
        self.clearance_height = 2.0
        self.max_depth = 10.0
        self.min_depth = 0.3
        self.cell_size = 0.25
        self.k = 4

        # Displacement from original position & rotation of robot, updated with data from localization node
        self.north = 0.0 # meters
        self.west = 0.0 # meters
        self.yaw = 0.0 # radians

        # Rotation matrix, continually updated, to convert point cloud points from
        # rotated frame back to global
        self.R = np.array([
            [np.cos(-1.0 * self.yaw), -np.sin(-1.0 * self.yaw)],
            [np.sin(-1.0 * self.yaw),  np.cos(-1.0 * self.yaw)]
        ])


    def synchronized_callback(self, pointcloud_msg, pose_msg):
        """
        Callback to handle synchronized PointCloud2 and TwistStamped messages.
        """
        if self.last_movement == "point_turn":
            return
        self.get_logger().info("Synchronized callback")
        pose = self.update_pose(pose_msg)
        self.pointcloud_callback(pointcloud_msg, pose)

        self.publish_obstacles()

    def pointcloud_callback(self, msg, pose):
        """
        Process incoming PointCloud2 messages
        """
        self.grid_init = True
        curr_obstacles = set()
        curr_free_space = set()

        R = np.array([
            [np.cos(-1.0 * pose[2]), -np.sin(-1.0 * pose[2])],
            [np.sin(-1.0 * pose[2]),  np.cos(-1.0 * pose[2])]
        ])
        for pt in point_cloud2.read_points(msg, skip_nans=True):
            self.point_cloud_point_to_grid(pt, pose, R, curr_obstacles, curr_free_space)

        self.decay_cost(curr_obstacles, curr_free_space)

    def point_cloud_point_to_grid(self, pt, pose, R, curr_obstacles, curr_free_space):
        '''
        Convert point cloud point to grid value, utilizing current yaw
        Takes into account coordinate system of camera, where x is right, y is 
        down, and z is forward. In the 2D grid, x is forward (north) and y is to 
        the left (west), same as global coordinates in gazebo. Returns True if 
        new obstacle detected, False if not
        '''
        height = -1.0 * pt[1]
        x, y = pt[2], pt[0]
        if x > self.max_depth - 0.5 or x < self.min_depth:
            return
        rotated_pt = R.dot(np.array([x, y]))
        x_rot = rotated_pt[0] + pose[0]
        y_rot = (-1.0 * rotated_pt[1]) + pose[1]
        if x_rot is None or y_rot is None:
            return
        # discretize to 0.25 m
        x_new = round(x_rot * self.k) / self.k
        y_new = round(y_rot * self.k) / self.k
        # don't update if out of grid bounds or previously detected an obstacle at that grid location
        if (x_new, y_new) in curr_obstacles:
            return
        if (x_new, y_new) not in self.grid_dict:
            self.grid_dict[(x_new, y_new)] = 0
        # Store traversable cells to decay
        if (self.expected_height - self.height_threshold < height < self.expected_height + self.height_threshold) or height > self.clearance_height:
            curr_free_space.add((x_new, y_new))
            return
        # increment cell cost if height seems to represent obstacle
        else:
            self.grid_dict[(x_new, y_new)] = min(self.max_cost, self.grid_dict[(x_new, y_new)]+1)
            if (x_new, y_new) not in curr_obstacles:
                curr_obstacles.add((x_new, y_new))
        return
    
    def publish_obstacles(self):
        """
        Publish newly detected obstacles from point cloud
        """
        msg = Float32MultiArray()
        data = []
        for ((x,y), cost) in self.grid_dict.items():
            if cost > 0:
                data.extend([x, y, float(cost)])
        msg.data = data
        self.new_obstacle_publisher.publish(msg)

    def decay_cost(self, curr_obstacles, curr_free_space):
        """
        Linearly decay cost of grid cells if an 
        obstacle not actively detected in that region
        """
        for (x, y) in self.grid_dict.keys():
            if (x, y) not in curr_free_space or (x, y) in curr_obstacles:
                continue
            self.grid_dict[(x, y)] = max(0, self.grid_dict[(x, y)]-1)
    
    def update_last_movement(self, msg):
        """
        Update last movement to avoid point cloud data while turning
        """
        self.last_movement = msg.data

    def update_pose(self, msg):
        """
        Update the pose of the robot using message from localization node
        """
        self.north = msg.twist.linear.x
        self.west = msg.twist.linear.y
        self.yaw = msg.twist.angular.z
        self.R = np.array([
            [np.cos(-1.0 * self.yaw), -np.sin(-1.0 * self.yaw)],
            [np.sin(-1.0 * self.yaw),  np.cos(-1.0 * self.yaw)]
        ])
        
        return [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]

def main(args=None):
    rclpy.init(args=args)
    node = CostmapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()