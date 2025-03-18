#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from message_filters import Subscriber, ApproximateTimeSynchronizer

from cmr_msgs.msg import GroundPlaneStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge

import numpy as np
import math
from shapely.geometry import Point, Polygon


class CostmapNode(Node):
    def __init__(self):
        super().__init__('costmap_node')

        self.declare_parameter('real', True) # FALSE IF RUNNING IN SIMULATION
        self.real = self.get_parameter('real').get_parameter_value().bool_value
        #self.real = True

        if self.real:
            self.ground_plane_sub = self.create_subscription(
                GroundPlaneStamped,
                '/camera/ground_plane',
                self.ground_plane_callback,
                10
            )
        
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            TwistStamped,
            '/autonomy/pose/robot/global',
            self.update_pose,
            10
        )
        
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
        self.max_cost = 100

        # Ground detection thresholds
        self.obstacle_threshold = 0.2
        self.ground_threshold = -0.3

        # Mounting height of camera
        self.camera_height = 1.0
        if self.real:
            self.camera_height = 1.2

        self.expected_height = -1.0 * self.camera_height
        self.clearance_height = 2.0
        self.max_depth = 10.0
        self.min_depth = 0.3
        self.cell_size = 0.25
        self.k = 4

        # Displacement from original position & rotation of robot, updated with data from localization node
        self.north = 0.0 # meters
        self.west = 0.0 # meters
        self.yaw = 0.0 # radians
        # Store last pose timestamp to compute current velocity [v_n, v_w, omega]
        self.last_pose_timestamp = None
        self.velocity = None
        # Rotation matrix, continually updated, to convert point cloud points from
        # rotated frame back to global
        self.R = np.array([
            [np.cos(self.yaw), -np.sin(self.yaw)],
            [np.sin(self.yaw),  np.cos(self.yaw)]
        ])

        # Timer to decay cell costs
        self.decay_timer = self.create_timer(10.0, self.decay_cost)

        # Timer to publish costmap
        self.pub_timer = self.create_timer(0.2, self.publish_obstacles)

    def pointcloud_callback(self, msg):
        """
        Process incoming PointCloud2 messages
        """
        if self.last_movement == "point_turn":
            return
        self.grid_init = True
        curr_obstacles = set()
        curr_free_space = set()
        north, west, R = self.interpolate_pose(msg.header.stamp)
        for pt in point_cloud2.read_points(msg, skip_nans=True):
            self.point_cloud_point_to_grid(pt, [north, west], R, curr_obstacles, curr_free_space)

        #self.decay_cost(curr_obstacles, curr_free_space)

    def point_cloud_point_to_grid(self, pt, pose, R, curr_obstacles, curr_free_space):
        '''
        Convert point cloud point to grid value, utilizing current yaw
        Takes into account coordinate system of camera, where x is right, y is 
        down, and z is forward. In the 2D grid, x is forward (north) and y is to 
        the left (west), same as global coordinates in gazebo. Returns True if 
        new obstacle detected, False if not
        '''
        if self.real:
            # ZED coordinate system is same as Gazebo, and coord system we are using
            # X forward, Y left, Z up
            height = self.camera_height + pt[2]
            x, y = pt[0], pt[1]
            dist = math.sqrt((x**2) + (y**2))
            if dist > self.max_depth or dist < self.min_depth:
                return
        else:
            # Y points downwards in camera coordinate frame in Gazebo
            height = self.camera_height - pt[1]
            x, y = pt[2], pt[0]
            #self.get_logger().info(f"x:  {x}  y:  {y}  height: {height}")
            if x > self.max_depth or x < self.min_depth:
                return
        
        rotated_pt = R.dot(np.array([x, y]))
        if self.real:
            x_rot = rotated_pt[0] + pose[0]
            y_rot = rotated_pt[1] + pose[1]
        else:
            x_rot = rotated_pt[0] + pose[0]
            y_rot = (-1.0*rotated_pt[1]) + pose[1]
        
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
        if (self.ground_threshold < height < self.obstacle_threshold) or height > self.clearance_height:
            curr_free_space.add((x_new, y_new))
            return
        # increment cell cost if height seems to represent obstacle
        else:
            self.grid_dict[(x_new, y_new)] = min(self.max_cost, self.grid_dict[(x_new, y_new)]+1)
            #self.grid_dict[(x_new, y_new)] = max(height, self.grid_dict[(x_new, y_new)])
            if (x_new, y_new) not in curr_obstacles:
                curr_obstacles.add((x_new, y_new))
        return

    def ground_plane_callback(self, msg):
        """
        Update costmap from vertices surrounding ground plane from ZED camera.
        """
        if self.last_movement == "point_turn":
            return
        pts = []
        north, west, R = self.interpolate_pose(msg.header.stamp)
        x = msg.x
        y = msg.y
        for i in range(len(x)):
            p = R.dot(np.array([x[i], y[i]]))
            pts.append([p[0]+north,p[1]+west])
        ground_polygon = Polygon(pts)
        
        # Iterate over grid cells and reduce cost if inside the ground polygon
        for (x, y) in list(self.grid_dict.keys()):
            if ground_polygon.contains(Point(x, y)):
                self.grid_dict[(x, y)] = min(self.grid_dict[(x, y)] - 3, 0)

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

    def decay_cost(self):
        """
        Linearly decay cost of grid cells if an 
        obstacle not actively detected in that region
        """
        for (x, y) in self.grid_dict.keys():
            self.grid_dict[(x, y)] = max(0, self.grid_dict[(x, y)]-1)
    
    '''def decay_cost(self, curr_obstacles, curr_free_space):
        """
        Linearly decay cost of grid cells if an 
        obstacle not actively detected in that region
        """
        for (x, y) in self.grid_dict.keys():
            #if (x, y) in curr_free_space and (x, y) not in curr_obstacles:
            if (x, y) not in curr_obstacles:
                self.grid_dict[(x, y)] = max(0, self.grid_dict[(x, y)]-1)'''

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
            [np.cos(self.yaw), -np.sin(self.yaw)],
            [np.sin(self.yaw),  np.cos(self.yaw)]
        ])
        
        return [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]

    def interpolate_pose(self, ts):
        """
        Returns a linear interpolation of the most accurate north, west
        and rotation matrix R based on current position, velocity, and 
        time delta between time measurement was received
        """
        if self.real:
            return self.north, self.west, self.R
        R = np.array([
            [np.cos(-1.0*self.yaw), -np.sin(-1.0*self.yaw)],
            [np.sin(-1.0*self.yaw),  np.cos(-1.0*self.yaw)]
        ])
        return self.north, self.west, R

    def timestamp_difference(self, timestamp1, timestamp2):
        """
        Calculate the time difference in seconds between two ROS 2 timestamps.

        Args:
            timestamp1 (builtin_interfaces.msg.Time): The first timestamp.
            timestamp2 (builtin_interfaces.msg.Time): The second timestamp.

        Returns:
            float: The time difference in seconds from timestamp1 to timestamp2.
        """
        time1 = Time.from_msg(timestamp1)
        time2 = Time.from_msg(timestamp2)
        return (time2 - time1).nanoseconds / 1e9


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