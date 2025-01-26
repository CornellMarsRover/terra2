#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
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
            queue_size=100,
            slop=1.0
        )
        self.sync.registerCallback(self.synchronized_callback)
        # 2D costmap publisher that publishes grid encoded as an image
        self.costmap_publisher = self.create_publisher(Image, '/autonomy/costmap/image_grid', 10)
        # Current robot position in the costmap grid
        self.costmap_position_publisher = self.create_publisher(Float32MultiArray, '/autonomy/costmap/shift', 10)
        self.timer = self.create_timer(0.1, self.publish_costmap)  # Publish at 10Hz
        self.bridge = CvBridge()


        # Parameters for the grid
        self.cell_size = 0.5  # Side length of square in grid in meters
        self.grid_size = 50.0  # Grid side length in meters, axis range from -grid_size/2 to grid_size/2
        # Coordinates of grid match Gazebo when there is no yaw
        # x - forward, initially north
        # y - left, initially west
        # z - up, north
        self.grid_length = int(self.grid_size/self.cell_size)
        self.center_index = int(self.grid_length/2)
        self.grid = np.zeros((self.grid_length, self.grid_length), dtype=np.int8)
        # Default grid target is the current robot position
        self.grid_target = (self.center_index, self.center_index)
        self.grid_init = False
        self.robot_pos = (self.center_index, self.center_index)
        
        # Store obstacles most recently detected
        self.curr_obstacles = set()

        # Store free space most recently detected
        self.curr_free_space = set()

        # Store all detected obstacles
        self.obstacles_global = set()
        self.obstacle_cost_threshold = 5

        # Maximum cost for occupied cells in the costmap
        self.max_cost = 20

        # Ground detection thresholds
        self.height_threshold = 0.2
        self.gradient_threshold = 0.2
        self.expected_height = -1.0
        self.clearance_height = 2.0
        self.max_depth = 20.0

        # Displacement from original position & rotation of robot, updated with data from localization node
        self.north = 0.0 # meters
        self.west = 0.0 # meters
        self.yaw = 0.0 # radians

        # Displacement from original position at the centerpoint of occupancy grid
        self.grid_center = [0.0, 0.0]

        # How much to shift new points detected before parsing to costmap
        self.grid_shift = [0.0, 0.0]

        # Position of robot in grid for parsing point cloud data and pose visualization
        self.robot_grid_position = [self.center_index, self.center_index, 0]

        # Rotation matrix, continually updated, to convert point cloud points from
        # rotated frame back to global
        self.R = np.array([
            [np.cos(-1.0 * self.yaw), -np.sin(-1.0 * self.yaw)],
            [np.sin(-1.0 * self.yaw),  np.cos(-1.0 * self.yaw)]
        ])

        # Timer to check if grid needs to be shifted
        self.shift_timer = self.create_timer(0.2, self.shift_grid)

        self.new_obstacle_publisher = self.create_publisher(Float32MultiArray, '/autonomy/costmap/new_obstacles', 10)

    def synchronized_callback(self, pointcloud_msg, pose_msg):
        """
        Callback to handle synchronized PointCloud2 and TwistStamped messages.
        """
        self.get_logger().info("Synchronized callback")
        self.update_pose(pose_msg)
        self.pointcloud_callback(pointcloud_msg)


    def publish_costmap(self):
        '''
        Publish the 2D costmap grid encoded as an image
        '''
        # Scale the costmap values from 0-10 to 0-128
        costmap_scaled = (self.grid * 128 / self.max_cost).astype(np.uint8)

        # Convert numpy array to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(costmap_scaled, encoding='mono8')
        msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message
        self.costmap_publisher.publish(msg)
        #self.get_logger().info("Published costmap with robot position")


    def shift_grid(self):
        """
        Shifts the occupancy grid when rover goes outside of center cell
        """
        d_north = (self.north - self.grid_center[0])
        d_west = (self.west - self.grid_center[1])
        self.grid_shift = [d_north, d_west]

        if abs(self.grid_shift[0]) > self.cell_size:
            # Shift grid to keep rover centered
            shift = round(-1.0 * self.grid_shift[0]/self.cell_size)
            # Row shift (units in cells)
            self.shift_rows(shift)
            # Grid center represented in NW meters
            self.grid_center[0] -= shift * self.cell_size
            # Grid shift for precise positioning relative to obstacles
            self.grid_shift[0] %= (-1.0 * shift * self.cell_size)

        if abs(self.grid_shift[1]) > self.cell_size:
            # Shift grid to keep rover centered
            shift = round(-1.0 * self.grid_shift[1]/self.cell_size)
            # Column shift (units in cells)
            self.shift_columns(shift)
            # Grid center represented in NW meters
            self.grid_center[1] -= shift * self.cell_size
            # Grid shift for precise positioning relative to obstacles
            self.grid_shift[1] %= (-1.0 * shift * self.cell_size)

        #self.get_logger().info(f"{self.grid_shift}")
        self.publish_costmap_shift()

    def shift_rows(self, amount, fill_value=0):
        """
        Shift the rows of the grid by a specified amount
        """
        if amount == 0:
            return
        # Integer shift in row if the robot moved north:
        # Swapped to account for rerun coordinates
        shift_rows = amount   # north cell displacement

        # Shift rows
        if shift_rows != 0:
            self.grid = np.roll(self.grid, shift_rows, axis=0)
            # Fill new rows at top or bottom
            if shift_rows > 0:
                # Robot moved north -> new "bottom" rows become unknown
                self.grid[:shift_rows, :] = fill_value
            else:
                # Robot moved south -> new "top" rows become unknown
                self.grid[self.grid_length+shift_rows:, :] = fill_value


    def shift_columns(self, amount, fill_value=0):
        """
        Shift the columns of the grid by a specified amount
        """
        if amount == 0:
            return
        # Integer shift in column if the robot moved west:
        shift_cols = amount   # west cell displacement

        # Shift columns
        if shift_cols != 0:
            self.grid = np.roll(self.grid, shift_cols, axis=1)
            # Fill new columns on left or right
            if shift_cols > 0:
                # Robot moved west -> new "right" columns become unknown
                self.grid[:, :shift_cols] = fill_value
            else:
                # Robot moved east -> new "left" columns become unknown
                self.grid[:, self.grid_length+shift_cols:] = fill_value

    def publish_costmap_shift(self):
        """
        Publishes costmap shift
        """
        costmap_shift = Float32MultiArray()
        costmap_shift.data = [float(self.grid_shift[0]), float(self.grid_shift[1])]
        self.costmap_position_publisher.publish(costmap_shift)

    def pointcloud_callback(self, msg):
        """
        Process incoming PointCloud2 messages
        """
        
        self.grid_init = True
        self.curr_obstacles = set()
        self.curr_free_space = set()
        #new_obstacles = set()

        for pt in point_cloud2.read_points(msg, skip_nans=True):
            obstacle = self.point_cloud_point_to_grid(pt)

        self.decay_cost()

    def publish_new_obstacles(self, new_obstacles):
        """
        Publish newly detected obstacles from point cloud
        """
        msg = Float32MultiArray()
        data = []
        for (x,y) in new_obstacles:
            data.extend([float(x), float(y)])
        msg.data = data
        self.new_obstacle_publisher.publish(msg)


    def point_cloud_point_to_grid(self, pt):
        '''
        Convert point cloud point to grid value, utilizing current yaw
        Takes into account coordinate system of camera, where x is right, y is down, and z is forward
        In the 2D grid, x is forward (north) and y is to the left (west), same as global coordinates in gazebo
        Returns True if new obstacle detected, False if not
        '''
        new_obstacle = (False, None)
        height = -1.0 * pt[1]
        x, y = pt[2], pt[0]
        if x > self.max_depth - 0.5:
            return new_obstacle
        rotated_pt = self.R.dot(np.array([x, y]))
        x_rot = rotated_pt[0]
        y_rot = rotated_pt[1]
        if x_rot is None or abs(x_rot) > self.grid_size or y_rot is None or abs(y_rot) > self.grid_size:
            return new_obstacle
        # want to normalize x and y to integer values in the range (0 to grid_length/2, -grid_length/2 to grid_length/2)
        #x_new = int(((x_rot-self.grid_shift[0])/self.cell_size) + self.robot_grid_position[0] - 1)
        #y_new = int(self.grid_length - (((y_rot+self.grid_shift[1])/self.cell_size) + self.robot_grid_position[1] - 1))
        x_new = int(((x_rot)/self.cell_size) + self.robot_grid_position[0] - 1)
        y_new = int(self.grid_length - (((y_rot)/self.cell_size) + self.robot_grid_position[1] - 1))
        # don't update if out of grid bounds or previously detected an obstacle at that grid location
        if not (0 <= x_new < self.grid_length) or not (0 <= y_new < self.grid_length) or (x_new, y_new) in self.curr_obstacles:
            return new_obstacle
            
        # Store traversable cells to decay
        if (self.expected_height - self.height_threshold < height < self.expected_height + self.height_threshold) or height > self.clearance_height:
            self.curr_free_space.add((x_new, y_new))
            return new_obstacle
            #self.grid[x_new, y_new] = max(0, self.grid[x_new, y_new]-1)

        # increment cell cost if height seems to represent obstacle
        else:
            self.grid[x_new, y_new] = min(self.max_cost, self.grid[x_new, y_new]+1)
            if (x_new, y_new) not in self.curr_obstacles:
                self.curr_obstacles.add((x_new, y_new))
                '''x_global = self.grid_center[0] + ((x - self.center_index) * self.cell_size)
                y_global = self.grid_center[1] + ((y - self.center_index) * self.cell_size)
                x_global = round(x_global * 2) / 2
                y_global = round(y_global * 2) / 2
                if self.grid[x_new, y_new] >= self.obstacle_cost_threshold and (x_global, y_global) not in self.obstacles_global:
                    self.obstacles_global.add((x_global, y_global))
                    new_obstacle = (True, (x_global, y_global))'''
        return new_obstacle
    

    def decay_cost(self):
        """
        Linearly decay cost of grid cells if an 
        obstacle not actively detected in that region
        """
        for x, r in enumerate(self.grid):
            for y, cost in enumerate(r):
                if (x, y) not in self.curr_free_space or (x, y) in self.curr_obstacles:
                    continue
                self.grid[x, y] = max(0, self.grid[x, y]-1)
    
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