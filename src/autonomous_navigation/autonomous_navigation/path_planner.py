#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Twist
import numpy as np
import random
import rerun as rr
import math
from std_msgs.msg import Int16, Float32MultiArray, Bool
from typing import List, Tuple, Optional
from cmr_msgs.msg import AutonomyDrive
import time
import heapq
from scipy.spatial import distance


class PIDController:
    """
    Simple PID Controller
    """
    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        correcting: bool,
        output_limits: Tuple[Optional[float], Optional[float]] = (None, None),
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_steer_angle = 0.0
        self.max_d = 0.2
        self.output_limits = output_limits
        self.last_time: Optional[rclpy.time.Time] = None
        self.correcting = correcting

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = None

    def compute(self, error: float, current_time: rclpy.time.Time) -> float:
        """
        Compute the PID controller output.

        :param error: The current error value.
        :param current_time: The current time (rclpy Time object).
        :return: The control output.
        """

        if self.last_time is None:
            delta_time = 0.0
        else:
            delta_time = (current_time - self.last_time).nanoseconds * 1e-9

        self.last_time = current_time

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * delta_time
        I = self.Ki * self.integral

        # Derivative term
        derivative = (
            (error - self.previous_error) / delta_time if delta_time > 0 else 0.0
        )
        D = self.Kd * derivative

        # Save error for next derivative calculation
        self.previous_error = error

        # Compute the output
        output = P + I + D

        # Apply output limits
        lower, upper = self.output_limits
        lower = max(lower, self.previous_steer_angle - self.max_d)
        upper = min(upper, self.previous_steer_angle + self.max_d)
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)
        self.previous_steer_angle = output
        
        return output
    

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter('visualize', True)
        self.declare_parameter('real', False) # FALSE IF RUNNING IN SIMULATION
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.real = self.get_parameter('real').get_parameter_value().bool_value 

        if self.visualize:
            # Initialize Rerun visualization
            rr.init("ground_plane_grid", spawn=True)

        # Callback group to prevent concurrent execution
        self.coord_target_group = MutuallyExclusiveCallbackGroup()
        
        # Subscribe to the point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.pointcloud_callback,
            10
        )
        
        # Subscribe to the robot pose topic
        self.pose_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/pose',
            self.update_pose,
            10
        )

        self.target_offset_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/coordinate_error',
            self.define_grid_target,
            10
        )

        self.waypoint_reached_subscription = self.create_subscription(
            Bool,
            '/autonomy/waypoint_reached',
            self.waypoint_reached_callback,
            10,
            callback_group=self.coord_target_group
        )

        # True if the current action is driving to a GPS coordinate
        self.drive_to_coord = True

        # Autonomy Movement Publishers
        self.ackerman_publisher = self.create_publisher(AutonomyDrive, '/autonomy/move/ackerman', 10)
        self.point_turn_publisher = self.create_publisher(Twist, '/autonomy/move/point_turn', 10)
        self.max_vel = 0.05
        # Timer for path following function (f_drive Hz)
        self.f_drive = 0.1
        self.follow_timer = self.create_timer(self.f_drive, self.follow_path, callback_group=self.coord_target_group)
        # Keep track of last autonomous movement to allow a pause when switching
        # between point turns and regular driving ("turn", or "drive")
        self.last_move = "drive"
        # Required wait between point turns and regular driving in seconds
        self.min_wait = 0.2
        self.curr_wait = 0.0

        # Timer for updating path (f_update Hz)
        self.f_update = 0.1
        self.update_timer = self.create_timer(self.f_update, self.update_path)
        self.current_path_valid = False

        # Initialize PID controller for ackerman
        self.pid = PIDController(
            Kp=0.5,
            Ki=0.0,
            Kd=0.1,
            correcting=False,
            output_limits=(-math.radians(45), math.radians(45)),
        )
        self.get_logger().info("Initialized PID controller for ackerman control")

        # Displacement from original position & rotation of robot, updated with data from navigation control loop node
        self.north = 0.0
        self.west = 0.0
        self.yaw = 0.0

        self.true_north = 0.0
        self.true_west = 0.0
        # Rotation matrix, continually updated, to convert point cloud points from
        # rotated frame back to global
        self.R = np.array([
            [np.cos(-1.0 * self.yaw), -np.sin(-1.0 * self.yaw)],
            [np.sin(-1.0 * self.yaw),  np.cos(-1.0 * self.yaw)]
        ])

        # Changes in displacements & rotation of robot from previous timestep
        self.d_north = 0.0
        self.d_west = 0.0
        self.d_yaw = 0.0

        # Parameters for the grid
        self.square_size = 0.5  # Side length of square in grid in meters
        self.grid_size = 40.0  # Grid side length in meters, axis range from -grid_size/2 to grid_size/2
        # Heightmap where value of (x,y) -1 - not traversable, 0 - unknown, 1 - traversablev
        # Coordinates of grid match Gazebo when there is no yaw
        # x - forward, initially north
        # y - left, initially west
        # z - up, north
        self.grid_length = int(self.grid_size/self.square_size)
        self.center_index = int(self.grid_length/2)
        self.grid = np.zeros((self.grid_length, self.grid_length), dtype=np.int8)
        # Default grid target is the current robot position
        self.grid_target = (self.center_index, self.center_index)
        self.grid_init = False
        self.robot_pos = (self.center_index, self.center_index)
        
        # Store obstacles most recently detected
        self.curr_obstacles = set()

        # Ground detection thresholds
        self.height_threshold = 0.2
        self.gradient_threshold = 0.2
        self.expected_height = -1.0
        self.clearance_height = 2.0
        self.max_depth = 20.0

        # Distance threshold in grid units for avoiding obstacles when path planning
        self.max_obstacle_distance = 3
        
        # Variable to hold the current planned path for the rover
        self.curr_path = []

        # Variables to visualize tree expansion from rover and goal
        self.start_tree_lines = []
        self.goal_tree_lines = []
        self.path_failure_point = None

        # Timer to redraw current state
        if self.visualize:
            self.redraw_timer = self.create_timer(0.2, self.plot_grid_rerun)


    def update_path(self):
        """
        Function to update the current path on a timer
        """
        if not self.drive_to_coord or not self.grid_init or self.grid_target == (self.center_index, self.center_index):
            return
        
        
        # Recompute the path if the current one is invalid or not yet been computed
        while self.curr_path == []:
            self.curr_path = self.compute_path()
            #self.get_logger().info(f"After recomputing, current path is {self.current_path_valid}")
        self.validate_path()
        if not self.current_path_valid:
            new_path = self.compute_path()
            if new_path != []:
                self.curr_path = new_path
            else:
                self.max_obstacle_distance = max(1, self.max_obstacle_distance-1)

    def follow_path(self):
        """
        Function to send appropriate drive commands to follow the current path
        """
        if not self.drive_to_coord or not self.current_path_valid:
            return
        
        # Current path should have at least two points,
        # the first of which is the current rover position
        if len(self.curr_path) < 2:
            return
        
        p1 = self.curr_path[1]
        dx = p1[0] - self.center_index
        dy = p1[1] - self.center_index
        '''if min(abs(dx),abs(dy)) > (5/self.square_size):
            self.max_vel = 0.07
        else:
            self.max_vel = 0.06'''

        angle_to_target = math.atan2(dy, dx)  # desired heading in radians
        # Calculate angle error
        angle_error = angle_to_target - self.yaw
        
        # Driving logic
        if abs(math.degrees(angle_error)) < 10:
            if self.last_move == "turn" and self.curr_wait < self.min_wait:
                self.curr_wait += self.f_drive
                # Orient wheels for next driving command
                self.publish_ackerman(0.0, math.degrees(angle_error))
                return
            else:
                self.last_move = "drive"
                self.curr_wait = 0.0
                self.publish_ackerman(self.max_vel, math.degrees(angle_error))
        else:
            if self.last_move == "drive" and self.curr_wait < self.min_wait:
                self.curr_wait += self.f_drive
                ### NEED COMMAND FOR ORIENTING WHEELS FOR POINT TURNING ###
                self.stop_robot()
                return
            else:
                self.last_move = "turn"
                self.curr_wait = 0.0
                if abs(angle_error) > np.pi:
                    self.point_turn(math.degrees(angle_error))
                else:
                    self.point_turn(-1.0 * math.degrees(angle_error))


    def point_turn(self, degrees):
        ### NEED TO MAKE CORRESPONDING POINT TURN LISTENER FOR REAL ROVER DRIVES CONTROLS ###
        """
        Function to publish point turns, takes in a desired turn amount
        of the rover in degrees
        """
        turn = Twist()
        turn.angular.z = degrees
        self.point_turn_publisher.publish(turn)


    def compute_path(self):
        """
        Compute the path using A* and visualize or log the results.
        """
        if not self.drive_to_coord or self.grid_target == (self.center_index, self.center_index):
            return []
        start = (self.center_index, self.center_index)
        goal = self.grid_target

        #self.get_logger().info(f"Computing path from {start} to {goal}")

        #return self.bi_rrt_path(start, goal, max_obstacle_distance=self.max_obstacle_distance)
        new_path = self.bi_rrt_with_cost(start, goal)
        if new_path is None:
            return []
        return new_path

    def define_grid_target(self, msg):
        """
        msg data is offsets of format [north (m), west (m), heading/yaw (rad)]
        """

        north = msg.data[0]
        west = msg.data[1]
        angle_error = msg.data[2]

        current_time = self.get_clock().now()
        steer_angle = math.degrees(self.pid.compute(angle_error, current_time))

        #self.publish_ackerman(self.max_vel, steer_angle)

        X = north
        Y = west
        dx = north
        dy = -1 * Y
        if abs(X) > self.grid_length/2 or abs(Y) > self.grid_length/2:
            if abs(X) > abs(Y):
                Y *= (self.grid_length/(2*abs(X)))
                if X < 0: 
                    X = -1.0 * self.grid_length/2
                else:
                    X = self.grid_length/2
            else:
                X *= (self.grid_length/(2*abs(Y)))
                if Y < 0: 
                    Y = -1.0 * self.grid_length/2
                else:
                    Y = self.grid_length/2
        
        dx = min(self.center_index - 1, max(-1.0 * self.center_index, X/self.square_size))
        dy = min(self.center_index - 1, max(-1.0 * self.center_index, Y/self.square_size))
        
        #self.get_logger().info(f"dx: {dx}  dy: {dy}")
        new_target = (int(self.center_index + dx), int(self.center_index + dy))
        
        if self.drive_to_coord:
            tolerance = 5
            difference = math.sqrt(((new_target[0] - self.grid_target[0])**2) + ((new_target[1] - self.grid_target[1])**2))
            if self.grid_target == (self.center_index, self.center_index) or difference >= tolerance:
                self.grid_target = new_target
                self.curr_path = [(self.center_index, self.center_index), self.grid_target]


    def publish_ackerman(self, vel, steer_angle):
        """"
        Publish ackerman driving commands during autonomous navigation
        """
        drive_msg = AutonomyDrive()
        drive_msg.vel = vel
        if self.real:
            # Real rover has a reversed sign for front angles
            drive_msg.fl_angle = -steer_angle
            drive_msg.fr_angle = -steer_angle
        else:
            drive_msg.fl_angle = steer_angle
            drive_msg.fr_angle = steer_angle
        drive_msg.bl_angle = 0.0
        drive_msg.br_angle = 0.0

        self.ackerman_publisher.publish(drive_msg)


    def waypoint_reached_callback(self, msg):
        """
        Stop the robot and pause if a gps waypoint has been reached
        """
        self.stop_robot()
        self.curr_path = []
        self.grid_target = (self.center_index, self.center_index)
        time.sleep(2)


    def update_pose(self, msg):
        '''
        Update pose and change in pose
        '''
        north = msg.data[0]
        west = msg.data[1]
        yaw = msg.data[2]
        
        self.shift_path(self.true_north - north, self.true_west - west)
        self.true_north = north
        self.true_west = west

        self.d_north = north - self.north
        self.d_west = west - self.west
        self.d_yaw = yaw - self.yaw
        self.yaw = yaw
        self.R = np.array([
            [np.cos(-1.0 * self.yaw), -np.sin(-1.0 * self.yaw)],
            [np.sin(-1.0 * self.yaw),  np.cos(-1.0 * self.yaw)]
        ])
        # Only update grid and pose if there is at least a half a square size change in a given direction, 
        # the shifting functions themselves will update north and west values
        rows = abs(self.d_north) > (self.square_size/2)
        columns = abs(self.d_west) > (self.square_size/2)

        if rows or columns:
            dr, dc = 0, 0
            if rows:
                dr = self.shift_rows()
            if columns:
                dc = self.shift_columns()
            
            self.shift_target(dr, dc)


    def shift_target(self, dr, dc):
        """
        Update the current grid target based on grid shift
        """
        if self.grid_target == (self.center_index, self.center_index):
            return
        
        new_target = (self.grid_target[0] + dr, self.grid_target[1] + dc)
        self.grid_target = new_target


    def shift_path(self, dr, dc):
        """
        Update the current path based on grid shift
        """
        if len(self.curr_path) < 2:
            return
        
        shifted_path = []
        for p in self.curr_path:
            shifted_path.append((p[0] + dr, p[1] + dc))
        
        # Remove the second point of first segment in path if close
        # to current robot position
        d = self.dist((self.center_index, self.center_index), shifted_path[1])
        #self.get_logger().info(f"Distance to first point after shifting path: {d}")
        if d < 1:
            self.curr_path = shifted_path[1:]
        else:
            self.curr_path = shifted_path
        self.curr_path[0] = self.robot_pos
        self.curr_path[-1] = self.grid_target

    def shift_rows(self, fill_value=0):
        """
        Shift the rows of the grid
        """
        # Integer shift in row if the robot moved north:
        # Swapped to account for rerun coordinates
        shift_rows = int(-1.0 * self.square_size * round(self.d_north))   # north displacement
        shift_dist = shift_rows * self.square_size

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
            
            # Update values to account for grid shift
            self.d_north += shift_dist
            self.north -= shift_dist
        return shift_rows


    def shift_columns(self, fill_value=0):
        """
        Shift the column of the grid
        """
        # Integer shift in column if the robot moved west:
        # Swapped to account for rerun coordinates
        shift_cols = int(-1.0 * self.square_size * round(self.d_west))
        shift_dist = shift_cols * self.square_size
        #self.get_logger().info(f"Shifting columns by {shift_cols}")

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
            
            # Update values to account for grid shift
            self.d_west += shift_dist
            self.west -= shift_dist
        return shift_cols
    

    def pointcloud_callback(self, msg):
        """Process incoming PointCloud2 messages and visualize traversability estimate."""
        if not self.drive_to_coord:
            return
        
        self.grid_init = True
        self.curr_obstacles = set()
        new_obstacle = False
        for pt in point_cloud2.read_points(msg, skip_nans=True):
            obstacle = self.point_cloud_point_to_grid(pt)
            new_obstacle = obstacle or new_obstacle


    def point_cloud_point_to_grid(self, pt):
        '''
        Convert point cloud point to grid value, utilizing current yaw
        Takes into account coordinate system of camera, where x is right, y is down, and z is forward
        In the 2D grid, x is forward (north) and y is to the left (west), same as global coordinates in gazebo
        Returns True if new obstacle detected, False if not
        '''
        new_obstacle = False
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
        x_new = int((x_rot/self.square_size) + self.center_index - 1)
        y_new = int(self.grid_length - ((y_rot/self.square_size) + self.center_index - 1))
        # don't update if out of grid bounds or previously detected an obstacle at that grid location and not much movement has occurred since
        if not (0 <= x_new < self.grid_length) or not (0 <= y_new < self.grid_length) or (x_new, y_new) in self.curr_obstacles:
            return new_obstacle
        # mark as traversable if height seems to be ground
        if (self.expected_height - self.height_threshold < height < self.expected_height + self.height_threshold) or height > self.clearance_height:
            self.grid[x_new, y_new] = 1
        
        elif self.grid[x_new, y_new] != -1:
            new_obstacle = True
            self.grid[x_new, y_new] = -1
            self.curr_obstacles.add((x_new, y_new))

        return new_obstacle


    def stop_robot(self):
        '''
        Sends a drive command to the robot to stop moving
        '''
        self.publish_ackerman(0.0, 0.0)
        self.get_logger().info("Robot stopped.")


    def plot_grid_rerun(self):
        """Visualize the grid"""
        points = []
        colors = []
        for x, r in enumerate(self.grid):
            for y, val in enumerate(r):
                points.append([x,y,0])
                if val == 1:
                    colors.append([0, 255, 0])
                elif val == -1:
                    colors.append([255, 0, 0])
                else:
                    colors.append([128, 128, 128])

        rr.log("traversability_estimate", rr.Points3D(np.array(points, dtype=np.float32), colors=colors, radii=0.1))
        rr.log("current_position", rr.Points3D(np.array([[self.center_index,self.center_index,0]]), colors=[0,0,255], radii=0.5))
        
        # Plot grid target
        if self.grid_target != (self.center_index, self.center_index):
            rr.log("target", rr.Points3D(np.array([[self.grid_target[0],self.grid_target[1],0]]), colors=[128,0,128], radii=0.25))
        
        # Plot current path
        if self.curr_path != []:
            points = []
            for point in self.curr_path:
                points.append([point[0], point[1], 1])

            rr.log("path", rr.LineStrips3D(
                np.array(points, dtype=np.float32),
                radii=0.05,
                colors=[0, 0, 255]
            ))
        else:
            rr.log("path", rr.Clear(recursive=True))

        # Plot tree expansions
        if len(self.curr_path) > 2:   
            if self.start_tree_lines != []:
                rr.log("tree/start", rr.LineStrips3D(
                    np.array(self.start_tree_lines, dtype=np.float32),
                    radii=0.1,
                    colors=[255, 255, 255]
                ))
            if self.goal_tree_lines != []:
                rr.log("tree/goal", rr.LineStrips3D(
                    np.array(self.goal_tree_lines, dtype=np.float32),
                    radii=0.1,
                    colors=[255, 140, 0]
                ))
        else:
            rr.log("tree", rr.Clear(recursive=True))

        # Draw the x (forward) axis of the robot
        start = [self.center_index, self.center_index, 0.0]
        reverseR = np.array([
            [np.cos(self.yaw), -np.sin(self.yaw)],
            [np.sin(self.yaw),  np.cos(self.yaw)]
        ])
        x_vec = reverseR.dot(np.array([2.0, 0]))
        x_end = [start[0] + x_vec[0], start[1] + x_vec[1], start[2]]

        #self.get_logger().info(f"{forward_vec}")

        # Draw the orientation
        rr.log("robot_direction", rr.LineStrips3D(
            np.array([start, x_end]),
            radii=0.1,
            colors=[255,0,255]
        ))


    def bi_rrt_with_cost(self, start, goal, max_iter=1000, step_size=1.0, connect_dist=2.0, w1=1.0, w2=1.0):
        """
        Bi-RRT with a cost heuristic that minimizes path distance and penalizes proximity to obstacles.

        :param start: Tuple (x, y) for the starting position.
        :param goal: Tuple (x, y) for the goal position.
        :param max_iter: Maximum number of iterations to run the algorithm.
        :param step_size: Step size for tree expansion.
        :param connect_dist: Distance threshold for connecting the two trees.
        :param w1: Weight for path distance in the cost heuristic.
        :param w2: Weight for proximity penalty in the cost heuristic.
        :return: The lowest-cost path found or None if no valid path is found.
        """

        def is_within_bounds(point):
            x, y = point
            return 0 <= x < self.grid.shape[0] and 0 <= y < self.grid.shape[1]

        def is_collision_free(p1, p2):
            """
            Check if a segment between two points is collision-free.
            """
            #num_samples = int(max(abs(p1[0] - p2[0]), abs(p1[1] - p2[1])))
            num_samples = 3
            for t in np.linspace(-1, 1, num_samples):
                px = int(p1[0] * (1 - t) + p2[0] * t)
                py = int(p1[1] * (1 - t) + p2[1] * t)
                if is_within_bounds((px, py)) and self.grid[px, py] == -1:
                    return False
            return True

        def euclidean_distance(p1, p2):
            return np.linalg.norm(np.array(p1) - np.array(p2))

        def proximity_penalty(point):
            """
            Compute proximity penalty for a point based on its distance to the nearest obstacle.
            """
            x, y = int(point[0]), int(point[1])
            for radius in range(1, int(connect_dist) + 1):
                for dx in range(-radius, radius + 1):
                    for dy in range(-radius, radius + 1):
                        nx, ny = x + dx, y + dy
                        if is_within_bounds((nx, ny)) and self.grid[nx, ny] == -1:
                            return 1.0 / radius
            return 0

        def path_cost(path):
            """
            Compute the total cost of a path based on distance and proximity penalties.
            """
            distance_cost = sum(euclidean_distance(path[i], path[i + 1]) for i in range(len(path) - 1))
            proximity_cost = sum(proximity_penalty(point) for point in path)
            return w1 * distance_cost + w2 * proximity_cost

        def grow_tree(tree, point, direction):
            """
            Expand the tree toward a new point.
            """
            dx, dy = direction
            new_point = (point[0] + dx * step_size, point[1] + dy * step_size)
            if not is_collision_free(point, new_point):
                return None
            return new_point

        def connect_trees(tree_a, tree_b):
            """
            Attempt to connect two trees directly if close enough.
            """
            for node_a in tree_a:
                for node_b in tree_b:
                    if euclidean_distance(node_a, node_b) <= connect_dist and is_collision_free(node_a, node_b):
                        return node_a, node_b
            return None, None

        def reconstruct_path(tree, start_node, end_node):
            """
            Reconstruct a path by tracing parent pointers in the tree.
            Returns path, bool indicating if path traces back fully to end
            """
            path = []
            current = end_node
            complete = False
            while current is not None:
                if current == start_node:
                    complete = True
                path.append(current)
                current = tree.get(current, None)
            
            return path[::-1], complete  # Reverse path to go from start_node to end_node

        def smooth_path(path):
            """
            Post-process the path to remove unnecessary nodes and smooth it.
            """
            smoothed_path = [path[0]]
            for i in range(1, len(path)):
                if not is_collision_free(smoothed_path[-1], path[i]):
                    smoothed_path.append(path[i - 1])
            smoothed_path.append(path[-1])
            return smoothed_path

        # Initialize trees as dictionaries
        tree_a = {tuple(start): None}  # Key: Node, Value: Parent
        tree_b = {tuple(goal): None}

        best_path = None
        best_cost = float('inf')

        for i in range(max_iter):
            self.get_logger().info(f"{i}")
            pts = []
            # Alternate growth between Tree A and Tree B
            for active_tree, passive_tree in [(tree_a, tree_b), (tree_b, tree_a)]:
                # Sample a random point
                random_point = (np.random.uniform(0, self.grid.shape[0]), np.random.uniform(0, self.grid.shape[1]))
                nearest_node = min(active_tree.keys(), key=lambda node: euclidean_distance(node, random_point))
                direction = ((random_point[0] - nearest_node[0]) / euclidean_distance(nearest_node, random_point),
                            (random_point[1] - nearest_node[1]) / euclidean_distance(nearest_node, random_point))
                new_point = grow_tree(active_tree, nearest_node, direction)
                if new_point is None:
                    continue
                pts.append([new_point[0], new_point[1], 0])
                # Add new point to the active tree
                active_tree[tuple(new_point)] = tuple(nearest_node)

                # Attempt to connect to the other tree
                node_a, node_b = connect_trees(active_tree.keys(), passive_tree.keys())
                if node_a is not None and node_b is not None:
                    # Build the path
                    path_a, a_complete = reconstruct_path(tree_a, start, node_a)
                    path_b, b_complete = reconstruct_path(tree_b, goal, node_b)
                    if not a_complete and b_complete:
                        continue
                    full_path = path_a + path_b[::-1]  # Combine the two paths
                    current_cost = path_cost(full_path)

                    # Update best path if this one is better    
                    if current_cost < best_cost:
                        best_path = full_path
                        best_cost = current_cost

            rr.log(f"rrt/{i}", rr.Points3D(np.array(pts), colors=[255,255,255], radii=0.25))
        self.get_logger().info(f"{best_path}")
        # Post-process the path to smooth it
        if best_path is not None:
            best_path = smooth_path(best_path)
        self.get_logger().info(f"{best_path}")
        return best_path




    ###################
    ###################
    ###################

    def bi_rrt_path(self, start, goal, max_obstacle_distance=1, max_iter=1000, angle_step=15, target_vs_expansion_weight=4.0):
        """
        Bi-Directional RRT implementation that grows trees from both the start and
        goal nodes. Expands in straight lines until encountering obstacles, then
        iteratively samples angles to find the best path.

        :param start: Tuple (x, y) for the start position in grid coordinates
        :param goal: Tuple (x, y) for the goal position in grid coordinates
        :param max_obstacle_distance: Distance (in grid cells) to stop before obstacles
        :param max_iter: Maximum number of iterations for the algorithm
        :param angle_step: Angle step size (in degrees) for sampling directions
        :return: A list of (x, y) tuples representing the path from start to goal
        """

        def is_within_bounds(point, grid):
            x, y = point
            return 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]

        def is_near_obstacle(point, grid, threshold):
            x, y = int(point[0]), int(point[1])
            if not is_within_bounds((x, y), grid):
                return True
            for dx in range(-threshold, threshold + 1):
                for dy in range(-threshold, threshold + 1):
                    nx, ny = x + dx, y + dy
                    if is_within_bounds((nx, ny), grid) and grid[nx, ny] == -1:
                        return True
            return False

        def expand_line(point, goal_point, direction, curr_direction, grid, max_distance, obstacle_threshold):
            # Expand the line to the optimal point based on proximity to the goal point and length
            x, y = point
            dx, dy = np.cos(direction), np.sin(direction)
            best_point = None
            best_score = float('-inf')
            for i in range(1, max_distance + 1):
                new_point = (x + i * dx, y + i * dy)
                new_point = tuple(map(int, np.round(new_point)))  # Ensure integer keys
                # Stop checking points if close to an obstacle
                if not is_within_bounds(new_point, grid):
                    break
                dist = np.linalg.norm(np.array(new_point) - np.array(goal_point))
                length = np.linalg.norm(np.array(new_point) - np.array(point))
                score = path_score(dist, length, abs(direction - curr_direction), is_near_obstacle(new_point, grid, obstacle_threshold))
                if best_point is None or score > best_score:
                    best_point = new_point
                    best_score = score

            return best_point, best_score


        def find_best_direction(point, grid, goal_point, angle_step, max_distance, min_distance, obstacle_threshold, curr_direction, r=(-90,90)):

            best_direction = None
            best_score = float('-inf')
            best_point = None
            # Get all points for visualizing search
            all_points = []

            for angle in range(r[0], r[1]+1, angle_step):
                direction = curr_direction + np.deg2rad(angle)
                new_point, score = expand_line(point, goal_point, direction, curr_direction, grid, max_distance, obstacle_threshold)
                if score > best_score:
                    best_score = score
                    best_direction = direction
                    best_point = new_point
                else:
                    all_points.append(new_point)

            return best_point, best_direction, all_points

        def path_score(dist_to_goal, segment_length, d_direction, near_obstacle=False, weight=target_vs_expansion_weight):
            score = (-dist_to_goal*weight) + segment_length
            # Need better cost function without adding scalars
            if near_obstacle:
                score -= 100.0
            return score

        def smooth_path(path, min_dist):
            
            if len(path) < 3:
                return path

            smoothed_path = []
            i = 0

            while i < (len(path)-1):
                p1, p2  = path[i], path[i+1]
                dist = math.sqrt(((p2[0]-p1[0])**2) + ((p2[1]-p1[1])**2))
                if dist < min_dist:
                    new_point = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
                    smoothed_path.append(new_point)
                    i += 2
                else:
                    smoothed_path.append(p1)
                    i += 1
            
            smoothed_path.append(path[-1])
            return smoothed_path


        def build_path(tree, node):
            path = []
            while node is not None:
                if node not in tree:  # Debugging missing key
                    #raise KeyError(f"Node {node} is not in the tree!")
                    break
                path.append(node)
                node = tree[node]
            return path[::-1]

        # Clear variables to visualize tree expansion lines
        self.start_tree_lines = []
        self.goal_tree_lines = []

        # Initialize trees for start and goal
        tree_start = {start: None}
        tree_goal = {goal: None}
        
        #self.get_logger().info(f"Initial expansion direction (current yaw of robot): {math.degrees(self.yaw)}")
        directions_start = {start: self.yaw}  # Initial direction for the start tree
        directions_goal = {goal: (self.yaw + np.pi)}    # Initial direction for the goal tree
        # Maximum straight-line distance before encountering obstacles
        max_distance = max_obstacle_distance * 10
        # Minimum straight-line distance for a path segment
        min_distance = max_obstacle_distance
        
        for i in range(max_iter):
            # Expand tree from start
            for (tree, directions, other_tree) in [(tree_start, directions_start, tree_goal),
                                                (tree_goal, directions_goal, tree_start)]:
                current_node = list(tree.keys())[-1]  # Latest added node
                goal_node = list(other_tree.keys())[-1]  # Latest node in the other tree
                tree_to_add = "start"
                if tree is tree_goal:
                    # only want to expand the goal tree for two iterations
                    if i > 3:
                        continue
                    tree_to_add = "goal"
                current_direction = directions[current_node]
                
                # Add point for visualization
                if tree_to_add == "start":
                    self.start_tree_lines.append([current_node[0], current_node[1], 0])
                else:
                    self.goal_tree_lines.append([current_node[0], current_node[1], 0])

                # Find the best direction and expand
                new_point, new_direction, all_points = find_best_direction(
                    current_node, self.grid, goal_node, angle_step, max_distance, 
                    min_distance, max_obstacle_distance, current_direction, r=(-90,90)
                )

                # Check between [91:270] if segment not found between [-90:90]
                if new_point is None:
                    new_point, new_direction, all_points = find_best_direction(
                        current_node, self.grid, goal_node, angle_step, max_distance, 
                        min_distance, max_obstacle_distance, current_direction, r=(91,270)
                    )
                    
                if new_point is None or tuple(new_point) in tree:
                    continue
                
                # For visualization
                new_strips = []
                for p in all_points:
                    if p is None:
                        continue
                    new_strips.append([p[0], p[1], 0])
                    new_strips.append([current_node[0], current_node[1], 0])
                new_strips.append([new_point[0], new_point[1], 0])

                if tree_to_add == "start":
                    self.start_tree_lines.extend(new_strips)
                else:
                    self.goal_tree_lines.extend(new_strips)

                # Add new point to the tree
                tree[tuple(new_point)] = current_node
                directions[tuple(new_point)] = new_direction

                # Check if the new point connects with the other tree
                for other_node in other_tree:
                    if np.linalg.norm(np.array(new_point) - np.array(other_node)) <= max_obstacle_distance:
                        # Combine paths from both trees
                        path_start = build_path(tree_start, tuple(new_point))
                        path_goal = build_path(tree_goal, other_node)
                        full_path = path_start + path_goal[::-1]  # Combine paths from both trees
                        return smooth_path(full_path, max_obstacle_distance)

        # Return empty if no path is found within max_iter
        return []


    def validate_path(self, max_obstacle_distance=1):
        """
        Validates if a path violates the maximum allowed distance from obstacles.

        :param path: List of (x, y) tuples representing the planned path
        :param grid: 2D NumPy array of the current grid (-1 = obstacle, 1 = free, 0 = unknown)
        :param max_obstacle_distance: Maximum allowed distance (in grid cells) from an obstacle
        :return: (is_valid, invalid_index)
                - is_valid: True if the path is valid, False otherwise
                - invalid_index: Index of the first invalid segment if the path is invalid, None if valid
        """

        def is_within_bounds(point, grid):
            x, y = int(point[0]), int(point[1])
            return 0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]

        def is_near_obstacle(point, grid, threshold):
            """
            Checks if a given point is within the maximum allowed distance to an obstacle.
            """
            x, y = int(point[0]), int(point[1])
            if not is_within_bounds(point, grid):
                return True  # Treat out-of-bounds points as invalid

            for dx in range(-threshold, threshold + 1):
                for dy in range(-threshold, threshold + 1):
                    nx, ny = x + dx, y + dy
                    if is_within_bounds((nx, ny), grid) and grid[nx, ny] == -1:
                        return True
            return False

        # Validate each segment of the path by sampling and checking points on each segment
        for i in range(len(self.curr_path)-1):
            start = self.curr_path[i]
            end = self.curr_path[i+1]
            num_points = max(1, int(self.dist(start,end)//2))
            points = np.linspace(start, end, num_points)
            for point in points:
                if is_near_obstacle(point, self.grid, max_obstacle_distance):
                    #return False, i
                    self.path_failure_point = (point[0], point[1])
                    self.current_path_valid = False
                    return
                
        self.path_failure_point = None
        self.current_path_valid = True

    def dist(self, p1, p2):
        '''
        Helper function for euclidean distance between two grid points
        '''
        return math.sqrt(((p2[0]-p1[0])**2) + ((p2[1]-p1[1])**2))

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
