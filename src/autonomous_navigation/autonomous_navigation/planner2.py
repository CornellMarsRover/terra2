#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import GroundPlaneStamped
from cv_bridge import CvBridge
import numpy as np
import math
from collections import deque
import heapq  # For priority queue in A*

import rerun as rr

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        # ------------------------------------
        # Tunable parameters for cost-based path planning
        # ------------------------------------
        self.max_cell_threshold = 2.0
        self.distance_weight = 0.1
        self.cost_weight = 1.0

        # If we want to visualize with Rerun
        self.declare_parameter('visualize', True)  # Shows rerun visualization if true
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.visualize = True

        # Subscriptions
        self.previous_target_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/target/previous',
            self.previous_target_callback,
            10
        )
        self.next_target_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/target/next',
            self.next_target_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            TwistStamped,
            '/autonomy/pose/robot/global',
            self.update_pose,
            10
        )
        self.obstacle_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/costmap',
            self.new_obstacle_callback,
            10
        )
        self.ground_plane_sub = self.create_subscription(
            GroundPlaneStamped,
            '/camera/ground_plane',
            self.ground_plane_callback,
            10
        )
        
        # Publisher for the next waypoint
        self.next_waypoint_publisher = self.create_publisher(Float32MultiArray, '/autonomy/path/next_waypoint', 10)

        # Initialize CvBridge for message conversion
        self.bridge = CvBridge()

        # Basic robot and target state
        self.robot_position = (0.0, 0.0)
        self.yaw = 0.0
        self.costmap_size = 40.0

        self.previous_target = (0.0, 0.0)  # [north (m), west (m)]
        self.next_target = (0.0, 0.0)
        self.target_yaw = None

        # Cache of discretized obstacle *positions* and a dict of all cell costs
        self.obstacles = set()  
        self.costs = dict()   # (x, y) -> cost in [0..100]

        # If cost above 4 is "obstacle" in old approach; we still fill self.obstacles
        # to keep old references, but not used in the same way
        self.threshold = 4  
        
        self.cell_size = 0.25
        self.k = 4

        # Path control
        self.current_path = deque()
        self.previous_points = deque()
        self.next_waypoint = None
        self.waypoint_threshold = 0.6
        self.invalidation_count = 0

        # Timers
        self.path_check_timer = self.create_timer(0.2, self.validate_path)
        
        self.use_stanley = False
        
        # For demonstration
        self.get_logger().info("Planner Node initialized")
        self.ground_plane = []

        if self.visualize:
            rr.init("ground_plane_grid", spawn=True)
            self.redraw_timer = self.create_timer(0.2, self.plot_grid_rerun)

    # -------------------------------------------------------------------------
    # Visualization
    # -------------------------------------------------------------------------
    def plot_grid_rerun(self):
        """
        Visualize current obstacles, path, and orientation via Rerun.
        """
        points = []
        colors = []
        radii = []
        
        # Obstacles
        for (x, y) in self.costs.keys():
            points.append([x - self.robot_position[0], y - self.robot_position[1], 0])
            color = min(255, int(255*self.costs[(x,y)]/10))
            colors.append([color, 0, 0])
            radii.append(0.1)

        # Robot position
        points.append([0, 0, 0])
        colors.append([0, 0, 255])
        radii.append(0.25)

        # Path
        for (x, y) in self.current_path:
            points.append([x - self.robot_position[0], y - self.robot_position[1], 0])
            colors.append([0, 255, 0])
            radii.append(0.1)

        # Ground plane
        if len(self.ground_plane) > 1:
            ground_pts = []
            for pt in self.ground_plane:
                ground_pts.append(pt)
            # close the loop
            ground_pts.append(self.ground_plane[0])
            rr.log("ground_plane", rr.LineStrips3D(
                np.array(ground_pts),
                radii=0.05,
                colors=[255, 255, 255]
            ))

        rr.log("path_planning_visualization", rr.Points3D(
            np.array(points, dtype=np.float32),
            colors=colors,
            radii=radii
        ))

        # Robot orientation
        reverseR = np.array([
            [np.cos(self.yaw), -np.sin(self.yaw)],
            [np.sin(self.yaw),  np.cos(self.yaw)]
        ])
        x_vec = reverseR.dot(np.array([2.0, 0]))
        x_end = [x_vec[0], x_vec[1], 0]

        rr.log("robot_direction", rr.LineStrips3D(
            np.array([[0, 0, 0], x_end]),
            radii=0.1,
            colors=[255, 0, 255]
        ))

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------
    def ground_plane_callback(self, msg):
        """
        Track the ground plane for visualization only.
        """
        self.ground_plane = []
        R = np.array([
            [np.cos(self.yaw), -np.sin(self.yaw)],
            [np.sin(self.yaw),  np.cos(self.yaw)]
        ])
        x = msg.x
        y = msg.y
        for i in range(len(x)):
            pt = R.dot(np.array([x[i], y[i]]))
            self.ground_plane.append([
                pt[0] - self.robot_position[0],
                pt[1] - self.robot_position[1],
                0
            ])

    def new_obstacle_callback(self, msg):
        """
        Costmap callback – updates local costs and "obstacles".
        """
        if len(msg.data) == 0:
            return
        self.costs = dict()
        for i in range(0, len(msg.data), 3):
            xx = msg.data[i]
            yy = msg.data[i+1]
            cost_val = msg.data[i+2]

            # Keep old obstacle set for reference
            if cost_val > self.threshold:
                self.obstacles.add((xx, yy))
            else:
                if (xx, yy) in self.obstacles:
                    self.obstacles.discard((xx, yy))

            # Store cost in dictionary
            self.costs[(xx, yy)] = cost_val

    def previous_target_callback(self, msg):
        """
        Update the previous target if needed.
        """
        self.previous_target = (msg.data[0], msg.data[1])

    def next_target_callback(self, msg):
        """
        Update the next target. If new, re-initialize path.
        """
        if len(msg.data) == 3:
            self.target_yaw = msg.data[2]

        new_target = (msg.data[0], msg.data[1])
        if self.next_target != new_target:
            self.next_target = new_target
            self.current_path = deque()
            self.current_path.append(self.next_target)
            self.next_waypoint = self.next_target
            self.get_logger().info(f"New next target: {self.next_target}")

    def update_pose(self, msg):
        """
        Update the robot's current position and orientation;
        check if we've reached the current waypoint.
        """
        self.robot_position = (msg.twist.linear.x, msg.twist.linear.y)
        self.yaw = msg.twist.angular.z

        if self.next_waypoint is not None:
            dx = self.next_waypoint[0] - self.robot_position[0]
            dy = self.next_waypoint[1] - self.robot_position[1]
            distance = math.sqrt(dx**2 + dy**2)
            if distance < self.waypoint_threshold:
                self.previous_points.append(self.current_path.popleft())
                self.invalidation_count = 0
                if len(self.current_path) == 0:
                    self.next_waypoint = None
                else:
                    self.next_waypoint = self.current_path[0]

    # -------------------------------------------------------------------------
    # Path Checking and Publishing
    # -------------------------------------------------------------------------
    def validate_path(self):
        """
        Periodically validate the path segment from the robot's position
        to the next waypoint. If invalid, re-plan.
        """
        if self.next_waypoint is None:
            return

        max_cell, total = self.compute_segment_cost(self.robot_position, self.next_waypoint)
        self.get_logger().info(f"Total segment cost: {total}\nmax cell cost: {max_cell}")
        if max_cell > self.max_cell_threshold:
            self.invalidation_count += 1
        else:
            # Segment is valid
            self.invalidation_count = max(0, self.invalidation_count - 1)

        if self.invalidation_count >= 3:
            self.compute_path()
            self.smooth_path()

        self.publish_waypoint()

    def publish_waypoint(self):
        """
        Publish the next waypoint plus a flag for using Stanley vs. pure pursuit.
        """
        if self.next_waypoint is None:
            return
        
        # Simple "dense" path heuristic
        self.use_stanley = self.is_path_segment_dense()

        waypoint_msg = Float32MultiArray()
        use_stanley_flag = 1.0 if self.use_stanley else 0.0
        waypoint_msg.data = [
            float(self.next_waypoint[0]),
            float(self.next_waypoint[1]),
            use_stanley_flag
        ]
        self.next_waypoint_publisher.publish(waypoint_msg)

    def is_path_segment_dense(self):
        """
        Example heuristic: if at least 3 consecutive points are within 1m, label it "dense".
        """
        path_list = list(self.current_path)
        if len(path_list) < 3:
            return False
        
        distance_threshold = 1.0
        for i in range(len(path_list) - 2):
            p1 = path_list[i]
            p2 = path_list[i + 1]
            p3 = path_list[i + 2]
            d12 = math.dist(p1, p2)
            d23 = math.dist(p2, p3)
            if d12 < distance_threshold and d23 < distance_threshold:
                return True
        return False

    # -------------------------------------------------------------------------
    # Cost-Based Segment Check
    # -------------------------------------------------------------------------
    def get_neighbor_costs(self, rx, ry, weight=1, n=1):
        """
        Get cost of neighboring diagonals
        n - int for grid cell expansion in each direction
        """
        cost = 0
        for x in range(-n, n, 1):
            for y in range(-n, n, 1):
                if x == 0 and y == 0:
                    continue
                x1 = rx + (x*0.25)
                y1 = ry + (y*0.25)
                d = math.sqrt(((x*0.25)**2) + ((y*0.25)**2))
                c = self.costs.get((x1, y1), 0.0) * (weight/d)
                cost += c
        return cost
    
    def compute_segment_cost(self, start, end, gap=1):
        """
        Sample points along the line segment from `start` to `end` and compute
        an average cost per meter. If the average cost per meter exceeds
        `self.threshold_cost_per_meter`, return None (invalid).
        
        Otherwise, return the average cost (float).
        """
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.sqrt(dx**2 + dy**2)

        # If there's effectively no length, treat as zero cost
        if distance < 1e-6:
            return 0.0

        # Number of sample points ~ distance in meters (or scale as needed)
        n = max(1, int(round(distance / self.cell_size * gap)))
        total_cost = 0.0
        max_cost = 0.0
        for i in range(n + 1):
            t = i / n
            sx = start[0] + t * dx
            sy = start[1] + t * dy
            
            # Snap to nearest cost cell in self.costs
            # Round to nearest 0.25 for the dictionary key if needed
            rx = round(sx * 4) / 4.0
            ry = round(sy * 4) / 4.0

            # Retrieve cost
            c = self.costs.get((rx, ry), 0.0)
            c2 = self.get_neighbor_costs(rx, ry, 1, 3)
            total_cost += c
            total_cost += c2
            max_cost = max(max_cost, c+c2)

        avg_cost = total_cost / (n + 1)
        #self.get_logger().info(f"Total cost: {total_cost}  Average cost: {avg_cost}")
        self.get_logger().info(f"max cell cost on path: {max_cost}")
        # Compute and return "average cost per meter"
        avg_cost_per_meter = avg_cost / distance
        return max_cost, total_cost

    # -------------------------------------------------------------------------
    # A* Path Planning (with length + cost)
    # -------------------------------------------------------------------------
    def compute_path(self, gap=2):
        """
        Run an A* search within a local ~50x50 region around the robot. 
        Each neighbor transition cost is a weighted combination of 
        (physical distance) and (average cost of segment).

        If the average cost of a segment > threshold, we skip it entirely.
        """
        if self.next_target is None:
            return

        start = self.robot_position
        goal = tuple(self.next_target)

        # Define local search region
        half_width = 50.0
        step_size = 0.25
        min_x = start[0] - half_width
        max_x = start[0] + half_width
        min_y = start[1] - half_width
        max_y = start[1] + half_width

        # Build discrete grids in x, y
        x_vals = np.arange(min_x, max_x + step_size, step_size)
        y_vals = np.arange(min_y, max_y + step_size, step_size)

        def in_bounds(xc, yc):
            return (min_x <= xc <= max_x) and (min_y <= yc <= max_y)

        def get_index(xc, yc):
            i = int(round((xc - min_x) / step_size))
            j = int(round((yc - min_y) / step_size))
            return (i, j)

        def get_coords(i, j):
            xc = min_x + i * step_size
            yc = min_y + j * step_size
            return (xc, yc)

        # Clamp goal
        gx = max(min(goal[0], max_x), min_x)
        gy = max(min(goal[1], max_y), min_y)
        goal_clamped = (gx, gy)

        start_idx = get_index(start[0], start[1])
        goal_idx = get_index(goal_clamped[0], goal_clamped[1])

        # Basic bound checks
        if not (0 <= start_idx[0] < len(x_vals) and 0 <= start_idx[1] < len(y_vals)):
            self.get_logger().warn("Start out of local region. Cannot plan.")
            return
        if not (0 <= goal_idx[0] < len(x_vals) and 0 <= goal_idx[1] < len(y_vals)):
            self.get_logger().warn("Goal out of local region. Cannot plan.")
            return

        # A* structures
        open_set = []
        heapq.heappush(open_set, (0.0, start_idx))  # (f_score, grid_index)
        came_from = {}
        g_score = {start_idx: 0.0}

        # 8-direction neighbors
        neighbors_deltas = [
            (1, 0), (1, 1), (0, 1), (-1, 1),
            (-1, 0), (-1, -1), (0, -1), (1, -1)
        ]

        def heuristic(a, b):
            """
            Combine distance and (optionally) cost for an admissible or near-admissible heuristic.
            For simplicity, we just do distance-based. You can add cost weighting as well.
            """
            ax, ay = get_coords(a[0], a[1])
            bx, by = get_coords(b[0], b[1])
            dist = math.dist((ax, ay), (bx, by))
            cost = self.get_neighbor_costs(ax, ay, weight=1, n=3)
            if cost > 1.0:
                self.get_logger().info(f"heuristic for point ({ax},{ay})  cost: {cost}")
            h = (dist * self.distance_weight) - (cost * self.cost_weight)
            return h

        found_path = False
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_idx:
                found_path = True
                break

            cur_g = g_score.get(current, float('inf'))
            cur_coords = get_coords(current[0], current[1])

            for dn in neighbors_deltas:
                ni = current[0] + dn[0]
                nj = current[1] + dn[1]
                neighbor = (ni, nj)
                if not (0 <= ni < len(x_vals) and 0 <= nj < len(y_vals)):
                    continue

                nbr_coords = get_coords(ni, nj)
                seg_cost, _ = self.compute_segment_cost(cur_coords, nbr_coords, gap=gap)
                if seg_cost is None:
                    # segment invalid
                    continue

                # step_distance is the Euclidean distance in XY
                step_distance = math.dist(cur_coords, nbr_coords)

                # Weighted cost of traveling from current -> neighbor
                travel_cost = (self.distance_weight * step_distance) \
                              + (self.cost_weight * seg_cost)

                tentative_g = cur_g + travel_cost
                old_g = g_score.get(neighbor, float('inf'))
                if tentative_g < old_g:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal_idx)
                    heapq.heappush(open_set, (f_score, neighbor))

        if not found_path:
            if gap > 1:
                self.get_logger().info("No path found. Trying reduced gap for cost sampling.")
                self.compute_path(gap=gap - 1)
            else:
                self.get_logger().error("A* planning failed to find any solution.")
            return

        # Reconstruct path
        path_indices = []
        cur = goal_idx
        while cur in came_from:
            path_indices.append(cur)
            cur = came_from[cur]
        path_indices.append(start_idx)
        path_indices.reverse()

        planned_path = []
        for (pi, pj) in path_indices:
            planned_path.append(get_coords(pi, pj))

        self.current_path.clear()
        # Typically we want to skip the first index if it is essentially at the start
        # because it's basically the robot's current position. But either approach is fine.
        if len(planned_path) < 2:
            return

        for wp in planned_path:
            self.current_path.append(wp)
        # pop off the first if it's basically the current position:
        self.current_path.popleft()

        # set next waypoint to first in the new path
        if len(self.current_path) > 0:
            self.next_waypoint = self.current_path[0]

    def smooth_path(self):
        """
        Simple collinearity smoothing: remove unnecessary vertices in a path.
        """
        if len(self.current_path) <= 2:
            return
        
        smoothed_path = deque()
        path_list = list(self.current_path)

        # Keep the very first point
        smoothed_path.append(path_list[0])

        for i in range(1, len(path_list) - 1):
            prev_point = path_list[i - 1]
            current_point = path_list[i]
            next_point = path_list[i + 1]

            v1 = (current_point[0] - prev_point[0], current_point[1] - prev_point[1])
            v2 = (next_point[0] - current_point[0], next_point[1] - current_point[1])

            mag1 = math.hypot(*v1)
            mag2 = math.hypot(*v2)
            if mag1 < 1e-6 or mag2 < 1e-6:
                # extremely close, skip
                continue

            v1_normalized = (v1[0] / mag1, v1[1] / mag1)
            v2_normalized = (v2[0] / mag2, v2[1] / mag2)
            dot = v1_normalized[0]*v2_normalized[0] + v1_normalized[1]*v2_normalized[1]

            # If the dot product is close to ±1, the points are nearly collinear
            if abs(dot) < 0.999:
                smoothed_path.append(current_point)

        # Always keep the last point
        smoothed_path.append(path_list[-1])

        self.current_path = smoothed_path
        self.next_waypoint = self.current_path[0]

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
