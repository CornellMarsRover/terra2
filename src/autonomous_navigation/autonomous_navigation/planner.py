#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped

from cv_bridge import CvBridge
import numpy as np
import math
from collections import deque
import heapq  # For priority queue in A*

import rerun as rr

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('visualize', True) # Shows rerun visualization if true
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        if self.visualize:
            # Initialize Rerun visualization
            rr.init("ground_plane_grid", spawn=True)

        # Subscription to the next and previous target topics
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
        # Subscribe to the robot pose topic
        self.pose_subscription = self.create_subscription(
            TwistStamped,
            '/autonomy/pose/robot/global',
            self.update_pose,
            10
        )

        # Costmap subscription
        self.obstacle_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/costmap',
            self.new_obstacle_callback,
            10
        )

        # Publisher for the next waypoint
        self.next_waypoint_publisher = self.create_publisher(Float32MultiArray, '/autonomy/path/next_waypoint', 10)

        # Bridge for (possible) debug image usage
        self.bridge = CvBridge()

        # Robot position data
        self.robot_position = (0.0, 0.0)
        self.yaw = 0.0
        self.costmap_size = 40.0

        # Targets
        self.previous_target = (0.0, 0.0) # [north (meters), west (meters)]
        self.next_target = (0.0, 0.0)
        self.target_yaw = None  # radians

        # Store the cost of each cell that comes from costmap
        # Key: (x, y) => Value: cost
        self.obstacle_costs = {}

        # We'll treat any cell above this cost as "impassable" if you want a hard threshold.
        # Adjust as desired or set to something large if all cells are passable but expensive.
        self.lethal_cost_threshold = 60.0

        # For discretizing certain path checks
        self.cell_size = 0.25
        self.k = 4

        # Path management
        self.current_path = deque()
        self.previous_points = deque()
        self.next_waypoint = None
        self.waypoint_threshold = 0.6

        # Check the current path at 2 Hz
        self.path_check_timer = self.create_timer(0.5, self.validate_path)

        # Timer to redraw visualization
        if self.visualize:
            self.redraw_timer = self.create_timer(0.2, self.plot_grid_rerun)

        self.get_logger().info("PlannerNode initialized and subscribed to costmap and target pose topics.")

    def new_obstacle_callback(self, msg):
        """
        Costmap callback: we store cost for each cell (x, y).
        If the cost is extremely high, you may treat it as lethal.
        """
        if len(msg.data) == 0:
            return
        for i in range(0, len(msg.data), 3):
            x_cell = msg.data[i]
            y_cell = msg.data[i+1]
            cost_val = msg.data[i+2]
            # Store the cost (so we can use it in A*)
            self.obstacle_costs[(x_cell, y_cell)] = cost_val

    def validate_path(self):
        """
        Validate the current path, running on a timer.
        If there's a next_waypoint, check the segment from the robot position to it.
        If it appears invalid (obstacle or super high cost?), re-plan.
        """
        if self.next_waypoint is None:
            return

        # Optional: Do a quick collision check for the direct segment
        # If invalid, re-plan
        valid = self.check_path_segment(self.robot_position, self.next_waypoint, gap=3)
        if not valid:
            self.compute_path()
            self.smooth_path()

        self.publish_waypoint()

    def publish_waypoint(self):
        """
        Publish next waypoint in path
        """
        if self.next_waypoint is None:
            return
        waypoint_msg = Float32MultiArray()
        waypoint_msg.data = [float(self.next_waypoint[0]), float(self.next_waypoint[1])]
        self.next_waypoint_publisher.publish(waypoint_msg)

    def previous_target_callback(self, msg):
        """
        Callback function to update the previous target
        """
        self.previous_target = (msg.data[0], msg.data[1])

    def next_target_callback(self, msg):
        """
        Callback function to update the target pose.
        """
        if len(msg.data) == 3:
            self.target_yaw = msg.data[2]

        if self.next_target != (msg.data[0], msg.data[1]):
            self.next_target = (msg.data[0], msg.data[1])
            self.current_path = deque()
            self.current_path.append(self.next_target)
            self.next_waypoint = self.next_target
            self.get_logger().info(f"Next target: {self.next_target}")

    def update_pose(self, msg):
        """
        Update robot position and yaw.
        Then check if the robot is close to the current next path waypoint.
        """
        self.robot_position = (msg.twist.linear.x, msg.twist.linear.y)
        self.yaw = msg.twist.angular.z
        if self.next_waypoint is not None:
            dx = self.next_waypoint[0] - self.robot_position[0]
            dy = self.next_waypoint[1] - self.robot_position[1]
            distance = math.sqrt(dx**2 + dy**2)
            if distance < self.waypoint_threshold:
                self.get_logger().info("Reached next path waypoint")
                if len(self.current_path) > 0:
                    self.previous_points.append(self.current_path.popleft())
                if len(self.current_path) == 0:
                    self.next_waypoint = None
                else:
                    self.next_waypoint = self.current_path[0]

    def check_path_segment(self, start, end, gap=2):
        """
        Checks if the segment from `start` to `end` passes through any lethal obstacles.
        If you want a pure cost-based approach, you can remove or relax this check.

        Returns:
            bool: True if it is free enough to traverse; False if blocked.
        """
        # Let’s do a small sampling of points along the line
        # and see if they exceed a 'lethal' threshold or if the dictionary has them as obstacles.
        distance = math.dist(start, end)
        if distance < 1e-3:
            return True

        n_samples = max(1, int(distance / 0.25))  # sample every 0.25 m, for example
        for i in range(n_samples+1):
            t = i / n_samples
            sx = start[0] + t*(end[0] - start[0])
            sy = start[1] + t*(end[1] - start[1])
            # Round to nearest cell
            cell = (round(sx*self.k)/self.k, round(sy*self.k)/self.k)

            cost_here = self.obstacle_costs.get(cell, 0.0)
            if cost_here >= self.lethal_cost_threshold:
                return False

        return True

    def get_cell_cost(self, x, y):
        """
        Return the cost from the costmap dictionary, defaulting to 0.0 if unknown.
        """
        # Round or snap to whichever resolution you're storing in the dictionary
        rx = round(x * self.k)/self.k
        ry = round(y * self.k)/self.k
        return self.obstacle_costs.get((rx, ry), 0.0)

    def compute_path(self, gap=4):
        """
        A* path planning within a local region, factoring in obstacle costs.

        - The cost of traveling from a cell to its neighbor is:
              step_distance + alpha * obstacle_cost_of_neighbor
          (where `alpha` is a weighting factor you can tune).

        - If a neighbor's cost is >= lethal threshold, we skip it entirely.
        """
        if self.next_target is None:
            return

        start = self.robot_position
        goal = tuple(self.next_target)

        # Local search region
        half_width = 20.0
        step_size = 0.5
        min_x = start[0] - half_width
        max_x = start[0] + half_width
        min_y = start[1] - half_width
        max_y = start[1] + half_width

        # Grid arrays
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

        # Clamp the goal
        gx = max(min(goal[0], max_x), min_x)
        gy = max(min(goal[1], max_y), min_y)
        goal_clamped = (gx, gy)

        start_idx = get_index(start[0], start[1])
        goal_idx = get_index(goal_clamped[0], goal_clamped[1])

        # Basic boundary checks
        if not (0 <= start_idx[0] < len(x_vals) and 0 <= start_idx[1] < len(y_vals)):
            self.get_logger().warn("Start is out of local grid. Cannot plan.")
            return
        if not (0 <= goal_idx[0] < len(x_vals) and 0 <= goal_idx[1] < len(y_vals)):
            self.get_logger().warn("Goal is out of local grid. Cannot plan.")
            return

        open_set = []
        heapq.heappush(open_set, (0.0, start_idx))  # (f_score, node)
        came_from = {}
        g_score = {start_idx: 0.0}

        # 8-connected neighbors
        neighbors_8 = [(1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]

        # Tuning factor for how heavily to weight the obstacle cost.
        alpha = 0.05

        def heuristic(a, b):
            # Euclidean distance to goal
            ax, ay = get_coords(a[0], a[1])
            bx, by = get_coords(b[0], b[1])
            return math.dist((ax, ay), (bx, by))

        found_path = False

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_idx:
                found_path = True
                break

            current_g = g_score[current]
            for dn in neighbors_8:
                ni = current[0] + dn[0]
                nj = current[1] + dn[1]
                neighbor = (ni, nj)
                if not (0 <= ni < len(x_vals) and 0 <= nj < len(y_vals)):
                    continue

                cur_coords = get_coords(current[0], current[1])
                nbr_coords = get_coords(ni, nj)

                # If purely cost-based skipping is desired, do so if cost is lethal:
                neighbor_cost = self.get_cell_cost(nbr_coords[0], nbr_coords[1])
                if neighbor_cost >= self.lethal_cost_threshold:
                    # treat as impassable
                    continue

                # Movement cost is the step distance plus alpha*cost at the neighbor
                step_distance = math.dist(cur_coords, nbr_coords)
                step_cost = step_distance + alpha * neighbor_cost

                tentative_g = current_g + step_cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal_idx)
                    heapq.heappush(open_set, (f_score, neighbor))

        if not found_path:
            if gap > 2:
                self.get_logger().info("No path found with A*. Reducing gap and re-trying.")
                self.compute_path(gap=gap-1)
            else:
                self.get_logger().warn("No path found; giving up.")
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
            px, py = get_coords(pi, pj)
            planned_path.append((px, py))

        self.current_path.clear()
        if len(planned_path) <= 1:
            return

        for wp in planned_path:
            self.current_path.append(wp)

        # The very first point is near the robot; skip it to get an actual "next" waypoint
        if len(self.current_path) > 1:
            self.current_path.popleft()

        self.get_logger().info(f"A* path found: {list(self.current_path)}")
        # Set next waypoint
        if self.current_path:
            self.next_waypoint = self.current_path[0]

    def smooth_path(self):
        """
        Smooth the current path by removing intermediate waypoints that lie on nearly straight lines.
        """
        if len(self.current_path) <= 2:
            return

        smoothed_path = deque()
        smoothed_path.append(self.current_path[0])  # Always keep the first point

        for i in range(1, len(self.current_path) - 1):
            prev_point = self.current_path[i - 1]
            current_point = self.current_path[i]
            next_point = self.current_path[i + 1]

            # Calculate vectors
            v1 = (current_point[0] - prev_point[0], current_point[1] - prev_point[1])
            v2 = (next_point[0] - current_point[0], next_point[1] - current_point[1])
            v1_mag = math.dist(prev_point, current_point)
            v2_mag = math.dist(current_point, next_point)

            # If they are too tiny, skip
            if v1_mag < 1e-6 or v2_mag < 1e-6:
                continue

            v1_norm = (v1[0]/v1_mag, v1[1]/v1_mag)
            v2_norm = (v2[0]/v2_mag, v2[1]/v2_mag)
            dot_product = v1_norm[0]*v2_norm[0] + v1_norm[1]*v2_norm[1]

            # If the angle is not ~straight, we keep the corner
            if abs(dot_product) < 0.999:
                smoothed_path.append(current_point)

        smoothed_path.append(self.current_path[-1])  # Keep the last point
        self.current_path = smoothed_path
        self.next_waypoint = self.current_path[0] if self.current_path else None
        self.get_logger().info(f"Smoothed path: {list(self.current_path)}")

    def plot_grid_rerun(self):
        """
        Visualize local state and path in Rerun.
        """
        points = []
        colors = []
        radii = []

        # Plot obstacle points with color depending on cost
        for (cell, cost_val) in self.obstacle_costs.items():
            (ox, oy) = cell
            # Plot in robot-relative coordinates for convenience
            rx = ox - self.robot_position[0]
            ry = oy - self.robot_position[1]

            # Color: from green (low cost) to red (high cost)
            cost_clamped = min(cost_val, 100.0)
            red_intensity = int(255 * (cost_clamped / 100.0))
            green_intensity = 255 - red_intensity
            b = 0

            points.append([rx, ry, 0.0])
            colors.append([red_intensity, b, b])
            radii.append(0.1)

        # Robot
        points.append([0, 0, 0])
        colors.append([0, 0, 255])
        radii.append(0.25)

        # Path
        for (x, y) in self.current_path:
            rx = x - self.robot_position[0]
            ry = y - self.robot_position[1]
            points.append([rx, ry, 0])
            colors.append([0, 255, 0])
            radii.append(0.1)

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
        x_vec = reverseR.dot(np.array([2.0, 0.0]))
        x_end = [x_vec[0], x_vec[1], 0]

        # Draw the orientation
        rr.log("robot_direction", rr.LineStrips3D(
            np.array([[0, 0, 0], x_end]),
            radii=0.1,
            colors=[255,0,255]
        ))

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
