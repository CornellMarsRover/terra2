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

        self.declare_parameter('visualize', True) # Shows rerun visualization if true
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        self.visualize = True
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
        # Subscribe to the robot pose topicpoints
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
        # Publisher for the next waypoint for waypoint following in controller node
        self.next_waypoint_publisher = self.create_publisher(Float32MultiArray, '/autonomy/path/next_waypoint', 10)

        # Initialize CvBridge for message conversion
        self.bridge = CvBridge()

        # Store current robot position
        self.robot_position = (0.0, 0.0)
        self.yaw = 0.0
        self.costmap_size = 40.0

        # Store next and previous mission targets
        self.previous_target = (0.0, 0.0) # [north (meters), west (meters)]
        self.next_target = (0,0, 0.0)
        self.target_yaw = None  # radians

        # Cache of detected obstacles
        self.obstacles = set()
        self.max_cost = 100 # Maximum cost of a cell (defined in costmap node)
        self.threshold = 4 # minimum cell cost to be classified as obstacle
        self.cell_size = 0.25
        self.k = 4
        # Current path (sequence of waypoints)
        self.current_path = deque()
        self.previous_points = deque()
        self.next_waypoint = None
        self.waypoint_threshold = 0.6
        
        # Check the current path at a frequency of 2 Hz
        self.path_check_timer = self.create_timer(0.5, self.validate_path)
        self.invalidation_count = 0 # counter to be tolerant to faulty path invalidations
        self.costs = dict()
        self.use_stanley = False  # We'll set this dynamically

        self.get_logger().info("PlannerNode initialized and subscribed to costmap and target pose topics.")
        self.ground_plane = []

        if self.visualize:
            # Initialize Rerun visualization
            rr.init("ground_plane_grid", spawn=True)
            self.redraw_timer = self.create_timer(0.2, self.plot_grid_rerun)

    def plot_grid_rerun(self):
        """
        Visualize the local state and path
        """
        points = []
        colors = []
        radii = []
        
        # Obstacles
        for (x,y) in self.obstacles:
            points.append([x-self.robot_position[0], y-self.robot_position[1], 0])
            colors.append([255,0,0])
            radii.append(0.1)

        # Robot position
        points.append([0, 0, 0])
        colors.append([0, 0, 255])
        radii.append(0.25)

        # Path
        for (x,y) in self.current_path:
            points.append([x-self.robot_position[0], y-self.robot_position[1], 0])
            colors.append([0,255,0])
            radii.append(0.1)
        if len(self.ground_plane) > 1:
            ground_pts = []
            for pt in self.ground_plane:
                ground_pts.append(pt)
            ground_pts.append(self.ground_plane[0])
            rr.log("ground_plane", rr.LineStrips3D(
                np.array(ground_pts),
                radii=0.05,
                colors=[255,255,255]
            ))
        rr.log("path_planning_visualization", rr.Points3D(np.array(points, dtype=np.float32), colors=colors, radii=radii))

        # Robot orientation
        reverseR = np.array([
            [np.cos(self.yaw), -np.sin(self.yaw)],
            [np.sin(self.yaw),  np.cos(self.yaw)]
        ])
        x_vec = reverseR.dot(np.array([2.0, 0]))
        x_end = [x_vec[0], x_vec[1], 0]

        # Draw the orientation
        rr.log("robot_direction", rr.LineStrips3D(
            np.array([[0, 0, 0], x_end]),
            radii=0.1,
            colors=[255,0,255]
        ))

    def ground_plane_callback(self, msg):
        self.ground_plane = []
        R = np.array([
            [np.cos(self.yaw), -np.sin(self.yaw)],
            [np.sin(self.yaw),  np.cos(self.yaw)]
        ])
        #R = np.array([[1.0, 0.0], [0.0, 1.0]])
        x = msg.x
        y = msg.y
        for i in range(len(x)):
            pt = R.dot(np.array([x[i], y[i]]))
            self.ground_plane.append([pt[0]-self.robot_position[0], pt[1]-self.robot_position[1], 0])

    def new_obstacle_callback(self, msg):
        """
        Costmap callback
        """
        if len(msg.data) == 0:
            return
        for i in range(0, len(msg.data), 3):
            if (msg.data[i+2]) > self.threshold:
                self.obstacles.add((msg.data[i], msg.data[i+1]))
            elif (msg.data[i], msg.data[i+1]) in self.obstacles:
                self.obstacles.discard((msg.data[i], msg.data[i+1]))
            self.costs[(msg.data[i], msg.data[i+1])] = msg.data[i+2]

    def validate_path(self):
        """
        Validate the current path, running on a timer.
        If there's a next_waypoint, check the segment from the robot position to it.
        """
        if self.next_waypoint is None:
            return
        valid = self.check_path_segment(self.robot_position, self.next_waypoint, gap=3)
        if not valid:
            self.invalidation_count += 1
        else:
            self.invalidation_count = max(0, self.invalidation_count-1)

        if self.invalidation_count >= 3:
            #self.get_logger().info("Replanning path")
            self.compute_path()
            self.smooth_path()

        self.publish_waypoint()

    def publish_waypoint(self):
        """
        Publish next waypoint in path with an added flag for using Stanley.
        """
        if self.next_waypoint is None:
            return
        
        # Decide whether this portion of the path is "dense" based on distance 
        # between consecutive waypoints or the total number of waypoints 
        # clustered in a small area.  Customize as desired.
        self.use_stanley = self.is_path_segment_dense()

        waypoint_msg = Float32MultiArray()

        # Add third element: 1.0 if we want to use Stanley, else 0.0
        use_stanley_flag = 1.0 if self.use_stanley else 0.0
        waypoint_msg.data = [
            float(self.next_waypoint[0]),
            float(self.next_waypoint[1]),
            use_stanley_flag
        ]
        self.next_waypoint_publisher.publish(waypoint_msg)

    def is_path_segment_dense(self):
        """
        Returns True if the path around the current waypoint is "dense."
        Simple heuristic example: if at least 3 consecutive waypoints 
        are within 1.0m of each other. Adjust to your liking.
        """
        path_list = list(self.current_path)

        if len(path_list) < 3:
            return False

        # Look at next 3 (or more) consecutive waypoints
        # If they are all within e.g. 1.0 meter, call it dense.
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

        # If new target received, set it to next target
        if self.next_target != (msg.data[0], msg.data[1]):
            self.next_target = (msg.data[0], msg.data[1])
            self.current_path = deque()
            self.current_path.append(self.next_target)
            self.next_waypoint = self.next_target
            self.get_logger().info(f"Next target: {self.next_target}")

    def update_pose(self, msg):
        """
        Callback function to update the robot position and yaw
        Checks if the robot is within proximity of next path waypoint
        """
        self.robot_position = (msg.twist.linear.x, msg.twist.linear.y)
        self.yaw = msg.twist.angular.z
        if self.next_waypoint is not None:
            dx = self.next_waypoint[0] - self.robot_position[0]
            dy = self.next_waypoint[1] - self.robot_position[1]
            distance = math.sqrt(dx**2 + dy**2)
            #self.get_logger().info(f"Distance to path waypoint: {distance}")
            if distance < self.waypoint_threshold:
                #self.get_logger().info("Reached next path waypoint")
                self.previous_points.append(self.current_path.popleft())
                self.invalidation_count = 0
                if len(self.current_path) == 0:
                    self.next_waypoint = None
                else:
                    self.next_waypoint = self.current_path[0]

    def check_path_segment(self, start, end, gap=1):
        """
        Checks if segment to next waypoint does not run through any obstacles.

        Args:
        - start (tuple): Starting point (x, y).
        - end (tuple): Ending point (x, y).

        Returns:
        - bool: True if all sampled points along the line pass the checks, False otherwise.
        """
        point_set = self.obstacles
        max_length = self.costmap_size / 2

        # Compute the distance between start and end
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        distance = math.sqrt(dx**2 + dy**2)

        # Adjust the end point if the segment exceeds the max length
        if distance > max_length:
            scale = max_length / distance
            end = (start[0] + dx * scale, start[1] + dy * scale)

        # Sample along the line from start to end
        n = max(1, int(round(distance)))  # number of sample points
        sampled_points = []
        for i in range(n + 1):
            t = i / n
            sampled_x = start[0] + t * (end[0] - start[0])
            sampled_y = start[1] + t * (end[1] - start[1])
            # Round each sample to the nearest 1 meter for checking neighbors
            rounded_x, rounded_y = round(sampled_x*self.k)/self.k, round(sampled_y*self.k)/self.k
            sampled_points.append((rounded_x, rounded_y))

        # Check each sampled point's neighborhood at 0.25m increments
        # Check a smaller range if validating
        d = [0]
        for i in range(1,gap+1):
            d.append(-1.0 * i * self.cell_size)
            d.append(i * self.cell_size)

        for (sx, sy) in sampled_points:
            neighbors = [
                (sx + dx_, sy + dy_)
                for dx_ in d
                for dy_ in d
            ]
            for nbr in neighbors:
                if nbr in point_set:
                    return False  # Obstacle found

        return True  # No obstacle found in the line segment
    
    def compute_path(self, gap=2):
        """
        Implement an A* planning algorithm to plan a path within a 50x50m
        local region (±25.0m from the robot's current position in both x and y).
        The map is discretized at 0.5m steps, and each step's validity is checked
        with check_path_segment().

        This version includes a *hierarchy* in the order neighbors are explored:
          1) (1, 0)
          2) (1, 1), (1, 1)  [as provided]
          3) (1, 0), (-1, 0)
          4) (1, -1), (-1, -1)
          5) (0, 1)
        """
        if self.next_target is None:
            return

        start = self.robot_position
        goal = tuple(self.next_target)

        # Define local search region around the robot
        half_width = 20.0
        step_size = 0.25
        min_x = start[0] - half_width
        max_x = start[0] + half_width
        min_y = start[1] - half_width
        max_y = start[1] + half_width

        # Create discrete grids in x and y
        x_vals = np.arange(min_x, max_x + step_size, step_size)
        y_vals = np.arange(min_y, max_y + step_size, step_size)

        # Helper functions to map (x, y) <-> grid indices (i, j)
        def in_bounds(xc, yc):
            return (min_x <= xc <= max_x) and (min_y <= yc <= max_y)

        def get_index(xc, yc):
            # Round to nearest discrete cell index
            i = int(round((xc - min_x) / step_size))
            j = int(round((yc - min_y) / step_size))
            return (i, j)

        def get_coords(i, j):
            # Return the center (x, y) of the cell
            xc = min_x + i * step_size
            yc = min_y + j * step_size
            return (xc, yc)

        # Clamp the goal within our local grid if it's outside
        gx = max(min(goal[0], max_x), min_x)
        gy = max(min(goal[1], max_y), min_y)
        goal_clamped = (gx, gy)

        # Convert start/goal to grid coordinates
        start_idx = get_index(start[0], start[1])
        goal_idx = get_index(goal_clamped[0], goal_clamped[1])

        # Basic check in case start or goal is out of bounds of the discrete array
        if not (0 <= start_idx[0] < len(x_vals) and 0 <= start_idx[1] < len(y_vals)):
            self.get_logger().warn("Start position is out of local 50x50 grid. Cannot plan.")
            return
        if not (0 <= goal_idx[0] < len(x_vals) and 0 <= goal_idx[1] < len(y_vals)):
            self.get_logger().warn("Goal position is out of local 50x50 grid. Clamping did not help. Cannot plan.")
            return

        # A* setup
        open_set = []
        heapq.heappush(open_set, (0.0, start_idx))  # (f_cost, (i, j))
        came_from = dict()  # (i, j) -> (parent_i, parent_j)
        g_score = {start_idx: 0.0}

        # Use the requested neighbor expansion hierarchy
        neighbors_hierarchy = [
            (1, 0),            # 1
            (1, 1), (1, 1),    # 2
            (1, 0), (-1, 0),   # 3
            (1, -1), (-1, -1), # 4
            (0, 1)             # 5
        ]

        def heuristic(a, b):
            # Euclidean distance heuristic
            ax, ay = get_coords(a[0], a[1])
            bx, by = get_coords(b[0], b[1])
            return math.sqrt((ax - bx)**2 + (ay - by)**2)

        # A* search
        found_path = False
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_idx:
                found_path = True
                break

            for dn in neighbors_hierarchy:
                ni = current[0] + dn[0]
                nj = current[1] + dn[1]
                neighbor = (ni, nj)

                # Check bounds
                if not (0 <= ni < len(x_vals) and 0 <= nj < len(y_vals)):
                    continue

                # Check if path segment is valid using check_path_segment
                cur_coords = get_coords(current[0], current[1])
                nbr_coords = get_coords(ni, nj)
                if not self.check_path_segment(cur_coords, nbr_coords, gap=gap):
                    continue

                # Cost from current to neighbor (cardinal vs diagonal)
                step_cost = math.sqrt(dn[0]**2 + dn[1]**2)
                tentative_g = g_score.get(current, float('inf')) + step_cost

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal_idx)
                    heapq.heappush(open_set, (f_score, neighbor))

        if not found_path:
            if gap > 1:
                self.get_logger().info("No path found with A* in the local 50x50 region.\nReducing obstacle padding")
                self.compute_path(gap=gap-1)
            return

        # Reconstruct path
        path_indices = []
        cur = goal_idx
        while cur in came_from:
            path_indices.append(cur)
            cur = came_from[cur]
        path_indices.append(start_idx)
        path_indices.reverse()

        # Convert path indices back to real-world coordinates
        planned_path = []
        for pi, pj in path_indices:
            px, py = get_coords(pi, pj)
            planned_path.append((px, py))

        # Store path in self.current_path
        self.current_path.clear()
        if len(planned_path) <= 1:
            return
        for wp in planned_path:
            self.current_path.append(wp)

        # Set the next waypoint to the first step beyond the robot's current position (if it exists)
        if len(self.current_path) > 1:
            # The first in the list is effectively the current position, so we skip to the next
            self.current_path.popleft()

    def smooth_path(self):
        """
        Smooth the current path by removing intermediate waypoints that are part of a straight line segment.
        This keeps the first and last points and removes points that represent unnecessary corners.

        Modifies:
        - self.current_path: Updates with the smoothed version of the path.
        """
        if len(self.current_path) <= 2:
            # Path is already smooth if it has two or fewer points
            return

        smoothed_path = deque()

        for i in range(1, len(self.current_path) - 1):
            # Get three consecutive points: previous, current, and next
            prev_point = self.current_path[i - 1]
            current_point = self.current_path[i]
            next_point = self.current_path[i + 1]

            # Calculate vectors
            v1 = (current_point[0] - prev_point[0], current_point[1] - prev_point[1])
            v2 = (next_point[0] - current_point[0], next_point[1] - current_point[1])

            # Normalize vectors
            v1_magnitude = math.sqrt(v1[0]**2 + v1[1]**2)
            v2_magnitude = math.sqrt(v2[0]**2 + v2[1]**2)
            if v1_magnitude < 0.01 or v2_magnitude < 0.01:
                continue
            v1_normalized = (v1[0] / v1_magnitude, v1[1] / v1_magnitude)
            v2_normalized = (v2[0] / v2_magnitude, v2[1] / v2_magnitude)

            # Check if vectors are aligned (dot product close to 1 or -1)
            dot_product = v1_normalized[0] * v2_normalized[0] + v1_normalized[1] * v2_normalized[1]

            if abs(dot_product) < 0.999:
                # Not a straight line: keep this point as it represents a corner
                smoothed_path.append(current_point)

        smoothed_path.append(self.current_path[-1])  # Always keep the last point
        smoothed_path.append(self.next_target)
        self.current_path = smoothed_path
        self.next_waypoint = self.current_path[0]
        #self.get_logger().info(f"Smoothed path: {list(self.current_path)}")


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
