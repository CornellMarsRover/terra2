#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

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

        # Create a subscription to the costmap and costmap position topic
        self.costmap_subscription = self.create_subscription(
            Image,
            '/autonomy/costmap',
            self.costmap_callback,
            10
        )
        self.costmap_pose_subscription = self.create_subscription(
            Image,
            '/autonomy/pose/costmap',
            self.costmap_centerpoint_callback,
            10
        )

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
            Float32MultiArray,
            '/autonomy/pose/robot/global',
            self.update_pose,
            10
        )

        # Publisher for the next waypoint for waypoint following in controller node
        self.next_waypoint_publisher = self.create_publisher(Float32MultiArray, '/autonomy/path/next_waypoint', 10)

        # Initialize CvBridge for message conversion
        self.bridge = CvBridge()

        # Store current robot position
        self.robot_position = (0.0, 0.0)
        self.yaw = 0.0

        # Store next and previous mission targets
        self.previous_target = (0.0, 0.0) # [north (meters), west (meters)]
        self.next_target = (0.0, 0.0)
        self.target_yaw = None  # radians

        # Current costmap
        self.costmap = None
        self.costmap_center = [0.0, 0.0] # Centerpoint of current costmap (in meters)
        self.costmap_center_index = (50, 50) # Index of centerpoint in the costmap image
        self.costmap_size = 50.0 # Costmap sidelength in meters
        self.cell_size = 0.5 # Cell sidelength in meters

        # Cache of detected obstacles
        self.obstacles = set()
        self.threshold = 3 # minimum cell cost to be classified as obstacle

        # Current path (sequence of waypoints)
        self.current_path = deque()
        self.previous_points = deque()
        self.next_waypoint = None

        # Check the current path at a frequency of 2 Hz
        self.path_check_timer = self.create_timer(0.5, self.validate_path)

        # Timer to redraw visualization
        if self.visualize:
            self.redraw_timer = self.create_timer(0.2, self.plot_grid_rerun)

        self.get_logger().info("PlannerNode initialized and subscribed to costmap and target pose topics.")

    def validate_path(self):
        """
        Validate the current path, running on a timer.
        If there's a next_waypoint, check the segment from the robot position to it.
        """
        if self.next_waypoint is None:
            return
        valid = self.check_path_segment(self.robot_position, self.next_waypoint)
        if not valid:
            self.get_logger().info("Replanning path")
            self.compute_path()
        else:
            self.get_logger().info("Current path valid")

    def costmap_callback(self, msg):
        """
        Callback function for the costmap topic. Converts the Image message back to a numpy array.
        """
        try:
            # Convert the ROS Image message to a numpy array
            self.costmap = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self.store_obstacles_global()
        except Exception as e:
            self.get_logger().error(f"Failed to process costmap: {e}")

    def store_obstacles_global(self):
        """
        Helper function to store the global positions of obstacles
        from the current costmap, rounded to the nearest 0.5 meter
        """
        if self.costmap is None:
            return

        for x, row in enumerate(self.costmap):
            for y, cost in enumerate(row):
                if cost >= self.threshold:
                    x_coord = self.costmap_center[0] + ((x - self.costmap_center_index[0]) * self.cell_size)
                    y_coord = self.costmap_center[1] + ((y - self.costmap_center_index[1]) * self.cell_size)
                    # Round coords to nearest 0.5
                    x_coord = round(x_coord * 2) / 2
                    y_coord = round(y_coord * 2) / 2
                    self.obstacles.add((x_coord, y_coord))

    def costmap_centerpoint_callback(self, msg):
        """
        Callback function to update costmap centerpoint coordinate
        """
        self.costmap_center = (msg.data[0], msg.data[1])

    def previous_target_callback(self, msg):
        """
        Callback function to update the previous target
        """
        self.previous_target = (msg.data[0], msg.data[1])

    def next_target_callback(self, msg):
        """
        Callback function to update the target pose.
        """
        self.next_target = [msg.data[0], msg.data[1]]
        if len(msg.data) == 3:
            self.target_yaw = msg.data[2]

        # If there's no "active" next_waypoint, set it now
        if self.next_waypoint is None:
            self.next_waypoint = self.next_target
            self.get_logger().info(f"Next waypoint: {self.next_waypoint}")

    def update_pose(self, msg):
        """
        Callback function to update the robot position and yaw
        """
        self.robot_position = (msg.data[0], msg.data[1])
        self.yaw = msg.data[2]

    def check_path_segment(self, start, end):
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
            rounded_x, rounded_y = round(sampled_x), round(sampled_y)
            sampled_points.append((rounded_x, rounded_y))

        # Check each sampled point's neighborhood at 0.5m increments
        for (sx, sy) in sampled_points:
            neighbors = [
                (sx + dx_, sy + dy_)
                for dx_ in (-1, -0.5, 0, 0.5, 1)
                for dy_ in (-1, -0.5, 0, 0.5, 1)
            ]
            for nbr in neighbors:
                if nbr in point_set:
                    return False  # Obstacle found

        return True  # No obstacle found in the line segment
    
    def compute_path(self):
        """
        Implement an A* planning algorithm to plan a path within a 25x25m
        local region (±12.5m from the robot's current position in both x and y).
        The map is discretized at 0.5m steps, and each step's validity is checked
        with check_path_segment().
        """
        if self.next_target is None:
            return

        start = self.robot_position
        goal = tuple(self.next_target)

        # Define local search region around the robot
        half_width = 12.5
        step_size = 1.0
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
            self.get_logger().warn("Start position is out of local 25x25 grid. Cannot plan.")
            return
        if not (0 <= goal_idx[0] < len(x_vals) and 0 <= goal_idx[1] < len(y_vals)):
            self.get_logger().warn("Goal position is out of local 25x25 grid. Clamping did not help. Cannot plan.")
            return

        # A* setup
        open_set = []
        heapq.heappush(open_set, (0.0, start_idx))  # (f_cost, (i, j))
        came_from = dict()  # (i, j) -> (parent_i, parent_j)
        g_score = {start_idx: 0.0}

        # 8-connected movement offsets (up, down, left, right, diagonals)
        neighbors_8 = [
            (1, 0), (-1, 0), (0, 1), (0, -1),  # cardinals
            (1, 1), (1, -1), (-1, 1), (-1, -1) # diagonals
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

            for dn in neighbors_8:
                ni = current[0] + dn[0]
                nj = current[1] + dn[1]
                neighbor = (ni, nj)

                # Check bounds
                if not (0 <= ni < len(x_vals) and 0 <= nj < len(y_vals)):
                    continue

                # Check if path segment is valid using check_path_segment
                cur_coords = get_coords(current[0], current[1])
                nbr_coords = get_coords(ni, nj)
                if not self.check_path_segment(cur_coords, nbr_coords):
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
            self.get_logger().info("No path found with A* in the local 25x25 region.")
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
        for wp in planned_path:
            self.current_path.append(wp)

        # Set the next waypoint to the first step beyond the robot's current position (if it exists)
        if len(self.current_path) > 1:
            # The first in the list is effectively the current position, so we skip to the next
            self.next_waypoint = self.current_path[1]
        elif len(self.current_path) == 1:
            # The path only has one point (start==goal), so it's effectively done
            self.next_waypoint = self.current_path[0]
        

        self.get_logger().info(f"A* path computed. Path length: {len(self.current_path)} waypoints.")
        if self.next_waypoint:
            self.get_logger().info(f"Next waypoint after planning: {self.next_waypoint}")

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
        radii.append(0.5)

        # Path
        for (x,y) in self.current_path:
            points.append([x-self.robot_position[0], y-self.robot_position[1], 0])
            colors.append([0,255,0])
            radii.append(0.1)

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
