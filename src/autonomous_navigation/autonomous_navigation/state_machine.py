#!/usr/bin/env python3

"""
State Machine ROS2 Node
High-level logic for autonomy
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int16, Float32MultiArray, String
from geometry_msgs.msg import Twist, TwistStamped
from cmr_msgs.msg import Detection
import yaml
import math
import os
import time
import numpy as np
from collections import deque
from typing import Dict, Tuple, Optional, Deque
from ament_index_python.packages import get_package_share_directory
from pyubx2 import llh2ecef

# === URC object-detection subsystem (overview: cmr_cams/DETECTION_TESTING.md) ===
# YOLO mission objects detected on the base station (urc_objects_v9.pt). A
# waypoint whose target name is one of these triggers the detect-and-stop flow
# instead of the AR-marker approach flow. Key methods below: update_detection,
# detection_callback, publish_detection_state.
YOLO_CLASSES = {'mallet', 'bottle', 'ice_pick'}
NO_TARGET = 'none'

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # Declare parameters
        self.declare_parameter('real', True)  # FALSE IF TESTING IN SIM
        self.declare_parameter('waypoint_tolerance', 2.0)  # meters
        # GPS topic used purely for one-shot origin capture. Default matches the
        # real rover (RTK driver). Override to '/navsatfix' for sim.
        self.declare_parameter('gps_topic', '/rtk/navsatfix_data')
        # Distance to a YOLO object's GNSS coordinate at which we start sampling
        # base-station detection. Tune to GNSS accuracy: ~3 m for objects 1 & 2,
        # ~10 m for object 3. Used as a fallback when a waypoint's targets-map
        # entry does not specify its own search_radius.
        self.declare_parameter('detection_vicinity_radius', 5.0)
        # How long to hold the rover stopped after a confident detection (so the
        # operator/judges see the stopped state + overlay + banner) before
        # marking the target done and advancing.
        self.declare_parameter('detection_hold_seconds', 4.0)

        # Get parameters
        self.real = self.get_parameter('real').get_parameter_value().bool_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.gps_topic = self.get_parameter('gps_topic').get_parameter_value().string_value
        self.detection_vicinity_radius = self.get_parameter('detection_vicinity_radius').get_parameter_value().double_value
        self.detection_hold_seconds = self.get_parameter('detection_hold_seconds').get_parameter_value().double_value

        # # Choose which waypoints file to load
        # if self.real:
        #     waypoints_file = 'config/waypoints_duff.yaml'
        if self.real:
            waypoints_file = 'config/waypoints_engquad.yaml'
        else:
            #waypoints_file = 'config/sim_waypoints_condensed.yaml'
            waypoints_file = 'config/waypoints.yaml'
            waypoints_file = 'config/waypoints_long.yaml'

        self.get_logger().info(f"Waypoints file: {waypoints_file}")
        # Every entry in the YAML is treated as a real navigation target.
        # The local-frame origin (north=0, west=0) is captured from the first
        # GPS fix we receive, NOT from the YAML, so the rover's actual starting
        # position defines the frame.
        self.waypoints = self.load_waypoints(waypoints_file)

        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Shutting down.')
            rclpy.shutdown()
            return

        # Local-frame origin is captured from the rover's first GPS fix in
        # gps_callback below. Until then, next_coordinate is None and the
        # control loop is a no-op.
        self.initial_lat = None
        self.initial_lon = None
        self.origin_captured = False

        # Drive every waypoint in the YAML, starting at index 0.
        self.current_waypoint_index = 0
        self.next_waypoint = self.waypoints[self.current_waypoint_index]
        self.next_coordinate_lat = self.next_waypoint['latitude']
        self.next_coordinate_lon = self.next_waypoint['longitude']

        # Change from initial pose
        self.north = 0.0 # meters
        self.west = 0.0 # meters
        self.yaw = 0.0 # radians

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints.')


        # One-shot subscriber that captures the rover's starting GPS as the
        # local-frame origin. Same NavSatFix the localization node uses.
        self.gps_subscription = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_callback, 10
        )

        # Subscriptions to localization and game object data
        self.pose_subscription = self.create_subscription(
            TwistStamped, '/autonomy/pose/robot/global', self.pose_callback, 10
        )
        self.object_subscription = self.create_subscription(
            Twist, '/autonomy/target_object/position', self.object_callback, 10
        )
        # Single best detection from the base-station YOLO node (round-trips
        # back over the radio link). Drives the detect-and-stop flow.
        self.detection_subscription = self.create_subscription(
            Detection, '/autonomy/detection/result', self.detection_callback, 10
        )

        # Publishers
        self.state_publisher = self.create_publisher(String, '/autonomy/state', 10)
        self.previous_target_publisher = self.create_publisher(Float32MultiArray, '/autonomy/target/previous', 10)
        self.next_coordinate_publisher = self.create_publisher(Float32MultiArray, '/autonomy/target/global', 10)
        self.object_publisher = self.create_publisher(String, '/autonomy/target_object/name', 10)
        # Tells the base-station detection node which YOLO class to look for
        # right now ("none" when not hunting). Published every control loop.
        self.active_target_publisher = self.create_publisher(String, '/autonomy/detection/active_target', 10)
        # Authoritative onboard arrived/detected state. Held True through the
        # detection hold. Drives the GUI banner and a future LED indicator.
        self.arrived_publisher = self.create_publisher(Bool, '/autonomy/detection/arrived', 10)
        # Rover starts at the local-frame origin, so the "previous target" we
        # advertise to the local planner is (0, 0).
        self.previous_target = [0.0, 0.0]
        # Filled in once the GPS origin is captured.
        self.next_coordinate = None

        # Per-waypoint mission metadata, indexed by waypoint index (0-based).
        # update_search_params falls back to a plain coordinate target if an
        # index isn't listed here, so YAMLs with fewer entries than this map
        # (e.g. eng-quad smoke tests) just drive coord-to-coord without any
        # AR-marker search behavior.
        # Tuple: (object_name, threshold_m, search_radius_m, r_step_m, theta_step_deg)
        self.targets = {
            0: ('coordinate', 2.0, 0.0, 0.0, 0.0),
            1: ('ar1', 1.5, 10.0, 0.2, 15.0),
            2: ('ar2', 1.5, 15.0, 0.2, 15.0),
            3: ('ar3', 1.5, 20.0, 0.2, 15.0), # 1
            4: ('ar1', 1.5, 10.0, 0.2, 30.0),
            5: ('ar2', 1.5, 15.0, 0.2, 30.0),
            6: ('ar3', 1.5, 20.0, 0.2, 30.0), # 2
            7: ('ar1', 1.5, 10.0, 0.2, 45.0),
            8: ('ar2', 1.5, 15.0, 0.2, 45.0),
            9: ('ar3', 1.5, 20.0, 0.2, 45.0), # 3
            10: ('ar1', 1.5, 10.0, 0.4, 15.0),
            11: ('ar2', 1.5, 15.0, 0.4, 15.0),
            12: ('ar3', 1.5, 20.0, 0.4, 15.0), # 4
            13: ('ar1', 1.5, 10.0, 0.4, 30.0),
            14: ('ar2', 1.5, 15.0, 0.4, 30.0),
            15: ('ar3', 1.5, 20.0, 0.4, 30.0), # 5
            16: ('ar1', 1.5, 10.0, 0.4, 45.0),
            17: ('ar2', 1.5, 15.0, 0.4, 45.0),
            18: ('ar3', 1.5, 20.0, 0.4, 45.0),  # 6
            19: ('ar1', 1.5, 10.0, 0.6, 30.0),
            20: ('ar2', 1.5, 15.0, 0.6, 30.0),
            21: ('ar3', 1.5, 20.0, 0.6, 30.0), # 7
            # --- URC 2026 detect-and-stop objects -----------------------------
            # A target named with a YOLO class ('mallet'|'bottle'|'ice_pick')
            # uses the base-station detect-and-stop flow: drive to the GNSS
            # coordinate, start sampling detection within search_radius (set it
            # to ~GNSS error: ~5 m for objects 1-2, ~12 m for object 3), and STOP
            # the moment the object is confidently detected -- at ANY distance.
            # The 3rd element (search_radius) is the vicinity at which sampling
            # starts; r_step/theta_step only matter for the no-detection spiral
            # fallback. Map these to the waypoint indices of your mission YAML:
            #   N:   ('mallet',   1.5, 5.0,  0.4, 30.0),  # object 1
            #   N+1: ('ice_pick', 1.5, 5.0,  0.4, 30.0),  # object 2 (rock pick hammer)
            #   N+2: ('bottle',   1.5, 12.0, 0.4, 30.0),  # object 3 (water bottle)
        }

        # Initial state is driving to first target coordinate
        self.current_state = None
        self.threshold = None
        self.search_radius = None
        self.r_step = None
        self.theta_step = None
        self.searched = False
        self.update_search_params() # Updates above values using current waypoint index

        self.target_position = None
        self.search_waypoints = deque()
        self.dist_to_coord = math.inf

        # --- Detect-and-stop state (YOLO objects) ---------------------------
        # Class to arm the base detector with this loop ("none" => disarmed).
        self.active_target = NO_TARGET
        # Latest detection of the *current* target reported by the base node.
        self.detection_seen = False
        self.detection_label = ''
        # True once the current YOLO target has been confidently detected; held
        # through detection_hold_seconds before we advance.
        self.arrived = False
        self.arrived_time = None

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    def gps_callback(self, msg: NavSatFix):
        """
        Capture the rover's first GPS fix as the local-frame (north=0, west=0)
        origin, then compute the local coords of the first waypoint. This is
        a one-shot — we never update the origin again.
        """
        if self.origin_captured:
            return
        self.initial_lat = msg.latitude
        self.initial_lon = msg.longitude
        n, w = self.get_north_west_meters(self.next_coordinate_lat, self.next_coordinate_lon)
        self.next_coordinate = (n, w)
        self.origin_captured = True
        self.get_logger().info(
            f"GPS origin captured lat={self.initial_lat:.8f} lon={self.initial_lon:.8f} "
            f"first target at (n={n:+.2f} m, w={w:+.2f} m)"
        )

    def update_targets(self):
        """
        Updates targets from gps coordinate
        """
        # Update distance to the target GPS coordinate
        dx = self.next_coordinate[0] - self.north
        dy = self.next_coordinate[1] - self.west
        self.dist_to_coord = math.sqrt(dx**2 + dy**2)
        
        reached = False
        # If driving to just coordinate, distance to coord is sufficient to check
        if self.dist_to_coord < self.threshold:
            if self.current_state == 'coordinate':
                reached = True
            elif len(self.search_waypoints) > 0 and self.target_position is None:
                self.next_coordinate = self.search_waypoints.popleft()
            elif self.target_position is None and not self.searched:
                self.search_waypoints = generate_search_points(self.next_coordinate, self.search_radius, self.r_step, self.theta_step)
                self.searched = True
                self.threshold = 2.0 # to account for badly selected waypoints
                self.get_logger().info(f"Search waypoints generated: {self.search_waypoints}")
            else:
                reached = True
                self.searched = False
        # If the object has been found and within allowed threshold, target reached
        elif self.target_position is not None:
            dx = self.target_position[0] - self.north
            dy = self.target_position[1] - self.west
            d = math.sqrt(dx**2 + dy**2)
            reached = (d < self.threshold)
            #self.get_logger().info(f"Distance to target object: {d}")
            if d < self.threshold * 2:
                self.publish_target_name()
        if not reached:
            return

        self.advance_waypoint()

    def advance_waypoint(self):
        """Advance to the next waypoint in the YAML, reloading its target
        metadata (via update_search_params) and GNSS coordinate. Returns False
        when there are no more waypoints."""
        self.current_waypoint_index += 1
        if self.current_waypoint_index >= len(self.waypoints):
            return False
        # Update target object and coordinates
        self.update_search_params()
        self.previous_target = self.next_coordinate
        self.next_waypoint = self.waypoints[self.current_waypoint_index]
        self.next_coordinate_lat = self.next_waypoint['latitude']
        self.next_coordinate_lon = self.next_waypoint['longitude']
        n, w = self.get_north_west_meters(self.next_coordinate_lat, self.next_coordinate_lon)
        self.next_coordinate = [n, w]
        return True


    def pose_callback(self, msg):
        """
        Updates the current pose of the robot (north, west, yaw).
        """
        self.north = msg.twist.linear.x
        self.west = msg.twist.linear.y
        self.yaw = msg.twist.angular.z
    
    def object_callback(self, msg):
        self.target_position = [msg.linear.x, msg.linear.y]
        self.get_logger().info(f"Current target {self.current_state} found at {self.target_position}")

    def detection_callback(self, msg: Detection):
        """Record the latest base-station detection. ``detected`` is already
        gated (by the base node) on the armed class clearing the stop
        confidence threshold, so we just remember it for the current target."""
        self.detection_seen = bool(msg.detected)
        self.detection_label = (msg.label or '').strip().lower()

    def control_loop(self):
        """
        Main control loop running at 10 Hz.
        Determines movement commands based on current position and waypoints.
        """
        self.publish_state()

        if not self.origin_captured:
            # No GPS origin yet -> we don't know where we are in the local
            # frame, so we can't navigate. Wait for the first fix.
            self.get_logger().info(
                f"Waiting for first GPS fix on {self.gps_topic} to set local-frame origin...",
                throttle_duration_sec=2.0,
            )
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached.', throttle_duration_sec=5.0)
            # If the mission finished on a confident detection, keep the rover
            # stopped (hold the latch); otherwise disarm the base detector.
            if not self.arrived:
                self.active_target = NO_TARGET
            self.publish_detection_state()
            return

        if self.current_state in YOLO_CLASSES:
            # Detect-and-stop object: arm the base detector near the coordinate,
            # stop on a confident detection, hold, then advance. While stopped
            # we freeze the driving target (the controller is already halted via
            # the latched /autonomy/motion/stop).
            self.update_detection()
            if not self.arrived:
                self.update_targets()
        else:
            # Plain coordinate / AR-marker target: existing behavior, detector
            # disarmed.
            self.active_target = NO_TARGET
            self.update_targets()

        self.publish_targets()
        self.publish_detection_state()

    def update_detection(self):
        """Detect-and-stop sequencing for YOLO mission objects.

        Arms the base-station detector within vicinity of the object's GNSS
        coordinate, latches an onboard arrived/detected state on a confident
        detection, holds it for detection_hold_seconds, then advances. The
        actual rover halt is done by the controller via the latched
        /autonomy/motion/stop (set by the base node) -- this method only
        sequences the mission and the onboard arrived state. We never drive up
        to the object: the rover may stop at any distance.
        """
        dx = self.next_coordinate[0] - self.north
        dy = self.next_coordinate[1] - self.west
        self.dist_to_coord = math.sqrt(dx * dx + dy * dy)

        if self.arrived:
            # Keep the detector armed so the base node holds the stop latched.
            self.active_target = self.current_state
            elapsed = time.time() - (self.arrived_time or time.time())
            if elapsed >= self.detection_hold_seconds:
                advanced = self.advance_waypoint()
                if advanced:
                    self.get_logger().info(
                        f"Target done (held {elapsed:.1f}s) -- advancing to waypoint "
                        f"{self.current_waypoint_index}")
                else:
                    # No more waypoints: stay stopped (arrived stays True so the
                    # latch holds). Mission complete.
                    self.get_logger().info(
                        'Final object detected -- mission complete, holding stop.',
                        throttle_duration_sec=5.0)
            return

        # Not yet arrived: arm the detector once within vicinity of the
        # coordinate. Use the per-target search_radius if set, else the param.
        vicinity = self.search_radius if self.search_radius and self.search_radius > 0 \
            else self.detection_vicinity_radius
        within = self.dist_to_coord < vicinity
        self.active_target = self.current_state if within else NO_TARGET

        if within and self.detection_seen and self.detection_label == self.current_state:
            self.arrived = True
            self.arrived_time = time.time()
            self.get_logger().info(
                f"*** OBJECT DETECTED: {self.current_state} -- STOPPING "
                f"(distance to coordinate {self.dist_to_coord:.1f} m) ***")

    def publish_detection_state(self):
        """Publish the armed target (for the base detector) and the onboard
        arrived/detected state (for the GUI banner and a future LED)."""
        target_msg = String()
        target_msg.data = self.active_target
        self.active_target_publisher.publish(target_msg)

        arrived_msg = Bool()
        arrived_msg.data = self.arrived
        self.arrived_publisher.publish(arrived_msg)

    def publish_targets(self):
        """
        Publish previous and next targets
        """
        prev_target = Float32MultiArray()
        prev_target.data = [float(self.previous_target[0]), float(self.previous_target[1])]
        self.previous_target_publisher.publish(prev_target)

        next_target = Float32MultiArray()
        if self.target_position is None:
            next_target.data = [float(self.next_coordinate[0]), float(self.next_coordinate[1])]
        else:
            next_target.data = [float(self.target_position[0]), float(self.target_position[1])]
        self.next_coordinate_publisher.publish(next_target)

        # Look for target if not yet found
        if self.dist_to_coord < self.search_radius and self.target_position is None:
            self.publish_target_name()

    def publish_target_name(self):
        """
        Publish the name of the autonomy game object to be searched for
        """
        object_msg = String()
        object_msg.data = self.current_state
        self.object_publisher.publish(object_msg)

    def get_north_west_meters(self, target_lat, target_lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        using previous GPS coordinate as a reference
        Publishes to the target pose topic
        """
        R = 6378137.0
        lat1_rad = math.radians(target_lat)
        lat2_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(target_lon)
        lon2_rad = math.radians(self.initial_lon)
        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        # Compute north west distances
        north = -1.0 * delta_lat * R
        west = delta_lon * R * math.cos(mean_lat)
        return north, west
    
    def update_search_params(self):
        """
        Update search params with the current waypoint index. Falls back to a
        plain coordinate target (no AR-marker search) for any index that isn't
        explicitly listed in self.targets.
        """
        t = self.targets.get(
            self.current_waypoint_index,
            ('coordinate', self.waypoint_tolerance, 0.0, 0.0, 0.0),
        )
        self.current_state = t[0]
        self.threshold = t[1]
        self.search_radius = t[2]
        self.r_step = t[3]
        self.theta_step = t[4]
        # Reset object variables
        self.target_position = None
        self.search_waypoints = deque()
        # Reset detect-and-stop state for the new target.
        self.active_target = NO_TARGET
        self.detection_seen = False
        self.detection_label = ''
        self.arrived = False
        self.arrived_time = None
        
    def publish_state(self):
        """
        Publish current state
        """
        state = String()
        state.data = self.current_state
        self.state_publisher.publish(state)

    def load_waypoints(self, waypoints_file):
        """
        Loads waypoints from a YAML file.
        """
        package_share = get_package_share_directory('autonomous_navigation')
        full_path = os.path.join(package_share, waypoints_file)

        if not os.path.isfile(full_path):
            self.get_logger().error(f'Waypoints file not found: {full_path}')
            return []

        with open(full_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
                self.get_logger().info('Waypoints loaded successfully.')
            except yaml.YAMLError as e:
                self.get_logger().error(f'Error parsing YAML file: {e}')
                return []

        waypoints = data.get('waypoints', [])
        return waypoints

    def destroy_node(self):
        super().destroy_node()

def generate_search_points(center, radius, radial_step, angular_step_deg):
    """
    Helper function to generate waypoints in an outwards spiral
    to look for objects once a coordinate has been reached
    """
    points = deque()
    r = 0
    theta = 0
    while r < radius:
        r += radial_step
        theta += angular_step_deg
        if theta >= 360:
            theta %= 360
        x = r * np.cos(math.radians(theta)) + center[0]
        y = r * np.sin(math.radians(theta)) + center[1]
        points.append((x, y))
    return points

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down state machine node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
