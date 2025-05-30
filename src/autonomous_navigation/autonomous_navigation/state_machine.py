#!/usr/bin/env python3

"""
State Machine ROS2 Node
High-level logic for autonomy, with preplanned coarse waypoints
"""

import os
import math
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray, String
from collections import deque
from ament_index_python.packages import get_package_share_directory


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        # real vs sim flag
        self.declare_parameter('real', True)
        self.real = self.get_parameter('real').get_parameter_value().bool_value

        # Load GPS waypoints
        waypoints_file = (
            'config/waypoints_test_site_utah.yaml'
            if self.real else
            'config/sim_waypoints_condensed.yaml'
        )
        waypoints_path = os.path.join(
            get_package_share_directory('autonomous_navigation'),
            waypoints_file
        )
        self.get_logger().info(f"Waypoints file: {waypoints_path}")
        self.waypoints = self.load_waypoints(waypoints_path)
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded. Shutting down.')
            rclpy.shutdown()
            return

        # reference start coordinate
        start = self.waypoints[0]
        self.initial_lat = start['latitude']
        self.initial_lon = start['longitude']
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")

        # Preplan ALL coarse paths if running on real hardware
        self.coarse_paths = {}
        self.coarse_paths[1] = [(0.06498901918530464, -0.0850945757701993), (3.86498901899904, 13.71490542421816), (13.86498901899904, 23.71490542421816)]
        self.coarse_paths[2] = [(20.664989018812776, 39.314905424194876), (30.664989018812776, 49.314905424194876)]
        self.coarse_paths[3] = [(52.064989019185305, 58.71490542421816), (62.064989019185305, 68.71490542421816), (72.0649890191853, 78.71490542421816), (82.0649890191853, 88.71490542421816)]
        self.coarse_paths[4] = [(94.46498901862651, 104.71490542421816), (84.46498901862651, 114.71490542421816), (74.46498901862651, 124.71490542421816), (64.46498901862651, 134.71490542421816)]
        self.coarse_paths[5] = [(61.26498901844025, 144.31490542419488), (65.0649890191853, 158.11490542418323)]
        self.coarse_paths[6] = [(69.86498901899904, 171.51490542420652), (56.26498901844025, 175.31490542419488), (42.664989018812776, 179.11490542418323), (29.064989019185305, 182.9149054242298), (15.464989018626511, 186.71490542421816)]
        self.coarse_paths[7] = [(1.8649890189990401, 188.71490542421816), (-11.735010981559753, 185.11490542418323), (-25.335010981187224, 181.51490542420652), (-38.935010980814695, 177.9149054242298), (-52.53501098137349, 181.71490542421816), (-66.13501098100096, 178.11490542418323)]

        # State for iterating through coarse waypoints
        self.coarse_waypoints = deque()
        self.coarse_index = 0
    
        # Global waypoint index
        self.current_waypoint_index = 1

        # Robot state in local frame
        self.north = 0.0
        self.west = 0.0
        self.yaw = 0.0

        # Object-search configuration
        self.targets = {
            1: ('coordinate', 2.0, 10.0),
            2: ('coordinate', 2.0, 10.0),
            3: ('ar1', 2.0, 10.0),
            4: ('ar2`', 2.0, 10.0),
            5: ('ar3`', 2.0, 15.0),
            6: ('coordinate`', 2.0, 10.0),
            7: ('coordinate`', 2.0, 10.0),
        }

        self.r_step = 0.2
        self.theta_step = 15
        self.search_waypoint_threshold = 2.0

        # Object-search state
        self.current_object = None
        self.threshold      = None
        self.search_radius  = None
        self.object_found   = False
        self.target_position = [0.0, 0.0]
        self.next_coordinate = [0.0, 0.0]
        

        # ROS publishers & subscribers
        self.pose_sub = self.create_subscription(
            TwistStamped,
            '/autonomy/pose/robot/global',
            self.pose_callback, 10
        )
        self.obj_sub = self.create_subscription(
            Twist,
            '/autonomy/target_object/position',
            self.object_callback, 10
        )
        self.global_pub = self.create_publisher(
            Float32MultiArray,
            '/autonomy/target/global', 10
        )
        self.local_pub = self.create_publisher(
            Float32MultiArray,
            '/autonomy/target/local', 10
        )
        self.name_pub = self.create_publisher(
            String,
            '/autonomy/target_object/name', 10
        )
        self.led_pub = self.create_publisher(
            String,
            '/autonomy/led', 10
        )

        self.ang_step = 30.0
        self.r_step = 0.1
        self.search_waypoints = deque()
        # Kick off with the first target
        self.get_logger().info("Initializing first target & coarse segment")
        self.update_search_params()
        

        # Control loop @10Hz
        self.timer = self.create_timer(0.1, self.control_loop)


    def load_waypoints(self, path: str):
        if not os.path.isfile(path):
            self.get_logger().error(f'Waypoints file not found: {path}')
            return []
        with open(path, 'r') as f:
            try:
                data = yaml.safe_load(f)
                self.get_logger().info('Waypoints loaded successfully.')
            except yaml.YAMLError as e:
                self.get_logger().error(f'YAML parse error: {e}')
                return []
        return data.get('waypoints', [])


    def update_search_params(self):
        """
        Advance to the next global GPS waypoint:
          - set up object/search parameters
          - load preplanned coarse waypoints for this segment
        """
        if self.current_waypoint_index not in self.targets:
            return

        # assign search parameters
        obj, thr, sr = self.targets[self.current_waypoint_index]
        self.current_object = obj
        self.threshold      = thr
        self.search_radius  = sr
        self.object_found   = False
        if self.search_waypoints:
            self.search_waypoints.clear()

        # load precomputed coarse path for this segment (if real)
        if self.real:
            self.coarse_waypoints = deque(
                self.coarse_paths.get(self.current_waypoint_index, [])
            )
            self.get_logger().info(
                f"Loaded {len(self.coarse_waypoints)} coarse waypoints for segment #{self.current_waypoint_index}"
            )

        # compute global‐frame target position (north/west) of the GPS waypoint itself
        wp = self.waypoints[self.current_waypoint_index]
        n, w = self.get_north_west_meters(wp['latitude'], wp['longitude'])
        self.next_coordinate = [n, w]
        if self.current_object != 'coordinate':
            self.search_waypoints = deque(self.generate_search_waypoints(n,w,self.ang_step, self.r_step, self.search_radius))
            self.get_logger().info(f"search waypoints: {self.search_waypoints}")
        self.get_logger().info(
            f"Advancing to WP#{self.current_waypoint_index}: "
            f"{obj} @ [{n:.1f}, {w:.1f}] (thr={thr})"
        )


    def control_loop(self):
        """
        10Hz control.  If we have coarse waypoints (real==True),
        publish them one by one on '/autonomy/target/local' until done.
        Otherwise fall back on global/object logic.
        """
        # 2) global‐target / object search logic (as before)
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached.')
            return

        self.check_targets()
        self.publish_targets()

    def check_targets(self):
        
        dx2 = self.next_coordinate[0] - self.north
        dy2 = self.next_coordinate[1] - self.west
        d = math.sqrt((dx2**2) + (dy2**2))
        waypoint_dist = None
        reached = False
        timeout = False

        if self.coarse_waypoints and len(self.coarse_waypoints) > 0:
            tx, ty = self.coarse_waypoints[0]
            dx1, dy1 = tx - self.north, ty - self.west
            waypoint_dist = math.sqrt((dx1**2) + (dy1**2))
        self.get_logger().info(f"coordinate distance: {d}   waypoint distance: {waypoint_dist}   threshold: {self.threshold}")
        
        #if d > (self.search_radius * 2) or d > 10.0:
        
        if d > 10.0:
            if waypoint_dist is not None and waypoint_dist < 2.5:
                self.coarse_waypoints.popleft()
            if not (self.current_object != 'coordinate' and self.object_found):
                if self.coarse_waypoints and len(self.coarse_waypoints) > 0:
                    self.target_position = self.coarse_waypoints[0]
                else:
                    self.target_position = self.next_coordinate
        else:
            if self.current_object == 'coordinate':
                if d < 2.0:
                    reached = True
                else:
                    self.target_position = self.next_coordinate
            elif not self.object_found:
                if len(self.search_waypoints) == 0:
                    timeout = True
                else:
                    sx, sy = self.search_waypoints[0]
                    dx3, dy3 = self.north - sx, self.west - sy
                    d3 = math.sqrt((dx3**2) + (dy3**2))
                    if d3 < 2.0:
                        self.search_waypoints.popleft()
                        if len(self.search_waypoints) == 0:
                            timeout = True
                        else:
                            self.target_position = self.search_waypoints[0]
                    else:
                        self.target_position = self.search_waypoints[0]
                        self.publish_target_name()
            else:
                gx, gy = self.target_position[0], self.target_position[1]
                dx4, dy4 = self.north - gx, self.west - gy
                d4 = math.sqrt((dx4**2) + (dy4**2))
                if d4 < 1.5:
                    reached = True

        if reached or timeout:
            if timeout:
                self.get_logger().info("TARGET TIMEOUT")
            if reached:
                self.get_logger().info("TARGET REACHED")
            self.blink_led()
            self.current_waypoint_index += 1
            self.update_search_params()

    def generate_search_waypoints(self, x, y, a, r, max_r):
        pts = []
        curr_r = r
        curr_a = 0.0
        while curr_r < max_r:
            if curr_a > 180:
                curr_a = -1 * (360 - curr_a)
            a_rad = math.radians(curr_a)
            dx = curr_r * (math.cos(a_rad) - math.sin(a_rad))
            dy = curr_r * (math.sin(a_rad) + math.sin(a_rad))
            pts.append((x+dx, y+dy))
            if curr_a < 0:
                curr_a -= a
            else:
                curr_a += a
            curr_r += r
        return pts

    def publish_targets(self):
        local_msg = Float32MultiArray()
        local_msg.data = [
            float(self.target_position[0]),
            float(self.target_position[1])
        ]
        global_msg = Float32MultiArray()
        global_msg.data = [
            float(self.target_position[0]),
            float(self.target_position[1])
        ]
        self.local_pub.publish(local_msg)
        self.global_pub.publish(global_msg)

    def publish_target_name(self):
        m = String()
        m.data = self.current_object
        self.name_pub.publish(m)


    def blink_led(self):
        m = String()
        m.data = "blink"
        self.led_pub.publish(m)


    def pose_callback(self, msg: TwistStamped):
        self.north = msg.twist.linear.x
        self.west  = msg.twist.linear.y
        self.yaw   = msg.twist.angular.z
        

    def object_callback(self, msg: Twist):
        if self.current_object == 'coordinate':
            return
        obj_id = int(msg.linear.z)
        if obj_id != self.current_waypoint_index:
            return
        self.object_found = True
        self.target_position = [msg.linear.x, msg.linear.y]
        self.get_logger().info(f"Found {self.current_object} @ {self.target_position}")


    def get_north_west_meters(self, target_lat, target_lon):
        """
        Convert (lat,lon) → local (north,west) from the starting reference.
        """
        R = 6378137.0
        φ1 = math.radians(self.initial_lat)
        φ2 = math.radians(target_lat)
        λ1 = math.radians(self.initial_lon)
        λ2 = math.radians(target_lon)

        dφ = φ2 - φ1
        dλ = λ2 - λ1
        meanφ = 0.5 * (φ1 + φ2)

        north = dφ * R
        west  = -dλ * R * math.cos(meanφ)
        return north, west


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
