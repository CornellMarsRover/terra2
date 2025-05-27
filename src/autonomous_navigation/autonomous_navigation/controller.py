#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray, String
import math
import time

from ament_index_python.packages import get_package_share_directory
from cmr_msgs.msg import AutonomyDrive

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('real', True) # FALSE IF RUNNING IN SIMULATION
        self.real = self.get_parameter('real').get_parameter_value().bool_value

        # Subscribe to the robot pose topic
        self.pose_subscription = self.create_subscription(
            TwistStamped,
            '/autonomy/pose/robot/global',
            self.update_pose,
            10
        )

        self.waypoint_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/path/next_waypoint',
            self.update_waypoint,
            10
        )

        # Drive command publishers
        self.ackerman_publisher = self.create_publisher(AutonomyDrive, '/autonomy/move/ackerman', 10)
        self.point_turn_publisher = self.create_publisher(Twist, '/autonomy/move/point_turn', 10)
        self.movement_id_publisher = self.create_publisher(String, '/autonomy/move/move_type', 10)

        # Store current robot position
        self.robot_position = (0.0, 0.0)
        self.yaw = 0.0

        # Movement parameters
        self.point_turn_velocity = 0.5
        self.ackerman_velocity = 0.5
        if not self.real:
            self.ackerman_velocity = 0.057
            self.point_turn_velocity = 0.085
        # Timers and state
        self.last_movement = 'ackerman'
        self.last_command_time = self.get_clock().now().to_msg()
        self.min_wait = 0.5 if self.real else 0.5

        self.point_turn_threshold = 45

        # Next waypoint in path
        self.waypoint = None
        self.use_stanley = False   # Will be set by the 3rd element in the waypoint array

        # Stanley controller parameters
        self.k_stanley = 1.0       # Gain for cross-track error term
        # If speed is small, dividing by speed can be large, so be mindful. 
        # We'll pretend the speed is ~ self.ackerman_velocity

        # Create a timer to periodically command velocities
        self.drive_commander = self.create_timer(0.1, self.follow_waypoint)

    def follow_waypoint(self):
        """
        Logic for sending drive commands:
          1. If no waypoint, do nothing.
          2. Compute heading error to next waypoint.
          3. If heading error is above threshold, do a point turn.
          4. Otherwise, if 'use_stanley' is True, run Stanley logic for steering angle.
          5. If 'use_stanley' is False, use your original "ackerman" approach.
        """
        if self.waypoint is None:
            return

        # Position difference to the waypoint
        x_error = self.waypoint[0] - self.robot_position[0]
        y_error = self.waypoint[1] - self.robot_position[1]

        # Distance
        distance_to_wp = math.sqrt(x_error**2 + y_error**2)

        # Heading to target
        angle_to_target = math.atan2(y_error, x_error)
        heading_error = angle_to_target - self.yaw
        # Normalize heading error to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        angle_error_deg = math.degrees(heading_error)
        #self.get_logger().info(f"Heading error: {angle_error_deg}")
        
        # If large angle error to next waypoint, use point-turn
        if abs(angle_error_deg) > self.point_turn_threshold:
            # Possibly wait for wheels to re-position
            curr_time = self.get_clock().now().to_msg()
            dt = self.compute_time_delta(curr_time, self.last_command_time)
            #self.get_logger().info(f"dt: {dt}")
            if self.last_movement == "ackerman" and dt < self.min_wait:
                # Just publish a tiny turn command
                self.publish_point_turn(0.00001)
            else:
                # Actual point turn
                turn_sign = 1.0 if angle_error_deg < 0 else -1.0
                if self.real:
                    turn_sign *= -1.0
                self.publish_point_turn(turn_sign * self.point_turn_velocity)
                self.last_movement = 'point_turn'
                self.last_command_time = curr_time
            self.publish_movement(self.last_movement)
            return

        # If angle error is not large, use either the original ackerman or Stanley
        if self.use_stanley:
            # Compute Stanley control for steering
            # heading_error + arctan(k * cross_track / (speed+epsilon))

            # 1) heading error is computed above
            # 2) cross-track error: project robot position onto line from 
            #    "previous waypoint" to "this waypoint" or just treat the line from robot -> waypoint 
            #    for a short example. We'll do a simple approach here.

            # For demonstration, let's assume cross-track is just the perpendicular distance 
            # from the robot to the line connecting (0,0)->(x_error,y_error).
            # For better performance, you'd want the segment from the "previous waypoint" 
            # to the "current waypoint" in the full path.  We'll keep it minimal.

            # Robot in local frame = (0,0). The path is a line from (0,0)->(x_error,y_error).
            # Cross track is 0 for the direct line approach, so let's do a small cheat:
            # We'll treat the waypoint as if there's a line along angle_to_target. 
            # If you have more of the path, you'd do a better geometry calculation.

            cross_track_error = 0.0  
            # If you had the previous waypoint or a local path, compute an actual perpendicular 
            # distance to that segment. For now, we show a placeholder of 0.0.

            # Speed-based term
            speed = self.ackerman_velocity + 1e-6

            stanley_steer = heading_error + math.atan2(self.k_stanley * cross_track_error, speed)
            # Convert to degrees
            steer_angle_deg = math.degrees(stanley_steer)

            # Publish ackerman with stanley_steer
            curr_time = self.get_clock().now().to_msg()
            dt = self.compute_time_delta(curr_time, self.last_command_time)
            if self.last_movement == 'point_turn' and dt < self.min_wait:
                # Wait after a point turn
                self.publish_ackerman(0.0, steer_angle_deg)
            else:
                self.publish_ackerman(self.ackerman_velocity, steer_angle_deg)
                self.last_movement = 'ackerman'
                self.last_command_time = curr_time
            self.publish_movement(self.last_movement)

        else:
            # Not using Stanley: original logic
            curr_time = self.get_clock().now().to_msg()
            dt = self.compute_time_delta(curr_time, self.last_command_time)

            # If we just point turned, wait
            if self.last_movement == 'point_turn' and dt < self.min_wait:
                self.publish_ackerman(0.0, math.degrees(heading_error))
            else:
                self.publish_ackerman(self.ackerman_velocity, math.degrees(heading_error))
                self.last_movement = 'ackerman'
                self.last_command_time = curr_time

            self.publish_movement(self.last_movement)

    def publish_movement(self, movement):
        """
        Publishes id of last movement
        """
        msg = String()
        msg.data = movement
        self.movement_id_publisher.publish(msg)

    def publish_point_turn(self, point_turn_velocity):
        """
        Publishes point turn message
        """
        point_turn_msg = Twist()
        point_turn_msg.angular.z = point_turn_velocity
        self.point_turn_publisher.publish(point_turn_msg)

    def publish_ackerman(self, vel, steer_angle_deg):
        """
        Publish ackerman drive message (steer_angle in degrees)
        """
        drive_msg = AutonomyDrive()
        drive_msg.vel = vel
        drive_msg.fl_angle = steer_angle_deg
        drive_msg.fr_angle = steer_angle_deg
        drive_msg.bl_angle = 0.0
        drive_msg.br_angle = 0.0

        self.ackerman_publisher.publish(drive_msg)

    def update_pose(self, msg):
        """
        Callback function to update the robot position and yaw
        """
        self.robot_position = (msg.twist.linear.x, msg.twist.linear.y)
        self.yaw = msg.twist.angular.z
        if not self.real:
            mag = math.degrees(abs(self.yaw))
            if (45.0 < mag < 135.0):
                self.ackerman_velocity = 0.06
                self.point_turn_velocity = 0.09
            else:
                self.ackerman_velocity = 0.057
                self.point_turn_velocity = 0.085

    def update_waypoint(self, msg):
        """
        Callback function to update waypoint to follow.
        We'll parse the third element (if present) as the "use_stanley" flag.
        """
        if len(msg.data) >= 2:
            self.waypoint = (msg.data[0], msg.data[1])
        if len(msg.data) >= 3:
            self.use_stanley = (msg.data[2] == 1.0)  # 1.0 means True
        else:
            self.use_stanley = False

    def compute_time_delta(self, current_stamp, last_stamp):
        """
        Compute time difference in seconds between two ROS 2 Time objects.
        """
        dt = (current_stamp.sec - last_stamp.sec) + \
             (current_stamp.nanosec - last_stamp.nanosec) * 1e-9
        return dt

    def stop_robot(self):
        self.publish_ackerman(0.0, 0.0)

    def destroy_node(self):
        self.stop_robot()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down controller node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
