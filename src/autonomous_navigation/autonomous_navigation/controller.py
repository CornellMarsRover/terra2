#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32MultiArray, String
import math

from autonomous_navigation import drive_command

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('real', True) # FALSE IF RUNNING IN SIMULATION
        self.real = self.get_parameter('real').get_parameter_value().bool_value
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.waypoint_tolerance = self.get_parameter(
            'waypoint_tolerance').get_parameter_value().double_value
        self.declare_parameter('input_timeout_s', 1.0)
        self.input_timeout_s = self.get_parameter('input_timeout_s').value

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

        self.stop_subscription = self.create_subscription(
            String,
            "/autonomy/stop",
            self.set_stop,
            10
        )
        self.stopped = False

        # The shared RoverNet drive node arbitrates this against manual input.
        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel_drives', 10)
        self.movement_id_publisher = self.create_publisher(String, '/autonomy/move/move_type', 10)

        # Store current robot position
        self.robot_position = (0.0, 0.0)
        self.yaw = 0.0
        self.last_pose_time_s = None
        self.last_waypoint_time_s = None

        # Movement parameters
        self.point_turn_velocity = 0.4
        self.ackerman_velocity = 0.9
        self.num_waypoints = 0
        # /cmd_vel_drives is normalized identically in simulation and hardware.
        # Timers and state
        self.last_movement = 'ackerman'
        self.last_command_time = self.get_clock().now().to_msg()
        self.min_wait = 1.0 if self.real else 0.5

        self.point_turn_threshold = 40

        # Next waypoint in path
        self.prev_waypoint = (0.0, 0.0)
        self.waypoint = None
        self.use_stanley = False   # Will be set by the 3rd element in the waypoint array

        self.k_stanley = 0.1

        # Create a timer to periodically command velocities
        self.drive_commander = self.create_timer(0.1, self.follow_waypoint)

    def follow_waypoint(self):
        """
        Send a point-turn or forward command toward the active waypoint.
        """
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.waypoint is None or not drive_command.inputs_fresh(
            now_s, self.last_pose_time_s, self.last_waypoint_time_s,
            self.input_timeout_s,
        ):
            self.stop_robot()
            self.publish_movement('stopped')
            return

        if self.stopped:
            self.stop_robot()
            return

        # Position difference to the waypoint
        x_error = self.waypoint[0] - self.robot_position[0]
        y_error = self.waypoint[1] - self.robot_position[1]

        if drive_command.waypoint_reached(
            self.robot_position, self.waypoint, self.waypoint_tolerance
        ):
            self.stop_robot()
            self.publish_movement('stopped')
            return

        # Heading to target
        angle_to_target = math.atan2(y_error, x_error)
        heading_error = angle_to_target - self.yaw
        # Normalize heading error to [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        angle_error_deg = math.degrees(heading_error)
        #self.get_logger().info(f"Heading error: {angle_error_deg}")

        # If large angle error to next waypoint, use point-turn
        if abs(angle_error_deg) > self.point_turn_threshold:
            self.point_turn_threshold = 20 # reduce threshold if point turning
            # Possibly wait for wheels to re-position
            curr_time = self.get_clock().now().to_msg()
            dt = self.compute_time_delta(curr_time, self.last_command_time)
            #self.get_logger().info(f"dt: {dt}")
            if self.last_movement == "ackerman" and dt < self.min_wait:
                # Stop while the steering modules transition to point-turn mode.
                self.publish_point_turn(0.0)
                #self.point_turn_threshold = 40
            else:
                # Actual point turn
                turn_sign = 1.0 if angle_error_deg > 0.0 else -1.0
                self.publish_point_turn(turn_sign * self.point_turn_velocity)
                self.last_movement = 'point_turn'
                self.last_command_time = curr_time
            self.publish_movement(self.last_movement)
            return

        else:
            self.point_turn_threshold = 50
        curr_time = self.get_clock().now().to_msg()
        dt = self.compute_time_delta(curr_time, self.last_command_time)
        steer_angle_deg = math.degrees(heading_error)
        if self.last_movement == 'point_turn' and dt < self.min_wait:
            self.publish_ackerman(0.0, steer_angle_deg)
        else:
            self.publish_ackerman(self.ackerman_velocity, steer_angle_deg)
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
        """Publish a signed point turn to the shared drive implementation."""
        command = drive_command.point_turn_command(point_turn_velocity)
        msg = Twist()
        msg.linear.x = command.linear_x
        msg.angular.z = command.angular_z
        self.drive_publisher.publish(msg)

    def publish_ackerman(self, vel, steer_angle_deg):
        """Publish forward motion plus bounded heading correction."""
        command = drive_command.forward_heading_command(vel, steer_angle_deg)
        drive_msg = Twist()
        drive_msg.linear.x = command.linear_x
        drive_msg.angular.z = command.angular_z
        self.drive_publisher.publish(drive_msg)

    def update_pose(self, msg):
        """
        Callback function to update the robot position and yaw
        """
        values = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Ignoring non-finite autonomy pose')
            return
        self.robot_position = values[:2]
        self.yaw = values[2]
        self.last_pose_time_s = self.get_clock().now().nanoseconds * 1e-9

    def update_waypoint(self, msg):
        """
        Callback function to update waypoint to follow.
        We'll parse the third element (if present) as the "use_stanley" flag.
        """
        waypoint = None
        if self.waypoint is not None:
            waypoint = self.waypoint
        if len(msg.data) < 2 or not all(math.isfinite(v) for v in msg.data):
            self.get_logger().error('Ignoring malformed autonomy waypoint')
            return
        self.waypoint = (msg.data[0], msg.data[1])
        self.last_waypoint_time_s = self.get_clock().now().nanoseconds * 1e-9
        if len(msg.data) >= 4:
            self.use_stanley = (msg.data[2] == 1.0)  # 1.0 means True
            self.num_waypoints = msg.data[3]
        else:
            self.use_stanley = False
        if waypoint is not None and waypoint != self.waypoint:
            self.prev_waypoint = waypoint

    def compute_time_delta(self, current_stamp, last_stamp):
        """
        Compute time difference in seconds between two ROS 2 Time objects.
        """
        dt = (current_stamp.sec - last_stamp.sec) + \
             (current_stamp.nanosec - last_stamp.nanosec) * 1e-9
        return dt

    def stop_robot(self):
        self.publish_ackerman(0.0, 0.0)

    def set_stop(self, msg):
        self.stopped = True

    def destroy_node(self):
        if rclpy.ok():
            self.stop_robot()
        super().destroy_node()

    def cross_track_error(self):
        """
        Compute the perpendicular distance from current pose to current segment
        """
        x1, y1 = self.waypoint[0], self.waypoint[1]
        x2, y2 = self.prev_waypoint[0], self.prev_waypoint[1]
        x3, y3 = self.robot_position[0], self.robot_position[1]

        # Direction vector of the line
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            return 0

        # Cross‐product magnitude between (p2−p1) and (p1−p3)
        # |(dx, dy) × (x1−x3, y1−y3)| = |dx*(y1−y3) − dy*(x1−x3)|
        num = abs(dx * (y1 - y3) - dy * (x1 - x3))

        # Length of the line segment vector
        den = math.hypot(dx, dy)

        return num / den
        
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down controller node.')
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
