#!/usr/bin/env python3

"""
Controller ROS2 Node

This node combines the functionalities of swerve_commander and ackerman_commander.
It switches between swerve drive control and ackerman steering based on the
message received on the `/control_mode` topic.
"""

import sys
import math
from typing import Dict, Tuple, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Bool, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from .swerve_joints import (
    JointEnd,
    JointKey,
    JointKind,
    JointSide,
    joint_name_from_key,
    joint_positions,
    joints,
)

from rclpy.duration import Duration

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
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)

        return output

class Controller(Node):
    """
    Controller node that combines swerve_commander and ackerman_commander functionalities.
    """

    # Descriptors for node parameters
    wheelbase_descriptor = ParameterDescriptor(
        name="wheelbase",
        type=ParameterType.PARAMETER_DOUBLE,
        description="Distance between front and rear wheels in meters",
    )
    wheel_track_descriptor = ParameterDescriptor(
        name="wheel_track",
        type=ParameterType.PARAMETER_DOUBLE,
        description="Distance between left and right wheels in meters",
    )
    wheel_radius_descriptor = ParameterDescriptor(
        name="wheel_radius",
        type=ParameterType.PARAMETER_DOUBLE,
        description="Radius of wheel in meters",
    )

    def __init__(self):
        # Initialize the node
        super().__init__("controller")

        # declare and get node parameters
        self.wheelbase: float
        self.wheel_track: float
        self.wheel_radius: float
        for descriptor in (
            self.wheelbase_descriptor,
            self.wheel_track_descriptor,
            self.wheel_radius_descriptor,
        ):
            self.declare_parameter(name=descriptor.name, descriptor=descriptor)
            value = self.get_parameter(descriptor.name).get_parameter_value()
            setattr(self, descriptor.name, value.double_value)

        # Set up a publisher for each of the 8 swerve joints using key iterator
        self.joint_publisher: Dict[JointKey, Publisher] = {}
        for key in joints():
            topic: str = f"/{joint_name_from_key(key)}/commands"
            self.joint_publisher[key] = self.create_publisher(
                Float64MultiArray, topic, 10
            )
            self.get_logger().debug(f"Publishing to topic: {topic}")

        # Initialize variables for control modes
        self.control_mode = "swerve"  # Default to swerve drive mode

        # Subscribe to /control_mode to determine control mode
        self.ackerman_subscriber = self.create_subscription(
            String,
            "/control_mode",
            self.control_mode_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Subscribed to /control_mode")

        # Subscribe to /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Subscribed to /cmd_vel")

        # Subscribe to /angle_error (only used in ackerman mode)
        self.angles_subscriber = self.create_subscription(
            Twist,
            "/angle_error",
            self.angles_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Subscribed to /angle_error")

        # Subscribe to /direct_control (only used in direct mode)
        self.direct_control_subscriber = self.create_subscription(
            Float32MultiArray,
            "/direct_control",
            self.direct_control_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Subscribed to /direct_control")

        # Initialize PID controller for ackerman mode
        self.pid = PIDController(
            Kp=0.5,
            Ki=0.0,
            Kd=0.1,
            correcting=False,
            output_limits=(-math.radians(30), math.radians(30)),
        )
        self.get_logger().info("Initialized PID controller for ackerman mode")

        # Variables to store angle error
        self.angle_error: Optional[float] = None  # radians

        # Variable to store linear velocity
        self.current_linear_x: float = 0.0  # m/s

        # Variables for swerve drive mode
        self.last_twist_msg: Optional[Twist] = None

        # Timer for PID updates in ackerman mode
        self.pid_timer = self.create_timer(
            1.0 / 30.0, self.pid_update_callback, callback_group=ReentrantCallbackGroup()
        )

    def control_mode_callback(self, msg: String) -> None:
        """
        Callback for /control_mode subscriber.
        Sets the control mode based on the received message.
        """
        self.control_mode = msg.data.lower()
        if self.control_mode == "ackerman":
            self.get_logger().info("Switched to Ackermann control mode.")
            self.pid.reset()
        elif self.control_mode == "swerve":
            self.get_logger().info("Switched to Swerve drive control mode.")
        elif self.control_mode == "direct":
            self.get_logger().info("Switched to Direct control mode.")
        else:
            self.get_logger().warn(f"Unknown control mode received: {self.control_mode}")

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Callback for /cmd_vel subscriber.
        Depending on the control mode, processes the Twist message accordingly.
        """
        if self.control_mode == "ackerman":
            # ackerman mode: Only use linear.x component
            self.current_linear_x = msg.linear.x
            self.get_logger().debug(
                f"ackerman Mode - Received /cmd_vel: linear.x = {self.current_linear_x} m/s"
            )
            # Wheel velocities will be updated in pid_update_callback
        elif self.control_mode == "swerve":
            # Swerve mode: Process the Twist message as in swerve_commander
            self.last_twist_msg = msg
            self.twist_to_swerve(msg)
            self.get_logger().info("Swerve Control Active")
            self.get_logger().info(f"command velocity: {msg}")
        elif self.control_mode == "direct":
            # Direct mode: Ignore /cmd_vel or handle differently if needed
            self.get_logger().debug("Direct Mode - /cmd_vel received but ignored.")
        else:
            self.get_logger().warn(f"Received /cmd_vel in unknown control mode: {self.control_mode}")


    def angles_callback(self, msg: Twist) -> None:
        """
        Callback for /angle_error subscriber.
        Extracts angle error.
        Assumes:
            - msg.angular.z is angle_error in radians
        """
        self.angle_error = msg.angular.z
        #self.get_logger().info(f"desired yaw = {math.degrees(msg.angular.x):.2f} deg")
        #self.get_logger().info(f"current yaw = {math.degrees(msg.angular.y):.2f} deg")
        self.get_logger().info(f"yaw error = {math.degrees(msg.angular.z):.2f} deg")

    def pid_update_callback(self) -> None:
        """
        Timer callback to update steering angles based on PID controller output in ackerman mode.
        Adjusts the steering angles of the front two wheels to correct angular error.
        """
        if self.control_mode != "ackerman":
            # Do nothing if not in ackerman mode
            return

        if self.angle_error is None:
            self.get_logger().debug("Waiting for yaw error...")
            return

        # Get current time
        current_time = self.get_clock().now()

        # Compute PID controller output
        control_effort = self.pid.compute(self.angle_error, current_time)

        self.get_logger().debug(
            f"PID Control Effort: {math.degrees(control_effort):.2f} degrees"
        )

        # Apply control effort to front steering angles
        # Front left and front right wheels are adjusted
        for side in [JointSide.LEFT, JointSide.RIGHT]:
            key = (JointKind.STEERING, JointEnd.FRONT, side)
            steering_angle_msg = Float64MultiArray()
            steering_angle_msg.data = [control_effort]  # radians
            self.joint_publisher[key].publish(steering_angle_msg)
            self.get_logger().debug(
                f"Published steering angle to {joint_name_from_key(key)}: {math.degrees(control_effort):.2f} degrees"
            )

        # Set rear steering angles to zero
        for side in [JointSide.LEFT, JointSide.RIGHT]:
            key = (JointKind.STEERING, JointEnd.REAR, side)
            steering_angle_msg = Float64MultiArray()
            steering_angle_msg.data = [0.0]  # radians
            self.joint_publisher[key].publish(steering_angle_msg)
            self.get_logger().debug(
                f"Published steering angle to {joint_name_from_key(key)}: 0.0 degrees"
            )

        # Compute wheel angular velocity based on linear velocity
        # Invert the wheel angular velocity to ensure positive linear.x moves forward
        wheel_angular_velocity = -self.current_linear_x / self.wheel_radius  # rad/s

        # Create wheel velocity message
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheel_angular_velocity]

        # Publish to all wheel velocity joints
        for key in joints():
            if key[0] == JointKind.WHEEL:
                self.joint_publisher[key].publish(wheel_msg)
                self.get_logger().debug(
                    f"Published wheel velocity to {joint_name_from_key(key)}: {wheel_angular_velocity:.2f} rad/s"
                )

    def twist_to_swerve(self, msg: Twist) -> None:
        """
        generates steering angle and wheel velocity for 8 swerve joints
        from input twist message on /cmd_vel and publishes it on respective
        controller topics.
        """
        vx: float = -1.0 * msg.linear.x
        vy: float = -1.0 * msg.linear.y
        wz: float = -1.0 * msg.angular.z

        # To prevent wheels from changing direction when stopping
        # if (wz == 0 and vx == 0 and vy == 0):
        # wheel position remains same
        if np.allclose([0, 0, 0], [wz, vx, vy]):
            self.stop()
            return

        # iterate over wheels
        # * calculate linear velocity components in cartesian plane
        # * calculate joint angle and direction given limited joint angles
        # * publish steering angle and wheel velocity for each wheel
        for end, side in joint_positions():
            # calculate linear velocity components in cartesian plane
            # for individual wheels
            kx = 1.0 if side is JointSide.RIGHT else -1.0
            ky = 1.0 if end is JointEnd.FRONT else -1.0
            wheel_vel_x = vx + (kx * self.wheelbase * wz / 2)
            wheel_vel_y = vy + (ky * self.wheel_track * wz / 2)

            # limit steering angle to +/- 90 degrees
            steering_angle: float = np.arctan2(wheel_vel_y, wheel_vel_x)
            rot_dir: float = 1.0

            if steering_angle > np.pi / 2.0:
                steering_angle -= np.pi
                rot_dir *= -1.0

            elif steering_angle < -np.pi / 2.0:
                steering_angle += np.pi
                rot_dir *= -1.0

            # calculate angular velocity of wheel
            wheel_angular_velocity = (
                rot_dir
                * np.sqrt(wheel_vel_x**2 + wheel_vel_y**2)
                / self.wheel_radius
            )

            # publish steering angle and wheel velocity
            for kind, value in (
                (JointKind.STEERING, steering_angle),
                (JointKind.WHEEL, wheel_angular_velocity),
            ):
                joint_msg: Float64MultiArray = Float64MultiArray()
                joint_msg.data = [value]
                key: JointKey = (kind, end, side)
                self.joint_publisher[key].publish(joint_msg)


    def direct_control_callback(self, msg: Float32MultiArray) -> None:
        """
        Callback for /direct_control subscriber.
        Directly controls the steering angles and wheel velocities based on received data.

        Expects an array of 8 floats in the following order:
        [front_right_steering, front_left_steering, rear_right_steering, rear_left_steering,
         front_right_wheel, front_left_wheel, rear_right_wheel, rear_left_wheel]
        """
        if self.control_mode != "direct":
            # Ignore direct_control messages if not in direct mode
            self.get_logger().debug("Received /direct_control but not in Direct mode. Ignored.")
            return

        if len(msg.data) != 8:
            self.get_logger().error(f"Received /direct_control message with {len(msg.data)} elements instead of 8.")
            return

        # Extract steering angles and wheel velocities
        steering_angles = msg.data[:4]
        wheel_velocities = msg.data[4:]

        # Define mapping from indices to joints
        # Order:
        # 0: front_right_steering
        # 1: front_left_steering
        # 2: rear_right_steering
        # 3: rear_left_steering
        # 4: front_right_wheel
        # 5: front_left_wheel
        # 6: rear_right_wheel
        # 7: rear_left_wheel

        mapping = {
            0: (JointKind.STEERING, JointEnd.FRONT, JointSide.RIGHT),
            1: (JointKind.STEERING, JointEnd.FRONT, JointSide.LEFT),
            2: (JointKind.STEERING, JointEnd.REAR, JointSide.RIGHT),
            3: (JointKind.STEERING, JointEnd.REAR, JointSide.LEFT),
            4: (JointKind.WHEEL, JointEnd.FRONT, JointSide.RIGHT),
            5: (JointKind.WHEEL, JointEnd.FRONT, JointSide.LEFT),
            6: (JointKind.WHEEL, JointEnd.REAR, JointSide.RIGHT),
            7: (JointKind.WHEEL, JointEnd.REAR, JointSide.LEFT),
        }

        # Publish steering angles
        for i in range(4):
            key = mapping[i]
            steering_angle_msg = Float64MultiArray()
            steering_angle_msg.data = [steering_angles[i]]  # radians
            self.joint_publisher[key].publish(steering_angle_msg)
            self.get_logger().debug(
                f"Direct Mode: Published steering angle to {joint_name_from_key(key)}: {math.degrees(steering_angles[i]):.2f} degrees"
            )

        # Publish wheel angular velocities
        for i in range(4, 8):
            key = mapping[i]
            wheel_vel = wheel_velocities[i - 4]
            # Invert the wheel angular velocity to ensure positive linear.x moves forward
            wheel_vel_inverted = -wheel_vel
            wheel_msg = Float64MultiArray()
            wheel_msg.data = [wheel_vel_inverted]
            self.joint_publisher[key].publish(wheel_msg)
            self.get_logger().debug(
                f"Direct Mode: Published wheel velocity to {joint_name_from_key(key)}: {wheel_vel_inverted:.2f} rad/s"
            )

    def stop(self):
        """
        set angular velocity of all 4 wheel positions to 0.0
        """
        msg: Float64MultiArray = Float64MultiArray()
        msg.data = [0.0]
        for end, side in joint_positions():
            key: JointKey = (JointKind.WHEEL, end, side)
            self.joint_publisher[key].publish(msg)


    def destroy_node(self):
        """
        Cleanup before shutting down the node.
        """
        super().destroy_node()


def main(args=None):
    rclpy.init(args=sys.argv)

    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down Controller node.")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
