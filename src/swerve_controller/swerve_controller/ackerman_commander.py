#!/usr/bin/env python3

"""
AckermanCommander ROS2 Node

This node implements an Ackermann steering controller for a robot. It listens for
linear velocity commands on the `/cmd_vel/ackerman` topic and desired/current yaw
angles on the `/angles` topic. Using a PID controller, it adjusts the steering angles
of the front two wheels to correct any angular errors, ensuring accurate navigation.
"""

import sys
import math
from typing import Dict, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Float64MultiArray

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
from rclpy.qos import qos_profile_sensor_data

class PIDController:
    """
    Simple PID Controller
    """

    def __init__(self, Kp: float, Ki: float, Kd: float, output_limits: Tuple[float, float] = (None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.output_limits = output_limits
        self.last_time = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = None

    def compute(self, error: float, current_time) -> float:
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
        derivative = (error - self.previous_error) / delta_time if delta_time > 0 else 0.0
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

class AckermanCommander(Node):
    """
    AckermanCommander implements the kinematics of an Ackermann drive robot.
    It listens for linear velocity commands and desired/current yaw angles,
    computes the necessary steering angles for the front wheels using a PID controller,
    and publishes wheel velocities and steering commands.
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
        super().__init__("ackerman_commander")

        # Declare and get node parameters
        self.declare_parameter(name=self.wheelbase_descriptor.name, descriptor=self.wheelbase_descriptor)
        self.declare_parameter(name=self.wheel_track_descriptor.name, descriptor=self.wheel_track_descriptor)
        self.declare_parameter(name=self.wheel_radius_descriptor.name, descriptor=self.wheel_radius_descriptor)

        self.wheelbase = self.get_parameter("wheelbase").get_parameter_value().double_value
        self.wheel_track = self.get_parameter("wheel_track").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value

        self.get_logger().info(f"Wheelbase: {self.wheelbase} m")
        self.get_logger().info(f"Wheel Track: {self.wheel_track} m")
        self.get_logger().info(f"Wheel Radius: {self.wheel_radius} m")

        # Set up a publisher for each of the 8 swerve joints using key iterator
        self.joint_publisher: Dict[JointKey, Publisher] = {}
        for key in joints():
            topic: str = f"/{joint_name_from_key(key)}/commands"
            self.joint_publisher[key] = self.create_publisher(
                Float64MultiArray, topic, 10
            )
            self.get_logger().debug(f"Publishing to topic: {topic}")

        # Register /cmd_vel/ackerman subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel/ackerman", self.cmd_vel_callback, 10
        )
        self.get_logger().info("Subscribed to /cmd_vel/ackerman")

        # Register /angles subscriber
        self.angles_sub = self.create_subscription(
            Twist, "/angles", self.angles_callback, qos_profile_sensor_data
        )
        self.get_logger().info("Subscribed to /angles")

        # Initialize PID controller for yaw correction
        # PID parameters can be tuned as needed
        self.pid = PIDController(Kp=0.5, Ki=0.0, Kd=0.1, output_limits=(-math.radians(30), math.radians(30)))
        self.get_logger().info("Initialized PID controller with Kp=0.5, Ki=0.0, Kd=0.1")

        # Variables to store desired and current yaw in radians
        self.desired_yaw = 0.0  # radians
        self.current_yaw = 0.0  # radians

        # Timer for PID updates at 30 Hz
        self.pid_timer = self.create_timer(1.0 / 30.0, self.pid_update_callback)
        self.get_logger().info("PID update timer initialized at 30 Hz")

        # Store the last commanded linear velocity
        self.current_linear_x = 0.0  # m/s

    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Callback for /cmd_vel/ackerman subscriber.
        Extracts the linear.x component and computes wheel velocities.
        """
        self.current_linear_x = msg.linear.x
        self.get_logger().debug(f"Received /cmd_vel/ackerman: linear.x = {self.current_linear_x} m/s")

        # Compute wheel angular velocities based on linear velocity
        wheel_angular_velocity = self.current_linear_x / self.wheel_radius  # rad/s

        # Create wheel velocity message
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [wheel_angular_velocity]

        # Publish to all wheel velocity joints
        for key in joints():
            if key[0] == JointKind.WHEEL:
                self.joint_publisher[key].publish(wheel_msg)
                self.get_logger().debug(f"Published wheel velocity to {joint_name_from_key(key)}: {wheel_angular_velocity} rad/s")

    def angles_callback(self, msg: Twist) -> None:
        """
        Callback for /angles subscriber.
        Extracts desired and current yaw angles from the Twist message.
        Assumes:
            - msg.angular.x is desired yaw in degrees
            - msg.angular.y is current yaw in degrees
        """
        desired_yaw_deg = msg.angular.x
        current_yaw_deg = msg.angular.y

        # Convert degrees to radians
        self.desired_yaw = math.radians(desired_yaw_deg)
        self.current_yaw = math.radians(current_yaw_deg)

        self.get_logger().debug(f"Received /angles: desired_yaw = {desired_yaw_deg} deg, current_yaw = {current_yaw_deg} deg")

    def pid_update_callback(self) -> None:
        """
        Timer callback to update steering angles based on PID controller output.
        Adjusts the steering angles of the front two wheels to correct angular error.
        """
        # Compute angular error
        error = self.desired_yaw - self.current_yaw
        # Normalize error to [-pi, pi]
        error = (error + math.pi) % (2 * math.pi) - math.pi

        self.get_logger().debug(f"PID Update: Error = {math.degrees(error)} degrees")

        # Get current time
        current_time = self.get_clock().now()

        # Compute PID controller output
        control_effort = self.pid.compute(error, current_time)

        self.get_logger().debug(f"PID Control Effort: {math.degrees(control_effort)} degrees")

        # Apply control effort to front steering angles
        # Assuming control_effort is the desired steering angle in radians
        # Front left and front right wheels are adjusted
        for side in [JointSide.LEFT, JointSide.RIGHT]:
            key = (JointKind.STEERING, JointEnd.FRONT, side)
            steering_angle_msg = Float64MultiArray()
            steering_angle_msg.data = [control_effort]  # radians
            self.joint_publisher[key].publish(steering_angle_msg)
            self.get_logger().debug(f"Published steering angle to {joint_name_from_key(key)}: {math.degrees(control_effort)} degrees")

        # Optionally, set rear steering angles to zero or keep them unchanged
        for side in [JointSide.LEFT, JointSide.RIGHT]:
            key = (JointKind.STEERING, JointEnd.REAR, side)
            steering_angle_msg = Float64MultiArray()
            steering_angle_msg.data = [0.0]  # radians
            self.joint_publisher[key].publish(steering_angle_msg)
            self.get_logger().debug(f"Published steering angle to {joint_name_from_key(key)}: 0.0 degrees")

    def stop(self):
        """
        Stop the robot by setting all wheel velocities to 0.
        """
        self.get_logger().info("Stopping robot: Publishing zero velocities to all wheels")
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0]
        for key in joints():
            if key[0] == JointKind.WHEEL:
                self.joint_publisher[key].publish(stop_msg)

    def destroy_node(self):
        """
        Cleanup before shutting down the node.
        """
        self.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    ackerman_commander = AckermanCommander()

    try:
        rclpy.spin(ackerman_commander)
    except KeyboardInterrupt:
        ackerman_commander.get_logger().info("Shutting down AckermanCommander node.")
    finally:
        ackerman_commander.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
