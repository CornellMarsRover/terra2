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
from cmr_msgs.msg import AutonomyDrive

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

        # Autonomy Drive Publisher
        self.drive_publisher = self.create_publisher(AutonomyDrive, '/autonomy_move', 10)

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


    def cmd_vel_callback(self, msg: Twist) -> None:
        """
        Callback for /cmd_vel subscriber.
        """
        # ackerman mode: Only use linear.x component
        self.current_linear_x = msg.linear.x
        self.get_logger().debug(
            f"ackerman Mode - Received /cmd_vel: linear.x = {self.current_linear_x} m/s"
        )


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

        drive_msg = AutonomyDrive()
        drive_msg.vel = self.current_linear_x
        drive_msg.fl_angle = control_effort
        drive_msg.fr_angle = control_effort
        drive_msg.bl_angle = 0.0
        drive_msg.br_angle = 0.0

        self.drive_publisher.publish(drive_msg)



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
