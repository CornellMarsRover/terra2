import sys
from typing import Dict

import numpy as np
import rclpy
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

from cmr_msgs.msg import AutonomyDrive  # Make sure this message type is defined and accessible


class CompatibleCommander(Node):
    """
    Node that listens to '/autonomy_drive' topic with AutonomyDrive messages,
    and sets the wheel steering angles and velocities accordingly.
    """

    def __init__(self):
        super().__init__('autonomy_swerve_commander')

        # Set up a publisher for each of the 8 swerve joints
        self.joint_publisher: Dict[JointKey, Publisher] = {}
        for key in joints():
            topic: str = f"/{joint_name_from_key(key)}/commands"
            self.joint_publisher[key] = self.create_publisher(Float64MultiArray, topic, 10)

        # Register '/autonomy_drive' subscriber
        self.autonomy_drive_sub = self.create_subscription(
            AutonomyDrive, '/autonomy_drive', self.autonomy_drive_callback, 10
        )

        self.wheel_radius = 0.1397

    def autonomy_drive_callback(self, msg: AutonomyDrive) -> None:
        """
        Callback function to process AutonomyDrive messages.
        """
        vel = msg.vel
        fl_angle = msg.fl_angle
        fr_angle = msg.fr_angle
        bl_angle = msg.bl_angle
        br_angle = msg.br_angle

        # Map the angles to the corresponding wheel positions
        angle_map = {
            (JointEnd.FRONT, JointSide.LEFT): fl_angle,
            (JointEnd.FRONT, JointSide.RIGHT): fr_angle,
            (JointEnd.REAR, JointSide.LEFT): bl_angle,
            (JointEnd.REAR, JointSide.RIGHT): br_angle,
        }

        for end, side in joint_positions():
            # Get the steering angle for this wheel in degrees
            steering_angle = angle_map[(end, side)]
            rot_dir = 1.0

            # Limit steering angle to +/- 90 degrees
            if steering_angle > 90.0:
                steering_angle -= 180.0
                rot_dir *= -1.0
            elif steering_angle < -90.0:
                steering_angle += 180.0
                rot_dir *= -1.0

            # Convert steering angle to radians
            steering_angle_rad = np.deg2rad(steering_angle)

            # Calculate wheel angular velocity
            wheel_angular_velocity = rot_dir * vel / self.wheel_radius

            # Publish steering angle and wheel velocity
            for kind, value in (
                (JointKind.STEERING, steering_angle_rad),
                (JointKind.WHEEL, wheel_angular_velocity),
            ):
                joint_msg = Float64MultiArray(data=[value])
                key: JointKey = (kind, end, side)
                self.joint_publisher[key].publish(joint_msg)


def main():
    rclpy.init(args=sys.argv)

    try:
        node = CompatibleCommander()
    except KeyboardInterrupt:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
