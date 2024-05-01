import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

PI = np.pi

class PoseGeneratorNode(Node):
    def __init__(self):
        super().__init__('pose_generator_node')
        self.declare_parameter('unit_distance', 2.0)
        self.unit_distance = self.get_parameter('unit_distance').value
       
        # Initialize variables
        self.origin_init = False
        self.pose_id = 0
        self.previous_pose = PoseStamped()
        self.origin = PoseStamped()

        # Substitute for the behavior tree input/output
        self.has_arrived = False

        # Timer to mimic on_tick behavior
        self.timer = self.create_timer(1.0, self.on_tick)

    def generate_next_pose(self):
        circle = (self.pose_id // 4) + 1
        location = self.pose_id % 4
        new_pose = PoseStamped()
        new_pose.pose.position.x = self.unit_distance * circle * ((location + 1) % 2)
        new_pose.pose.position.y = self.unit_distance * circle * (location % 2)
        if location > 1:
            new_pose.pose.position.x = -new_pose.pose.position.x
            new_pose.pose.position.y = -new_pose.pose.position.y
        angle = 0
        if location == 0 and circle > 1:
            angle = math.atan2(circle - 1, circle)
        elif location != 0:
            angle = (45 + 90 * location) * (PI / 180)
        c = math.cos(angle * 0.5)
        s = math.sin(angle * 0.5)

        new_pose.pose.orientation.w = c
        new_pose.pose.orientation.z = s
        new_pose.header.frame_id = "map"

        new_pose.pose.position.x += self.origin.pose.position.x
        new_pose.pose.position.y += self.origin.pose.position.y
        return new_pose

    def on_tick(self):
        if not self.origin_init:
            # Simulate getting the initial goal input (origin)
            self.origin.pose.position.x = 0.0
            self.origin.pose.position.y = 0.0
            self.origin_init = True

        if self.has_arrived:
            self.previous_pose = self.generate_next_pose()
            self.pose_id += 1
            # Here you would publish or handle the new pose as needed
            self.get_logger().info(f'New Goal Pose: {self.previous_pose.pose.position.x}, {self.previous_pose.pose.position.y}')
       
        # Simulate the process of checking if the robot has arrived
        self.has_arrived = not self.has_arrived

def main(args=None):
    rclpy.init(args=args)
    pose_generator_node = PoseGeneratorNode()
    rclpy.spin(pose_generator_node)

    pose_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()