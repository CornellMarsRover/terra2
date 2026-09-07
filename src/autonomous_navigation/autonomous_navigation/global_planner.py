#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from autonomous_navigation.planner_core import parse_planar_target


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')

        self.next_target_subscription = self.create_subscription(
            Float32MultiArray,
            '/autonomy/target/global',
            self.target_callback,
            10
        )
        self.waypoint_publisher = self.create_publisher(Float32MultiArray, '/autonomy/target/local', 10)

        self.curr_target = None
        self.publish_timer = self.create_timer(0.5, self.publish_waypoint)
        self.get_logger().info("Global Planner Node initialized")

    def target_callback(self, msg):
        try:
            self.curr_target, _ = parse_planar_target(msg.data)
        except ValueError as error:
            self.get_logger().error(f"Ignoring malformed target: {error}")
        
    def publish_waypoint(self):
        if self.curr_target is None:
            return
        waypoint = Float32MultiArray()
        waypoint.data = list(self.curr_target)
        self.waypoint_publisher.publish(waypoint)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
