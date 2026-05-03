#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SimGoalPublisher(Node):
    def __init__(self) -> None:
        super().__init__("sim_goal_publisher")

        self.declare_parameter("goal_north", 6.0)
        self.declare_parameter("goal_west", 0.0)
        self.declare_parameter("publish_period_s", 0.5)

        self.goal_north = float(self.get_parameter("goal_north").value)
        self.goal_west = float(self.get_parameter("goal_west").value)

        self.publisher = self.create_publisher(Float32MultiArray, "/autonomy/target/global", 10)
        period = float(self.get_parameter("publish_period_s").value)
        self.timer = self.create_timer(period, self.publish_goal)
        self.get_logger().info(
            f"Publishing fixed autonomy target ({self.goal_north:.2f}, {self.goal_west:.2f})"
        )

    def publish_goal(self) -> None:
        msg = Float32MultiArray()
        msg.data = [self.goal_north, self.goal_west]
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
