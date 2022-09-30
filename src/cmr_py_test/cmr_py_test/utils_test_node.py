import rclpy
from rclpy.node import Node
from std_msgs.msg._string import String


class PassthroughNode(Node):
    def __init__(self):
        super().__init__("utils_test_node")
        self.get_logger().info("The utils_test_node node.")
        self.sub = self.create_subscription(
            String,
            f"{self.get_namespace()}/{self.get_name()}/test_in",
            self._topic_callback,
            10,
        )
        self.get_logger().info(
            f"Listening on {self.get_namespace()}/{self.get_name()}/test_in"
        )
        self.pub = self.create_publisher(
            String, f"{self.get_namespace()}/{self.get_name()}/test_out", 10
        )
        self.get_logger().info(
            f"Publishing on {self.get_namespace()}/{self.get_name()}/test_out"
        )

    def _topic_callback(self, msg: String):
        self.get_logger().info(f"Received message: '{msg.data}'")
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PassthroughNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
