import rclpy
from rclpy.node import Node
from std_msgs.msg._string import String
import rclpy.action as action
from cmr_msgs.action._test_target_position import TestTargetPosition


class PassthroughNode(Node):
    """
    This node simply provides a topic and action server for testing purposes.

    It echos back every message on the topic /<ns>/<name>/test_in on
    the topic /<ns>/<name>/test_out.

    It echos back every target position goal received on the action server at
    /<ns>/<name>/test_action and responds on the action server
    /<ns>/<name>/test_return.
    """
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
        self.get_logger().info(
            f"Starting action server on {self.get_namespace()}/{self.get_name()}/test_action"
        )
        self.server = action.ActionServer(
            self,
            TestTargetPosition,
            f"{self.get_namespace()}/{self.get_name()}/test_action",
            self._action_server_callback,
        )
        self.client = action.ActionClient(
            self,
            TestTargetPosition,
            f"{self.get_namespace()}/{self.get_name()}/test_return",
        )

    def _topic_callback(self, msg: String):
        self.get_logger().info(f"Received message: '{msg.data}'")
        self.pub.publish(msg)

    def _action_server_callback(self, goal_handle: action.server.ServerGoalHandle):
        self.get_logger().info("Received goal")
        self.client.send_goal_async(goal_handle.request)
        goal_handle.succeed()
        return TestTargetPosition.Result(success=True)


def main():
    rclpy.init()
    node = PassthroughNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
