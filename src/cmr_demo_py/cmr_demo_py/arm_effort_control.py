import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class ArmEffortControl(Node):
    """A temporary test node that just publishes a constant effort of 10.0 to
    the arm joints at a rate of 2Hz. This should be removed once joystick control
    is implemented."""

    def __init__(self):
        super().__init__("arm_effort_control")
        self.publisher_ = self.create_publisher(
            Float64MultiArray, "/arm_controller/commands", 10
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def timer_cb(self):
        msg = Float64MultiArray()
        msg.data = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ArmEffortControl()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
