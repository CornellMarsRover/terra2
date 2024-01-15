import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/drives_controller/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received cmd_vel:\n' + str(msg))
        # If you specifically need to print at most once every second,
        # you'll need additional logic to handle that timing.

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
