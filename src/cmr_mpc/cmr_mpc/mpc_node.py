import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import AutonomyInfo  # Replace with your actual message import
from cmr_msgs.msg import ObstaclePosition, ObstaclePositions  # Import custom messages
from .mpc_controller import MPCController  # Adjust the import as necessary
import numpy as np

ROVER_RADIUS = 20

class MPCNode(Node):
    """
    This node subscribes to the /autonomy_information topic for rover state and goal information,
    and publishes the rover's next action to the /drives_controller/cmd_vel topic.
    """
    def __init__(self):
        super().__init__('mpc_node')
        self.publisher_ = self.create_publisher(TwistStamped, '/drives_controller/cmd_vel', 10)
        self.mpc_controller = MPCController()

        # Create a subscriber to receive the current state and goal from /autonomy_information
        self.subscription = self.create_subscription(
            AutonomyInfo,
            '/rover_state',
            self.autonomy_info_callback,
            10)

        # Create a subscriber for obstacle positions
        self.obstacle_subscription = self.create_subscription(
            ObstaclePositions,
            '/obstacle_positions',
            self.obstacle_callback,
            10)

        # Initialize current state, goal, and obstacles
        self.current_state = None
        self.goal = None
        self.obstacle_positions = []

    def autonomy_info_callback(self, msg):
        """Callback to receive rover's current state and goal from /autonomy_information."""
        self.current_state = (msg.x, msg.y, msg.theta)
        self.goal = (msg.goal_x, msg.goal_y)
        self.publish_mpc_commands()

        # Optional: Adjust logging level or frequency
        # self.get_logger().debug(f'Received autonomy information: state={self.current_state}, goal={self.goal}')

    def obstacle_callback(self, msg):
        """Callback to receive obstacle positions."""
        self.obstacles = [
            ObstaclePosition(
                x=obs_msg.x,
                y=obs_msg.y,
                width=obs_msg.width,
                height=obs_msg.height
            )
            for obs_msg in msg.positions
        ]
        self.mpc_controller.set_obstacles(self.obstacles)

    def publish_mpc_commands(self):
        """Call the MPC controller to compute the optimal action and publish the command."""
        if self.current_state is None or self.goal is None:
            self.get_logger().warn('Current state or goal not set yet.')
            return

        # Check if the rover is close enough to the goal
        distance_to_goal = np.linalg.norm(np.array(self.current_state[:2]) - np.array(self.goal))
        self.get_logger().info("Distance to Goal = " + str(distance_to_goal))
        if distance_to_goal < 50.0:  # Threshold in pixels
            # Stop the rover
            msg = TwistStamped()
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info('Rover reached the goal. Stopping.')
            return

        try:
            # Call the MPC controller to compute the optimal action (velocity and steering angle)
            linear_velocity, steering_angle = self.mpc_controller.compute_optimal_action(self.current_state, self.goal)
        except ValueError as e:
            self.get_logger().error(f'MPC Optimization failed: {e}')
            linear_velocity, steering_angle = 0.0, 0.0

        # Create a TwistStamped message
        msg = TwistStamped()
        msg.twist.linear.x = linear_velocity  # Forward velocity in pixels/sec
        msg.twist.angular.z = steering_angle  # Steering angle in radians

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published linear velocity: {linear_velocity:.2f}, steering angle: {steering_angle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    mpc_node = MPCNode()

    try:
        rclpy.spin(mpc_node)
    except KeyboardInterrupt:
        pass
    finally:
        mpc_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
