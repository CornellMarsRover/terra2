import pygame
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import AutonomyInfo  # Replace with your actual message import

# Constants for the rover dimensions and environment
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
WHITE = (255, 255, 255)
ROVER_COLOR = (0, 128, 255)
GOAL_COLOR = (0, 255, 0)  # Green for the goal

class RoverSimNode(Node):
    def __init__(self):
        super().__init__('rover_simulation')
        
        # Create a publisher for AutonomyInfo to send rover state and goal
        self.autonomy_publisher = self.create_publisher(AutonomyInfo, '/autonomy_information', 10)

        # Create a subscriber to receive velocity and steering angle commands
        self.subscription = self.create_subscription(
            TwistStamped,
            '/drives_controller/cmd_vel',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning
        
        # Rover state
        self.x = float(SCREEN_WIDTH // 2)  # Start at center
        self.y = float(SCREEN_HEIGHT // 2)
        self.angle = 0.0  # In radians
        self.velocity = 0.0  # pixels/sec
        self.steering_angle = 0.0  # radians

        # Tunable rover dimensions
        self.rover_width = 60
        self.rover_height = 40
        self.wheelbase = 60.0  # Wheelbase in pixels (consistent with MPCController)

        # Goal point
        self.goal = (float(SCREEN_WIDTH // 2), float(SCREEN_HEIGHT // 2))  # Initial goal set to center

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Rover Simulation")

        # Publish initial autonomy information
        self.publish_autonomy_info()

    def listener_callback(self, msg):
        """Callback to handle incoming TwistStamped messages"""
        self.velocity = msg.twist.linear.x
        self.steering_angle = msg.twist.angular.z  # Now represents steering angle

    def publish_autonomy_info(self):
        """Publish the rover's current state and goal as AutonomyInfo"""
        autonomy_info_msg = AutonomyInfo()
        autonomy_info_msg.x = self.x
        autonomy_info_msg.y = self.y
        autonomy_info_msg.theta = self.angle
        autonomy_info_msg.goal_x = self.goal[0] if self.goal else 0.0
        autonomy_info_msg.goal_y = self.goal[1] if self.goal else 0.0

        # Publish the AutonomyInfo message
        self.autonomy_publisher.publish(autonomy_info_msg)
        # Optional: Adjust logging level or frequency if needed
        # self.get_logger().debug(f'Published AutonomyInfo: state=({self.x:.2f}, {self.y:.2f}, {self.angle:.2f}), goal={self.goal}')

    def update_rover_position(self, delta_time):
        """Update the rover's position and orientation based on velocity and steering angle"""
        if abs(self.steering_angle) > 1e-5:
            # Calculate the change in heading angle using the bicycle model
            turning_radius = self.wheelbase / math.tan(self.steering_angle)
            angular_velocity = self.velocity / turning_radius
            self.angle = self.normalize_angle(self.angle + angular_velocity * delta_time)
        else:
            # Move straight if steering angle is zero
            self.angle = self.normalize_angle(self.angle)
        
        # Update position
        self.x += self.velocity * math.cos(self.angle) * delta_time
        self.y += self.velocity * math.sin(self.angle) * delta_time
        
        # Keep the rover within screen bounds
        self.x = max(0, min(self.x, SCREEN_WIDTH))
        self.y = max(0, min(self.y, SCREEN_HEIGHT))

        # After updating the rover's state, publish autonomy info
        self.publish_autonomy_info()

    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def draw_rover(self):
        """Draw the rover as a rectangle (box) on the screen with a direction indicator"""
        # Create the surface for the rover
        rover_surface = pygame.Surface((self.rover_width, self.rover_height), pygame.SRCALPHA)
        rover_surface.fill(ROVER_COLOR)

        # Draw an indicator on the front of the rover (a red line)
        indicator_length = self.rover_width // 2  # Length of the indicator
        indicator_color = (255, 0, 0)  # Red color for the direction indicator

        # Draw a small red line from the center of the rover to indicate the front
        pygame.draw.line(rover_surface, indicator_color, 
                         (self.rover_width // 2, self.rover_height // 2),  # Starting from center of the rover
                         (self.rover_width // 2 + indicator_length, self.rover_height // 2),  # End point in front of the rover
                         5)  # Line thickness

        # Rotate the rover surface according to the rover's angle
        rotated_surface = pygame.transform.rotate(rover_surface, -math.degrees(self.angle))

        # Get the new rect after rotation to position it correctly
        rect = rotated_surface.get_rect(center=(self.x, self.y))

        # Blit (draw) the rotated surface onto the screen
        self.screen.blit(rotated_surface, rect.topleft)

    def draw_goal(self):
        """Draw the goal point on the screen if it is set"""
        if self.goal:
            pygame.draw.circle(self.screen, GOAL_COLOR, (int(self.goal[0]), int(self.goal[1])), 10)  # Draw a small green circle at the goal

    def run(self):
        """Main simulation loop"""
        clock = pygame.time.Clock()
        
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

                # Check for mouse clicks to set the goal
                if event.type == pygame.MOUSEBUTTONDOWN:
                    self.goal = (float(event.pos[0]), float(event.pos[1]))  # Set the goal to the position of the mouse click

                    # After setting the goal, publish autonomy info
                    self.publish_autonomy_info()

            # Clear the screen
            self.screen.fill(WHITE)
            
            # Calculate delta_time
            delta_time = clock.tick(60) / 1000.0  # Run at 60 FPS, delta_time in seconds

            # Update the rover's position
            self.update_rover_position(delta_time)
            
            # Draw the goal if it has been set
            self.draw_goal()

            # Draw the rover
            self.draw_rover()
            
            # Update the display
            pygame.display.flip()
            
            # Spin ROS for callbacks with no timeout to avoid blocking
            rclpy.spin_once(self, timeout_sec=0)

def main(args=None):
    rclpy.init(args=args)
    node = RoverSimNode()
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
