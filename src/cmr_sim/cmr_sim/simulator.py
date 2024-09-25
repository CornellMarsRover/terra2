import pygame
import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from cmr_msgs.msg import AutonomyInfo  # Replace with your actual message import
from cmr_msgs.msg import ObstaclePosition, ObstaclePositions  # Import custom messages
import time  # For tracking time

# Constants for the rover dimensions and environment
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 900
WHITE = (255, 255, 255)
ROVER_COLOR = (0, 128, 255)
GOAL_COLOR = (0, 255, 0) 
ROVER_HEIGHT = 60
ROVER_WIDTH = 70
OBSTACLE_COLOR = (255, 0, 0)
OBSTACLE_WIDTH = 40
OBSTACLE_HEIGHT = 40
RANDOM_STATE = True


class Obstacle:
    def __init__(self, x, y, width, height):
        self.x = x  
        self.y = y 
        self.width = width
        self.height = height


class RoverSimNode(Node):
    def __init__(self):
        super().__init__('rover_simulation')
        self.autonomy_publisher = self.create_publisher(AutonomyInfo, '/rover_state', 10)
        self.obstacle_publisher = self.create_publisher(ObstaclePositions, '/obstacle_positions', 10)
        self.subscription = self.create_subscription(TwistStamped,'/drives_controller/cmd_vel', self.listener_callback, 10)
        
        # Initial Rover state
        self.x = float(SCREEN_WIDTH // 2)  # Start at center
        self.y = float(SCREEN_HEIGHT // 2)
        self.angle = 0.0  # In radians
        self.velocity = 0.0  # pixels/sec
        self.steering_angle = 0.0  # radians

        # Rover dimensions
        self.rover_width = ROVER_WIDTH
        self.rover_height = ROVER_HEIGHT
        self.wheelbase = 60.0

        # Initial goal point set to center
        self.goal = (float(SCREEN_WIDTH // 2), float(SCREEN_HEIGHT // 2)) 
        self.obstacles = [] 

        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Rover Simulation")

        # Publish initial rover state
        self.publish_state()

    def listener_callback(self, msg):
        """Callback to handle incoming TwistStamped messages"""
        self.velocity = msg.twist.linear.x
        self.steering_angle = msg.twist.angular.z  # Now represents steering angle

        # Update the last command received time
        self.last_cmd_time = time.time()
        self.update_rover_position()

    def publish_state(self):
        """Publish the rover's current state and goal as AutonomyInfo"""
        autonomy_info_msg = AutonomyInfo()
        autonomy_info_msg.x = self.x
        autonomy_info_msg.y = self.y
        autonomy_info_msg.theta = self.angle
        autonomy_info_msg.goal_x = self.goal[0] if self.goal else 0.0
        autonomy_info_msg.goal_y = self.goal[1] if self.goal else 0.0
        # Publish the AutonomyInfo message
        self.autonomy_publisher.publish(autonomy_info_msg)

    def publish_obstacle_info(self):
        """Publish obstacle positions to the MPC controller."""
        obstacle_msg = ObstaclePositions()
        obstacle_msg.positions = [
            ObstaclePosition(
                x=obs.x,
                y=obs.y,
                width=float(obs.width),
                height=float(obs.height)
            )
            for obs in self.obstacles
        ]
        self.obstacle_publisher.publish(obstacle_msg)

    def update_rover_position(self, delta_time = 0.1):
        """Update the rover's position and orientation based on velocity and steering angle using a bicycle model, 
        stochastically updates state when RANDOM_STATE is true for simulation of unknown conditions such as weather"""

        if abs(self.steering_angle) > 1e-5:
            turning_radius = self.wheelbase / math.tan(self.steering_angle)
            angular_velocity = self.velocity / turning_radius
            self.angle = self.normalize_angle(self.angle + angular_velocity * delta_time)
        else:
            # Move straight if steering angle is zero
            self.angle = self.normalize_angle(self.angle)
        
        # Update position
        self.x += self.velocity * math.cos(self.angle) * delta_time
        self.y += self.velocity * math.sin(self.angle) * delta_time

        # **Add Randomness to State Variables**
        if RANDOM_STATE and np.random.rand() < 0.05:
            # Define standard deviations for the noise
            position_noise_std = 0.5  # Adjust as needed (e.g., 0.5 pixels)
            angle_noise_std = np.radians(0.1)  # Adjust as needed (e.g., 1 degree in radians)

            self.x += np.random.normal(0, position_noise_std)
            self.y += np.random.normal(0, position_noise_std)
            self.angle += np.random.normal(0, angle_noise_std)
            self.angle = self.normalize_angle(self.angle)
        
        self.x = max(0, min(self.x, SCREEN_WIDTH))
        self.y = max(0, min(self.y, SCREEN_HEIGHT))

        # After updating the rover's state, publish autonomy info
        self.publish_state()

    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def draw_rover(self):
        """Draw the rover as a rectangle (box) on the screen with a direction indicator"""
        rover_surface = pygame.Surface((self.rover_width, self.rover_height), pygame.SRCALPHA)
        rover_surface.fill(ROVER_COLOR)

        indicator_length = self.rover_width // 2 
        indicator_color = (255, 0, 0)  

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
            pygame.draw.circle(self.screen, GOAL_COLOR, (int(self.goal[0]), int(self.goal[1])), 10)

    def draw_obstacles(self):
        """Draw obstacles on the screen as rectangles."""
        for obstacle in self.obstacles:
            rect = pygame.Rect(
                obstacle.x - obstacle.width / 2,
                obstacle.y - obstacle.height / 2,
                obstacle.width,
                obstacle.height
            )
            pygame.draw.rect(self.screen, OBSTACLE_COLOR, rect)


    def run(self):
        """Main simulation loop"""
        clock = pygame.time.Clock()
        
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return

                # Check for mouse clicks
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Left click to set the goal
                        self.goal = (float(event.pos[0]), float(event.pos[1]))  # Set the goal to the position of the mouse click
                        # After setting the goal, publish autonomy info
                        self.publish_state()
                    if event.button == 3:  # Right click to place an obstacle
                        obstacle_position = (float(event.pos[0]), float(event.pos[1]))
                        obstacle = Obstacle(
                            x=obstacle_position[0],
                            y=obstacle_position[1],
                            width=OBSTACLE_WIDTH,
                            height=OBSTACLE_HEIGHT
                        )
                        self.obstacles.append(obstacle)
                        self.publish_obstacle_info()


            self.screen.fill(WHITE)
            self.draw_goal()
            self.draw_obstacles()
            self.draw_rover()
            
            # Update the display
            pygame.display.flip()
            # Spin ROS for callbacks with no timeout to avoid blocking
            rclpy.spin_once(self, timeout_sec=0)

    def destroy_node(self):
        """Cleanup before shutting down"""
        pygame.quit()
        super().destroy_node()

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
