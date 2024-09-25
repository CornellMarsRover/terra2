import numpy as np
from scipy.optimize import minimize

class MPCController:
    def __init__(self, wheelbase=60.0, time_step=0.1, horizon=5, rover_radius = 20):
        """
        Initialize the MPC Controller.
        - wheelbase: in pixels
        - time_step: in seconds
        - horizon: number of steps
        """
        self.wheelbase = wheelbase
        self.time_step = time_step
        self.horizon = horizon #number of time_steps, total horizon duration = horizon * time_step
        self.max_velocity = 100.0  # pixels per second (adjust as needed)
        self.max_steering_angle = np.radians(80)  # Max steering angle in radians
        self.obstacles = []  # List of obstacle positions
        self.rover_radius = rover_radius

    def set_obstacles(self, obstacles):
        """Set obstacle positions (list of tuples with x, y coordinates)."""
        self.obstacles = obstacles

    def transition_function(self, state, velocity, steering_angle):
        """Compute the next state given the current state and action using bicycle model dynamics."""
        x, y, theta = state

        if abs(steering_angle) < 1e-4:  # Threshold to consider as zero
            # Straight-line motion
            next_theta = theta  # Heading remains the same
            next_x = x + velocity * np.cos(next_theta) * self.time_step
            next_y = y + velocity * np.sin(next_theta) * self.time_step
        else:
            # Bicycle model for turning motion
            turning_radius = self.wheelbase / np.tan(steering_angle)
            angular_velocity = velocity / turning_radius
            next_theta = self.normalize_angle(theta + angular_velocity * self.time_step)
            next_x = x + turning_radius * (np.sin(next_theta) - np.sin(theta))
            next_y = y - turning_radius * (np.cos(next_theta) - np.cos(theta))
        
        return np.array([next_x, next_y, next_theta])

    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        angle = np.fmod(angle + np.pi, 2 * np.pi)
        if angle < 0:
            angle += 2 * np.pi
        return angle - np.pi

    def objective_function(self, vars, current_state, goal):
        velocities = vars[:self.horizon]
        steering_angles = vars[self.horizon:]
        state = np.array(current_state)
        cost = 0.0

        # Define weights
        w_goal = 100.0
        w_velocity = 1.0
        w_steering = 1.0
        w_smoothness = 100.0
        w_obstacle = 1000.0

        prev_velocity = velocities[0]
        prev_steering = steering_angles[0]

        for t in range(self.horizon):
            # Compute the next state
            state = self.transition_function(state, velocities[t], steering_angles[t])

            # Weighted distance to goal
            distance_to_goal = np.linalg.norm(state[:2] - goal)
            cost += w_goal * distance_to_goal ** 2

            # Penalize high velocities and steering angles
            cost += w_velocity * velocities[t] ** 2
            cost += w_steering * steering_angles[t] ** 2

            if t > 0:
                # Penalize changes in control inputs (smoothness)
                cost += w_smoothness * (velocities[t] - prev_velocity) ** 2
                cost += w_smoothness * (steering_angles[t] - prev_steering) ** 2

            prev_velocity = velocities[t]
            prev_steering = steering_angles[t]

            # Obstacle avoidance cost
            for obs in self.obstacles:
                distance_to_obstacle = self.point_to_rectangle_distance(state[0], state[1], obs)
                if distance_to_obstacle < 0:
                    # Inside the obstacle
                    cost += w_obstacle * (-distance_to_obstacle) ** 2
                elif distance_to_obstacle < 10:
                    # Near the obstacle
                    cost += w_obstacle * (50 - distance_to_obstacle) ** 2

        return cost

    
    def point_to_rectangle_distance(self, px, py, rect):
        """Calculate the horizontal and vertical distances to obstacle"""
        dx = max(rect.x - rect.width / 2 - (px - self.rover_radius), 0, (px + self.rover_radius) - (rect.x + rect.width / 2))
        dy = max(rect.y - rect.height / 2 - (py - self.rover_radius), 0, (py + self.rover_radius) - (rect.y + rect.height / 2))
        if dx == 0 and dy == 0:
            # The rover overlaps with the obstacle
            return -min(rect.width, rect.height) / 2
        else:
            return np.hypot(dx, dy) - self.rover_radius


    def compute_optimal_action(self, current_state, goal):
        """Solve the optimization problem to compute the optimal action."""
        try:
            # Calculate direction towards the goal
            direction = np.arctan2(goal[1] - current_state[1], goal[0] - current_state[0])
            angle_diff = self.normalize_angle(direction - current_state[2])

            # Limit angle_diff to within max_steering_angle
            angle_diff = np.clip(angle_diff, -self.max_steering_angle, self.max_steering_angle)

            # Initial guess for steering angle
            initial_steering_angle = angle_diff

            # Initial velocity
            distance = np.linalg.norm(np.array(goal[:2]) - np.array(current_state[:2]))
            initial_velocity = min(
                self.max_velocity,
                distance / (self.horizon * self.time_step)
            )

            initial_guess = np.concatenate([
                np.full(self.horizon, initial_velocity),
                np.full(self.horizon, initial_steering_angle)
            ])

            # Define the bounds for each variable
            bounds = (
                [(0, self.max_velocity)] * self.horizon +
                [(-self.max_steering_angle, self.max_steering_angle)] * self.horizon
            )

            # Define the optimization problem
            result = minimize(
                self.objective_function,
                initial_guess,
                args=(current_state, goal),
                method='SLSQP',
                bounds=bounds,
                options={'disp': False, 'maxiter': 500}
            )

            if result.success:
                optimal_velocity = result.x[0]  # First velocity command
                optimal_steering_angle = result.x[self.horizon]  # First steering angle command
                return optimal_velocity, optimal_steering_angle
            else:
                print(f"Optimization failed: {result.message}")
                return 0.0, 0.0  # Stop if optimization fails

        except Exception as e:
            print(f"Exception in compute_optimal_action: {e}")
            # Optionally, you can print the traceback for more details
            import traceback
            traceback.print_exc()
            return 0.0, 0.0

