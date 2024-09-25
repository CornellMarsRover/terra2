import numpy as np
from scipy.optimize import minimize

class MPCController:
    def __init__(self, wheelbase=60.0, time_step=0.1, horizon=10):
        """
        Initialize the MPC Controller.
        - wheelbase: in pixels
        - time_step: in seconds
        - horizon: number of steps
        """
        self.wheelbase = wheelbase
        self.time_step = time_step
        self.horizon = horizon
        self.max_velocity = 100.0  # pixels per second (adjust as needed)
        self.max_steering_angle = np.radians(45)  # Max steering angle in radians
        self.obstacle_positions = []  # List of obstacle positions

    def set_obstacles(self, obstacles):
        """Set obstacle positions (list of tuples with x, y coordinates)."""
        self.obstacle_positions = obstacles

    def transition_function(self, state, velocity, steering_angle):
        """Compute the next state given the current state and action using bicycle model dynamics."""
        x, y, theta = state

        if abs(steering_angle) > 1e-5:
            turning_radius = self.wheelbase / np.tan(steering_angle)
            angular_velocity = velocity / turning_radius
            next_theta = self.normalize_angle(theta + angular_velocity * self.time_step)
        else:
            next_theta = self.normalize_angle(theta)

        next_x = x + velocity * np.cos(next_theta) * self.time_step
        next_y = y + velocity * np.sin(next_theta) * self.time_step

        return np.array([next_x, next_y, next_theta])

    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def objective_function(self, vars, current_state, goal):
        """Objective function to minimize."""
        velocities = vars[:self.horizon]
        steering_angles = vars[self.horizon:]
        state = np.array(current_state)
        cost = 0.0

        for t in range(self.horizon):
            # Compute the next state
            state = self.transition_function(state, velocities[t], steering_angles[t])

            # Compute the squared distance to the goal
            distance_to_goal = np.linalg.norm(state[:2] - goal)
            cost += distance_to_goal ** 2

            # Add control effort cost
            cost += 0.1 * velocities[t] ** 2 + 0.1 * steering_angles[t] ** 2

            # Penalize low velocities to encourage movement
            cost += 1.0 / (velocities[t] + 1e-5)  # Add a small epsilon to avoid division by zero

            # Add obstacle cost if any
            # ... existing obstacle cost code ...
        return cost


    def compute_optimal_action(self, current_state, goal):
        """Solve the optimization problem to compute the optimal action."""
        # Calculate direction towards the goal
        direction = np.arctan2(goal[1] - current_state[1], goal[0] - current_state[0])
        angle_diff = self.normalize_angle(direction - current_state[2])

        # Initial guess: velocities aiming towards the goal, steering angle towards the goal
        initial_velocity = min(
            self.max_velocity,
            np.linalg.norm(np.array(goal[:2]) - np.array(current_state[:2])) / (self.horizon * self.time_step)
        )
        initial_steering_angle = np.clip(angle_diff, -self.max_steering_angle, self.max_steering_angle)

        initial_guess = np.concatenate([
            np.full(self.horizon, initial_velocity),
            np.full(self.horizon, initial_steering_angle)
        ])

        # Define the bounds for each variable
        bounds = [(0, self.max_velocity)] * self.horizon + [(-self.max_steering_angle, self.max_steering_angle)] * self.horizon

        # Logging for debugging
        print(f"Direction to goal: {np.degrees(direction):.2f} degrees")
        print(f"Current heading: {np.degrees(current_state[2]):.2f} degrees")
        print(f"Angle difference: {np.degrees(angle_diff):.2f} degrees")
        print(f"Initial steering angle (degrees): {np.degrees(initial_steering_angle):.2f}")
        print(f"Initial velocity: {initial_velocity:.2f}")

        # Define the optimization problem
        result = minimize(
            self.objective_function,
            initial_guess,
            args=(current_state, goal),
            method='SLSQP',
            bounds=bounds,
            options={'disp': False, 'maxiter': 200}
        )

        if result.success:
            # Logging the optimizer's result
            print(f"Optimizer succeeded. Optimal cost: {result.fun:.2f}")
            print(f"Optimal velocities: {result.x[:self.horizon]}")
            print(f"Optimal steering angles (degrees): {np.degrees(result.x[self.horizon:])}")
            optimal_velocity = result.x[0]
            optimal_steering_angle = result.x[self.horizon]
            return optimal_velocity, optimal_steering_angle
        else:
            print(f"Optimization failed: {result.message}")
            return 0.0, 0.0
