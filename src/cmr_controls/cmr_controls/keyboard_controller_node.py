import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from pynput import keyboard
import threading

class KeyToTwistNode(Node):
    def __init__(self):
        super().__init__('key_to_twist_node')

        # Publisher for /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for /arm/joint_increment
        self.increment_publisher_ = self.create_publisher(Float32MultiArray, '/arm/joint_increment', 10)

        # Twist message to store current velocities
        self.twist = Twist()

        # Lock for thread safety
        self.lock = threading.Lock()

        # Dictionary to track currently pressed keys
        self.pressed_keys = set()

        # Key mappings for Twist messages
        self.key_mappings = {
            'q': ('linear.x', 0.003),
            'w': ('linear.y', 0.006),
            'e': ('linear.z', 0.003),
            'a': ('linear.x', -0.003),
            's': ('linear.y', -0.006),
            'd': ('linear.z', -0.003),
            'i': ('angular.x', 0.01),
            'o': ('angular.y', 0.01),
            'p': ('angular.z', 0.01),
            'j': ('angular.x', -0.01),
            'k': ('angular.y', -0.01),
            'l': ('angular.z', -0.01),
        }

        # Key mappings for joint increments
        self.increment_key_mappings = {
            'z': [-0.03, 0.0, 0.0, 0.0, 0.0, 0.0],
            'x': [0.03, 0.0, 0.0, 0.0, 0.0, 0.0],
        }

        # Start listening for key presses
        self.get_logger().info("Starting Key Listener...")
        self.start_key_listener()

        # Start a timer to continuously publish Twist messages
        self.timer = self.create_timer(0.1, self.publish_twist)

    def start_key_listener(self):
        """
        Starts the key listener to detect key presses and releases.
        """
        def on_press(key):
            try:
                # Add key to pressed_keys if it's in the key mappings
                if hasattr(key, 'char') and key.char:
                    with self.lock:
                        if key.char in self.key_mappings:
                            self.pressed_keys.add(key.char)
                        elif key.char in self.increment_key_mappings:
                            self.publish_joint_increment(self.increment_key_mappings[key.char])
            except Exception as e:
                self.get_logger().error(f"Error in key press: {e}")

        def on_release(key):
            try:
                # Remove key from pressed_keys when it's released
                if hasattr(key, 'char') and key.char:
                    with self.lock:
                        if key.char in self.key_mappings:
                            self.pressed_keys.discard(key.char)
            except Exception as e:
                self.get_logger().error(f"Error in key release: {e}")

        # Start the keyboard listener
        self.listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        self.listener.start()

    def publish_joint_increment(self, increment_values):
        """
        Publishes a Float32MultiArray to the /arm/joint_increment topic.

        Parameters:
            increment_values: A list of floats representing joint increments.
        """
        msg = Float32MultiArray()
        msg.data = increment_values
        self.increment_publisher_.publish(msg)
        self.get_logger().info(f"Published Joint Increment: {msg.data}")

    def publish_twist(self):
        """
        Publishes the current Twist message based on the pressed keys.
        """
        with self.lock:
            # Reset Twist message to zero
            self.twist = Twist()

            # Update Twist message based on currently pressed keys
            for key in self.pressed_keys:
                if key in self.key_mappings:
                    attr, value = self.key_mappings[key]
                    # Split the attribute into part and axis
                    part, axis = attr.split('.')
                    # Update the corresponding field in Twist
                    setattr(getattr(self.twist, part), axis, getattr(getattr(self.twist, part), axis) + value)

            # Publish the updated Twist message
            self.publisher_.publish(self.twist)

            # Log the current Twist message
            self.get_logger().info(f"Published Twist: linear=({self.twist.linear.x}, {self.twist.linear.y}, {self.twist.linear.z}), angular=({self.twist.angular.x}, {self.twist.angular.y}, {self.twist.angular.z})")


def main(args=None):
    rclpy.init(args=args)

    node = KeyToTwistNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down key listener...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
