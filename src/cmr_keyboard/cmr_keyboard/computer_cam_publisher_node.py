# camera_publisher/camera_publisher/camera_publisher_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ComputerCameraPublisherNode(Node):
    def __init__(self):
        super().__init__('computer_camera_publisher_node')

        # Publisher for the image topic
        self.publisher_ = self.create_publisher(Image, '/camA/image_raw', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Open the built-in camera (usually device 0)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Cannot open camera")
            return

        # Set camera resolution (optional)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2400)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1800)

        # Create a timer to publish images at a fixed rate (e.g., 10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to capture image")
            return

        # Convert the image to ROS Image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Publish the image
        self.publisher_.publish(img_msg)
        self.get_logger().debug("Published an image")

    def __del__(self):
        # Release the camera when the node is destroyed
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = ComputerCameraPublisherNode()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.get_logger().info("Shutting down camera publisher...")
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
