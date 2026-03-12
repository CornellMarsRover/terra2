"""
Lightweight camera publisher using cv2.VideoCapture.
Works with any USB camera (including ZED as a standard UVC device)
without requiring the ZED SDK (pyzed).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('image_topic', '/zed/image_left')

        camera_index = self.get_parameter('camera_index').value
        fps = self.get_parameter('fps').value
        image_topic = self.get_parameter('image_topic').value

        self.publisher = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open camera index {camera_index}')
            raise RuntimeError(f'Camera {camera_index} failed to open')

        # Drain initial frames (some cameras drop the first few)
        for _ in range(5):
            self.cap.read()
            time.sleep(0.1)

        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(
            f'Camera {camera_index} opened, publishing to {image_topic} at {fps} FPS')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
