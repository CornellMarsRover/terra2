#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

import cv2
from cv_bridge import CvBridge

import threading
import time


class MultiCameraImagePublisher(Node):
    def __init__(self, camera_ids, resolution=(640, 480), image_format='bgr8', publish_rate=30):
        """
        camera_ids: list of integers corresponding to /dev/videoX devices.
        resolution: tuple (width, height)
        image_format: OpenCV image encoding for ROS (e.g., 'bgr8', 'rgb8')
        publish_rate: publishing rate in Hz for each camera
        """
        super().__init__('multi_camera_image_publisher')
        self.get_logger().info("Initializing Multi-Camera Image Publisher...")

        self.camera_ids = camera_ids
        self.resolution = resolution
        self.image_format = image_format
        self.publish_rate = publish_rate

        self.bridge = CvBridge()
        self.cam_publishers = {}
        self.capture_threads = {}
        self.running = True

        for cam_id in camera_ids:
            topic_name = f"/camera_{cam_id}/image_raw"
            pub = self.create_publisher(Image, topic_name, 10)
            self.cam_publishers[cam_id] = pub
            thread = threading.Thread(target=self._capture_loop, args=(cam_id,), daemon=True)
            thread.start()
            self.capture_threads[cam_id] = thread

    def _capture_loop(self, cam_id):
        cap = cv2.VideoCapture(cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        cap.set(cv2.CAP_PROP_FPS, self.publish_rate)

        if not cap.isOpened():
            self.get_logger().error(f"Failed to open /dev/video{cam_id}")
            return

        self.get_logger().info(f"Started capture thread for camera {cam_id}")

        rate = 1.0 / self.publish_rate
        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f"Camera {cam_id}: Failed to read frame.")
                time.sleep(rate)
                continue

            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.image_format)
                now = self.get_clock().now().to_msg()
                msg.header.stamp = now
                msg.header.frame_id = f"camera_{cam_id}"
                self.cam_publishers[cam_id].publish(msg)
            except Exception as e:
                self.get_logger().error(f"Camera {cam_id}: Error publishing frame - {e}")

            time.sleep(rate)

        cap.release()
        self.get_logger().info(f"Camera {cam_id}: Video capture stopped.")

    def stop_all(self):
        self.running = False
        for cam_id, thread in self.capture_threads.items():
            if thread.is_alive():
                thread.join()
        self.get_logger().info("All camera capture threads stopped.")


def main(args=None):
    rclpy.init(args=args)

    # === Configuration ===
    camera_ids = [0, 2, 4, 6]      # /dev/videoX devices
    resolution = (320, 240)              # Change resolution here
    image_format = 'bgr8'                # Can be 'rgb8' or 'mono8', etc.
    publish_rate = 10                    # Hz

    node = MultiCameraImagePublisher(camera_ids, resolution, image_format, publish_rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
