#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import threading
import time
from cv_bridge import CvBridge

class MultiCameraStitchedPublisher(Node):
    def __init__(self,
                 camera_ids=(0, 2, 4, 6),
                 resolution=(320, 240),
                 image_format='bgr8',
                 publish_rate=10):
        """
        A ROS2 node that captures images from multiple cameras and publishes a
        single stitched 2x2 image of all four feeds.

        camera_ids: Tuple of 4 integers for the /dev/videoX devices.
                    By default: 0=front, 2=left, 4=back, 6=right.
        resolution: Tuple (width, height) of each camera.
        image_format: OpenCV encoding (e.g., 'bgr8', 'rgb8', 'mono8').
        publish_rate: Rate (Hz) at which the stitched image is published.
        """
        super().__init__('multi_camera_stitched_publisher')
        self.get_logger().info("Initializing Multi-Camera Stitched Publisher...")

        # Camera ID -> Label
        self.camera_labels = {
            0: "FRONT",
            2: "LEFT",
            4: "BACK",
            6: "RIGHT"
        }
        # Camera ID -> Position in the 2x2 grid (row, col)
        #   0 -> top-left,    4 -> top-right
        #   2 -> bottom-left, 6 -> bottom-right
        self.camera_positions = {
            0: (0, 0),
            4: (0, 1),
            2: (1, 0),
            6: (1, 1)
        }

        self.camera_ids = camera_ids
        self.resolution = resolution
        self.image_format = image_format
        self.publish_rate = publish_rate
        self.bridge = CvBridge()

        self.frames = {cid: None for cid in camera_ids}  # latest frames
        self.caps = {}
        self.threads = {}
        self.running = True

        # Open each camera, set resolution and spawn capture threads
        for cid in camera_ids:
            cap = cv2.VideoCapture(cid)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
            cap.set(cv2.CAP_PROP_FPS, publish_rate)
            if not cap.isOpened():
                self.get_logger().error(f"Failed to open /dev/video{cid}")
            self.caps[cid] = cap

            t = threading.Thread(target=self._capture_loop, args=(cid,))
            t.daemon = True
            t.start()
            self.threads[cid] = t

        # Publisher for the stitched image
        self.stitched_pub = self.create_publisher(Image, "/camera/stitched_image", 10)

        # Create a timer to publish the stitched image
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._publish_stitched_image)

    def _capture_loop(self, cid):
        """
        Continuously reads frames from camera 'cid' and stores the latest in self.frames[cid].
        """
        cap = self.caps[cid]
        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                # If no frame, just store None to indicate error
                self.frames[cid] = None
                time.sleep(0.01)
                continue
            self.frames[cid] = frame
            # Avoid busy looping at max speed
            time.sleep(0.001)

        cap.release()
        self.get_logger().info(f"Camera {cid} capture loop stopped.")

    def _publish_stitched_image(self):
        """
        Assembles the 2x2 image from the latest frames and publishes it.
        If any camera feed is unavailable, uses a black placeholder with white text.
        Draws bright green label text at the top-left of each subimage,
        and separates the subimages with black lines.
        """
        # Each subimage is resolution=(width, height)
        width, height = self.resolution
        # Final stitched image: 2x width, 2x height
        stitched = np.zeros((height * 2, width * 2, 3), dtype=np.uint8)

        for cid in self.camera_ids:
            # Determine top-left corner in the stitched array
            row, col = self.camera_positions[cid]
            y0 = row * height
            x0 = col * width

            # Get the latest frame or a placeholder
            frame = self.frames[cid]
            subimage = None
            if frame is not None:
                # Ensure the frame is the correct size (just in case)
                subimage = cv2.resize(frame, (width, height))
                # Rotate image 180 degrees
                subimage = cv2.rotate(subimage, cv2.ROTATE_180)
            else:
                # Create a placeholder
                subimage = np.zeros((height, width, 3), dtype=np.uint8)
                cv2.putText(subimage,
                            "NO IMAGE AVAILABLE",
                            (int(width * 0.1), int(height * 0.5)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,  # font scale
                            (255, 255, 255),  # white text
                            2,
                            cv2.LINE_AA)

            # Label the subimage (bright green text at top-left)
            camera_label = self.camera_labels.get(cid, f"Cam {cid}")
            cv2.putText(subimage,
                        camera_label,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,  # font scale
                        (0, 255, 0),  # bright green
                        2,
                        cv2.LINE_AA)

            # Insert subimage into stitched image
            stitched[y0:y0+height, x0:x0+width] = subimage

        # Draw black lines separating the images
        # Vertical line at x = width
        cv2.line(stitched, (width, 0), (width, height * 2), (0, 0, 0), 2)
        # Horizontal line at y = height
        cv2.line(stitched, (0, height), (width * 2, height), (0, 0, 0), 2)

        # Convert to ROS Image and publish
        try:
            msg = self.bridge.cv2_to_imgmsg(stitched, encoding=self.image_format)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "stitched_image"
            self.stitched_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing stitched image: {e}")

    def stop_all(self):
        """
        Stop capture loops and release resources.
        """
        self.running = False
        for cid, t in self.threads.items():
            if t.is_alive():
                t.join()
        self.get_logger().info("All camera capture threads stopped.")

def main(args=None):
    rclpy.init(args=args)

    # Adjust these to match your needs
    camera_ids = [0, 2, 4, 6]   # /dev/videoX devices
    resolution = (320, 240)     # each camera's resolution
    image_format = 'bgr8'
    publish_rate = 10           # Hz

    node = MultiCameraStitchedPublisher(
        camera_ids=camera_ids,
        resolution=resolution,
        image_format=image_format,
        publish_rate=publish_rate
    )

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
