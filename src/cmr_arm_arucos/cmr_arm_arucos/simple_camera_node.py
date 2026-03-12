"""
Lightweight camera publisher using cv2.VideoCapture.
Works with any USB camera (including ZED as a standard UVC device)
without requiring the ZED SDK (pyzed).

Auto-detects available video devices and tries each one.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import glob
import subprocess
import os


class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera_node')

        self.declare_parameter('camera_index', -1)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('image_topic', '/zed/image_left')
        self.declare_parameter('kill_competing', True)

        camera_index = self.get_parameter('camera_index').value
        fps = float(self.get_parameter('fps').value)
        image_topic = self.get_parameter('image_topic').value
        kill_competing = self.get_parameter('kill_competing').value

        self.publisher = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()

        if kill_competing:
            self._kill_competing_processes()

        self.cap = self._open_camera(camera_index)

        for _ in range(10):
            ret, _ = self.cap.read()
            if ret:
                break
            time.sleep(0.2)

        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(
            f'Camera opened, publishing to {image_topic} at {fps} FPS')

    def _kill_competing_processes(self):
        """Kill other processes that may be holding /dev/video* devices."""
        try:
            video_devices = glob.glob('/dev/video*')
            if not video_devices:
                return
            result = subprocess.run(
                ['fuser'] + video_devices,
                capture_output=True, text=True, timeout=5)
            pids = result.stdout.strip().split()
            my_pid = str(os.getpid())
            for pid in pids:
                pid = pid.strip()
                if pid and pid != my_pid:
                    self.get_logger().warn(f'Killing process {pid} holding camera device')
                    subprocess.run(['kill', '-9', pid], timeout=5)
            if pids:
                time.sleep(1.0)
        except Exception as e:
            self.get_logger().warn(f'Could not check/kill competing processes: {e}')

    def _open_camera(self, requested_index):
        """Try to open a camera, auto-detecting if index is -1."""
        video_devices = sorted(glob.glob('/dev/video*'))
        self.get_logger().info(f'Available video devices: {video_devices}')

        if requested_index >= 0:
            indices = [requested_index]
        else:
            indices = []
            for dev in video_devices:
                try:
                    indices.append(int(dev.replace('/dev/video', '')))
                except ValueError:
                    pass
            if not indices:
                indices = [0, 1, 2]

        for idx in indices:
            self.get_logger().info(f'Trying camera index {idx}...')
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.get_logger().info(
                        f'Camera {idx} opened successfully: {frame.shape[1]}x{frame.shape[0]}')
                    return cap
                cap.release()
                self.get_logger().warn(f'Camera {idx} opened but cannot read frames')
            else:
                self.get_logger().warn(f'Camera {idx} failed to open')

        for idx in indices:
            self.get_logger().info(f'Trying camera index {idx} with default backend...')
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                time.sleep(0.5)
                ret, frame = cap.read()
                if ret and frame is not None:
                    self.get_logger().info(
                        f'Camera {idx} opened (default backend): {frame.shape[1]}x{frame.shape[0]}')
                    return cap
                cap.release()

        raise RuntimeError(
            f'No working camera found. Tried indices {indices}. '
            f'Devices: {video_devices}')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
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
