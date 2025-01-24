import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
import struct
import math

class ObjectRecognitionSim(Node):
    def __init__(self):
        super().__init__('object_recognition_sim')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.point_cloud_sub = self.create_subscription(
            PointCloud2, '/camera/points', self.point_cloud_callback, 10)

        # Publisher
        self.detected_pub = self.create_publisher(
            PointStamped, '/autonomy/object/detected', 10)

        # Utilities
        self.bridge = CvBridge()
        self.intrinsic_matrix = None
        self.point_cloud = None

    def camera_info_callback(self, msg):
        """Extract camera intrinsic matrix from CameraInfo message."""
        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)

    def point_cloud_callback(self, msg):
        """Store the latest point cloud message."""
        self.point_cloud = msg

    def image_callback(self, msg):
        """Process the image to detect ArUco markers and compute corresponding points."""
        if self.intrinsic_matrix is None or self.point_cloud is None:
            return

        # Convert ROS Image to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect ArUco markers
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_params)

        if ids is not None:
            for corner in corners:
                # Compute the center of the marker
                center_x = int(np.mean(corner[0][:, 0]))
                center_y = int(np.mean(corner[0][:, 1]))

                # Get the corresponding 3D point from the point cloud
                point = self.get_point_from_pointcloud(center_x, center_y)

                if point:
                    # Publish the detected point
                    point_msg = PointStamped()
                    point_msg.header.stamp = msg.header.stamp
                    point_msg.header.frame_id = self.point_cloud.header.frame_id
                    point_msg.point.x, point_msg.point.y, point_msg.point.z = point

                    self.detected_pub.publish(point_msg)

    def get_point_from_pointcloud(self, u, v):
        """Retrieve a 3D point corresponding to a pixel from the PointCloud2 message."""
        if self.point_cloud is None:
            return None

        width = self.point_cloud.width
        height = self.point_cloud.height

        # Ensure the pixel is within bounds
        if u < 0 or u >= width or v < 0 or v >= height:
            return None

        point_index = v * width + u

        # Extract the point from the PointCloud2 data
        fmt = "<fff"  # XYZ format
        offset = point_index * struct.calcsize(fmt)
        data = self.point_cloud.data[offset:offset + struct.calcsize(fmt)]
        point = struct.unpack(fmt, data)

        # Handle invalid points (NaN or Inf)
        if any(math.isnan(coord) or math.isinf(coord) for coord in point):
            return None

        return point


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognitionSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
