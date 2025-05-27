#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('target_object_detector')
        self.bridge = CvBridge()

        # parameters
        self.declare_parameter('tag_cell_size', 0.025)  # meters
        self.declare_parameter('tag_grid_size', 4)      # cells per side

        cell_size = self.get_parameter('tag_cell_size').value
        grid_size = self.get_parameter('tag_grid_size').value
        self.marker_length = cell_size * grid_size  # e.g. 0.10 m

        # state
        self.current_target_string = "coordinate"
        self.current_target_id = None
        self.target_found = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # camera intrinsics (to be filled once CameraInfo arrives)
        self.camera_matrix = None
        self.dist_coeffs  = None

        # subscribers
        self.create_subscription(String,
                                 '/autonomy/target_object/name',
                                 self.name_cb, 10)

        self.create_subscription(TwistStamped,
                                 '/autonomy/pose/robot/global',
                                 self.pose_cb, 10)

        self.create_subscription(CameraInfo,
                                 '/camera/camera_info',
                                 self.cam_info_cb, 10)

        self.create_subscription(Image,
                                 '/camera/image_raw',
                                 self.image_cb, 10)

        # publisher
        self.pub_target = self.create_publisher(
            Twist,
            '/autonomy/target_object/position',
            10
        )

        # ArUco setup
        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(
                              cv2.aruco.DICT_4X4_50
                            )
        self.aruco_params = cv2.aruco.DetectorParameters()

    def name_cb(self, msg: String):
        if msg.data == 'coordinate':
            self.current_target_id = None
            return
        # Avoid resetting from same object messages
        #if msg.data == self.current_target_string:
        #   return
        try:
            tid = int(msg.data[2:])
            #self.get_logger().info(f'New target requested: AR#{tid}')
            self.current_target_id = tid
            self.target_found = False
        except ValueError:
            self.get_logger().warn(f"Bad target name '{msg.data}'")

    def pose_cb(self, msg: TwistStamped):
        self.robot_x     = msg.twist.linear.x
        self.robot_y     = msg.twist.linear.y
        self.robot_theta = msg.twist.angular.z

    def cam_info_cb(self, msg: CameraInfo):
        # only need to read once
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape((3, 3))
            D = np.array(msg.d)
            self.camera_matrix = K
            self.dist_coeffs  = D
            self.get_logger().info('Camera intrinsics received.')

    def image_cb(self, img_msg: Image):
        if self.current_target_id is None:
            return
        if self.target_found:
            out = Twist()
            out.linear.x  = float(self.target_coordinates[0])
            out.linear.y  = float(self.target_coordinates[1])
            out.angular.z = 0.0
            self.pub_target.publish(out)
            return
        if self.camera_matrix is None:
            self.get_logger().warn('No camera intrinsics yet.')
            return

        # Convert to grayscale
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        gray   = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        if ids is None:
            return

        ids = ids.flatten()
        idx_arr = np.where(ids == self.current_target_id)[0]
        if idx_arr.size == 0:
            return
        idx = idx_arr[0]

        # Estimate pose of all detected markers
        # rvecs, tvecs shape=(N,1,3)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs
        )

        # pick our target’s translation vector
        t = tvecs[idx][0]   # [x_cam, y_cam, z_cam] in OpenCV camera frame

        # Convert to your robot frame: 
        #   OpenCV camera frame: x→right, y→down, z→forward
        #   Robot frame        : x→forward, y→left,  z→up
        x_r =  t[2]          # forward
        y_r = -t[0]          # left
        # z_r = -t[1]        # up (unused)

        # Transform into Gazebo global (north=x, west=y)
        θ = self.robot_theta
        dx = x_r * np.cos(θ) - y_r * np.sin(θ)
        dy = x_r * np.sin(θ) + y_r * np.cos(θ)
        global_x = self.robot_x + dx
        global_y = self.robot_y + dy

        # Publish
        out = Twist()
        out.linear.x  = float(global_x)
        out.linear.y  = float(global_y)
        out.angular.z = 0.0
        self.pub_target.publish(out)
        self.target_coordinates = (global_x, global_y)
        self.get_logger().info(
            f'AR#{self.current_target_id} → '
            f'(north={global_x:.2f}, west={global_y:.2f})'
        )
        self.target_found = True


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
