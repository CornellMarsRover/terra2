#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('target_object_detector')
        self.bridge = CvBridge()

        self.declare_parameter('marker_length', 0.10)
        self.marker_length = self.get_parameter('marker_length').value

        # state
        self.current_target_string = "coordinate"
        self.current_target_id = None
        self.target_found = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # ZED left camera intrinsics (HD1080 mode)
        self.camera_matrix = np.array([
            [1065.0, 0.0, 960.0],
            [0.0, 1065.0, 540.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(4)

        # subscribers
        self.create_subscription(String,
                                 '/autonomy/target_object/name',
                                 self.name_cb, 10)

        self.create_subscription(TwistStamped,
                                 '/autonomy/pose/robot/global',
                                 self.pose_cb, 10)

        self.create_subscription(Image,
                                 '/zed/image_left',
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
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params)

        half = self.marker_length / 2.0
        self.marker_obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        self.get_logger().info('ArUco object detection node started.')

    def name_cb(self, msg: String):
        if msg.data == 'coordinate':
            self.current_target_id = None
            return
        try:
            tid = int(msg.data[2:])
            self.current_target_id = tid
            self.target_found = False
        except ValueError:
            self.get_logger().warn(f"Bad target name '{msg.data}'")

    def pose_cb(self, msg: TwistStamped):
        self.robot_x     = msg.twist.linear.x
        self.robot_y     = msg.twist.linear.y
        self.robot_theta = msg.twist.angular.z

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

        # ZED publishes bgra8, convert to BGR then grayscale
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        if cv_img.shape[-1] == 4:
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        if ids is None:
            return

        ids = ids.flatten()
        idx_arr = np.where(ids == self.current_target_id)[0]
        if idx_arr.size == 0:
            return
        idx = idx_arr[0]

        # Estimate pose of all detected markers
        # rvecs, tvecs shape=(N,1,3)
        success, rvec, tvec = cv2.solvePnP(
            self.marker_obj_points, corners[idx][0],
            self.camera_matrix, self.dist_coeffs)
        if not success:
            return

        # pick our target’s translation vector
        t = tvec.flatten()  # [x_cam, y_cam, z_cam] in OpenCV camera frame

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
