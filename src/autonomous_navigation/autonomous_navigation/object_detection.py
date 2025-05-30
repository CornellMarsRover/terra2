#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

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
        model_path = os.path.join(
            get_package_share_directory('autonomous_navigation'),
            'best.pt'
        )

        # set to False if you want to read CameraInfo over the wire
        self.real = True

        # state
        self.current_target_id = None
        self.target_found = False
        self.target_coordinates = (0.0, 0.0)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        # camera intrinsics (real)
        if self.real:
            self.camera_matrix = np.array([
                [532.005,   0.0,   654.085],
                [  0.0,   532.05,  376.963],
                [  0.0,     0.0,     1.0 ],
            ], dtype=np.float64)
            self.dist_coeffs = np.array([
                -0.0564406, 0.0365949, 0.000583109, 0.000200153, -0.0160142
            ], dtype=np.float64)
        else:
            self.camera_matrix = None
            self.dist_coeffs = None

        # subscriptions
        self.create_subscription(
            String,
            '/autonomy/target_object/name',
            self.name_cb,
            10
        )
        self.create_subscription(
            TwistStamped,
            '/autonomy/pose/robot/global',
            self.pose_cb,
            10
        )
        if not self.real:
            self.create_subscription(
                CameraInfo,
                '/camera/camera_info',
                self.cam_info_cb,
                10
            )
            self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_cb,
                10
            )
        else:
            self.create_subscription(
                Image,
                '/zed/image',
                self.image_cb,
                10
            )

        # publisher for target position
        self.pub_target = self.create_publisher(
            Twist,
            '/autonomy/target_object/position',
            10
        )

        # ArUco detector setup (OpenCV 4.7+ OO API)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.aruco_params
        )

        self.object_ids = {
            "ar1": 2,
            "ar2": 2,
            "ar3": 4,
            "mallet": 5,
            "bottle": 6
        }
        self.mallet_pos = None
        self.bottle_pos = None
        self.curr_id = 2
        self.get_logger().info("Object Detection Node initialized ooga booga")
    def name_cb(self, msg: String):
        name = msg.data
        #self.get_logger().info(f"{name}")
        if name == 'coordinate':
            self.current_target_id = None
            self.target_found = False
            return
        if name in self.object_ids:
            self.curr_id = self.object_ids[name]
        try:
            # assume names like "ar1"
            tid = int(name[-1])
            #self.get_logger().info(f'New target requested: AR#{tid}')
            self.current_target_id = tid
            self.target_found = False
        except ValueError:
            self.get_logger().warn(f"Bad target name '{name}'")

    def pose_cb(self, msg: TwistStamped):
        self.robot_x     = msg.twist.linear.x
        self.robot_y     = msg.twist.linear.y
        self.robot_theta = msg.twist.angular.z

    def cam_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape((3,3))
            D = np.array(msg.d)
            self.camera_matrix = K
            self.dist_coeffs  = D
            self.get_logger().info('Camera intrinsics received.')

    def image_cb(self, img_msg: Image):
        # wait until a target is requested
        if self.current_target_id is None:
            return

        '''# if we've already seen it once, just re-publish the last coords
        if self.target_found:
            out = Twist()
            out.linear.x  = float(self.target_coordinates[0])
            out.linear.y  = float(self.target_coordinates[1])
            out.angular.z = 0.0
            self.pub_target.publish(out)
            return'''

        # need intrinsics
        if self.camera_matrix is None:
            self.get_logger().warn('No camera intrinsics yet.')
            return

        # convert and grayscale
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        gray   = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # detect all markers
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return

        ids = ids.flatten()
        matches = np.where(ids == self.current_target_id)[0]
        if matches.size == 0:
            return
        
        # full pose estimation
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(
            corners,
            self.marker_length,
            self.camera_matrix,
            self.dist_coeffs
        )

        # pick our target
        idx = matches[0]
        t   = tvecs[idx]   # [x_cam, y_cam, z_cam]

        self.get_logger().info(f"{t}")
        # camera→robot frame
        x_r = t[2][0]   # forward
        y_r = -t[0][0]   # left

        # robot→global
        θ = self.robot_theta
        dx = x_r * np.cos(θ) - y_r * np.sin(θ)
        dy = x_r * np.sin(θ) + y_r * np.cos(θ)
        global_x = self.robot_x + dx
        global_y = self.robot_y + dy

        # publish
        out = Twist()
        out.linear.x  = float(global_x)
        out.linear.y  = float(global_y)
        out.linear.z = float(self.curr_id)
        self.get_logger().info(f"{global_x}  {global_y}  {self.curr_id}")
        out.angular.z = 0.0
        self.pub_target.publish(out)
        self.target_coordinates = (global_x, global_y)
        self.get_logger().info(
            f'AR#{self.current_target_id} → '
            f'(north={global_x:.2f}, west={global_y:.2f})'
        )
        self.target_found = True

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    """
    Estimate pose for each detected ArUco marker corner set.
    Returns (rvecs, tvecs, trash) to match the old API.
    """
    # define 3D marker corner positions in its own frame
    marker_points = np.array([
        [-marker_size/2,  marker_size/2, 0],
        [ marker_size/2,  marker_size/2, 0],
        [ marker_size/2, -marker_size/2, 0],
        [-marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)

    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        retval, rvec, tvec = cv2.solvePnP(
            marker_points, c, mtx, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        trash.append(retval)
        rvecs.append(rvec)
        tvecs.append(tvec)
    return np.array(rvecs), np.array(tvecs), trash

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
