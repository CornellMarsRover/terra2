#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import pyzed.sl as sl
import numpy as np
import math

from sensor_msgs.msg import PointCloud2, PointField, NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped, TwistStamped
from std_msgs.msg import Header, Float32MultiArray, MultiArrayDimension
from tf_transformations import euler_from_quaternion
import tf2_ros


class ZedAutonomy(Node):
    def __init__(self):
        super().__init__('zed_autonomy')

        # Publishers
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/camera/points', 10)
        self.ground_publisher = self.create_publisher(Float32MultiArray, '/camera/ground_plane', 10)
        self.pose_publisher = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize and open the ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.QUALITY
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Cannot open ZED camera: {status}')
            raise RuntimeError("ZED Camera open failed.")

        # Enable positional tracking
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_2
        status = self.zed.enable_positional_tracking(tracking_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to enable positional tracking: {status}')
            self.zed.close()
            raise RuntimeError("Positional tracking initialization failed.")
        
        # Resolution for retrieving the point cloud
        self.res = sl.Resolution(width=720, height=404)
        self.point_cloud = sl.Mat(self.res.width, self.res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)
        self.current_pose = sl.Pose()
        self.current_xy = [0.0, 0.0]
        self.current_yaw = 0.0  # Current camera yaw for ground plane rotation
        self.ground_plane = sl.Plane()  # Detected ground plane

        self.last_pose = np.zeros(6)
        self.last_covariance = np.zeros(36)

        # Timers for publishing data
        self.pointcloud_timer = self.create_timer(0.2, self.publish_pointcloud)
        self.pose_timer = self.create_timer(0.1, self.publish_pose_data)
        self.get_logger().info('ZedAutonomy node has been started.')

    def publish_pose_data(self):
        """Publishes the point cloud, pose, and transform."""
        self.publish_pose()
        self.publish_transform()

    def publish_ground_plane(self):
        """Publishes the current ground plane detection"""
        rotation_vec = self.current_pose.pose_data().get_rotation_matrix().get_rotation_vector()
        plane_transform = sl.Transform()
        plane_transform.init_transform(self.current_pose.pose_data())
        plane_transform.set_rotation_vector(rotation_vec[0], rotation_vec[1], rotation_vec[2])
        find_plane_status = self.zed.find_floor_plane(self.ground_plane, plane_transform)
        if find_plane_status == sl.ERROR_CODE.SUCCESS:
            pts = self.ground_plane_to_msg(self.ground_plane.get_bounds())
            ground_plane_msg = Float32MultiArray()
            ground_plane_msg.data = pts
            self.ground_publisher.publish(ground_plane_msg)

    def publish_pointcloud(self):
        """Captures the point cloud and publishes it."""
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, self.res)
            pc2_msg = self.convert_sl_mat_to_pointcloud2(self.point_cloud)
            self.pointcloud_publisher.publish(pc2_msg)
            self.publish_ground_plane()

    def publish_pose(self):
        """Publishes the camera's relative pose as a TwistStamped message."""
        if self.zed.get_position(self.current_pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:

            ns = self.current_pose.timestamp.data_ns
            # Extract translation
            translation = self.current_pose.get_translation(sl.Translation())
            translation = translation.get()

            # Extract rotation
            rotation = self.current_pose.get_rotation_vector()

            #pose_msg = TwistWithCovarianceStamped()
            pose_msg = TwistStamped()
            pose_msg.header = self.nanosec_to_ros_header(ns)

            # Translations
            self.current_xy = (translation[0], translation[1])
            '''pose_msg.twist.twist.linear.x = translation[0]
            pose_msg.twist.twist.linear.y = translation[1]
            pose_msg.twist.twist.linear.z = translation[2]
            self.get_logger().info(f"xy: {self.current_xy}")
            # Rotations
            self.current_yaw = rotation[2]
            pose_msg.twist.twist.angular.x = rotation[0]
            pose_msg.twist.twist.angular.y = rotation[1]
            pose_msg.twist.twist.angular.z = rotation[2]'''

            pose_msg.twist.linear.x = translation[0]
            pose_msg.twist.linear.y = translation[1]
            pose_msg.twist.linear.z = translation[2]
            self.get_logger().info(f"xy: {self.current_xy}")
            # Rotations
            self.current_yaw = rotation[2]
            pose_msg.twist.angular.x = rotation[0]
            pose_msg.twist.angular.y = rotation[1]
            pose_msg.twist.angular.z = rotation[2]

            '''
            covariance = self.current_pose.pose_covariance.reshape(-1)
            cov = covariance
            cov[0:18] += self.last_covariance[0:18]
            self.last_covariance = covariance
            pose_msg.twist.covariance = covariance
            '''
            self.pose_publisher.publish(pose_msg)

            # Update last global pose
            self.last_pose[0:3] = np.array(translation)
            self.last_pose[3:6] = np.array(rotation)

    def publish_transform(self):
        """
        Publishes a static transform from `map` to `zed_camera_frame`
        for visualization in RVIZ
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Parent frame
        t.child_frame_id = "zed_camera_frame"  # Camera frame

        # Static translation (camera is z m above the ground)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.3

        # Quaternion rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Publish transform
        self.tf_broadcaster.sendTransform(t)

    def convert_sl_mat_to_pointcloud2(self, sl_mat):
        """Convert an sl.Mat (F32_C4) to a sensor_msgs/PointCloud2."""
        pc_data = sl_mat.get_data()
        height, width, channels = pc_data.shape
        assert channels == 4, "Expecting 4 channels (X, Y, Z, RGBA)."

        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "zed_camera_frame"

        msg.height = height
        msg.width = width
        msg.is_bigendian = False
        msg.point_step = 16  # 4 floats * 4 bytes = 16 bytes
        msg.row_step = msg.point_step * width
        msg.is_dense = False  # Allow NaN points

        msg.fields.append(PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))

        msg.data = pc_data.tobytes()

        return msg
    
    def ground_plane_to_msg(self, ground_plane):
        """
        Convert ground plane np array to list of floats to publish,
        applying same yaw + XY transform as in the node.
        """
        R = np.array([
            [np.cos(self.current_yaw), -np.sin(self.current_yaw)],
            [np.sin(self.current_yaw),  np.cos(self.current_yaw)]
        ])
        x_vals = ground_plane[:, 0]
        y_vals = ground_plane[:, 1]
        
        result = []
        for x, y in zip(x_vals, y_vals):
            rotated_pt = R.dot(np.array([x, y]))
            rotated_pt[0] += self.current_xy[0]
            rotated_pt[1] += self.current_xy[1]
            result.extend(rotated_pt)

        return result

    def nanosec_to_ros_header(self, ns):
        """
        Helper function to convert timestamp in nanoseconds to seconds and nanoseconds
        """
        sec = int(ns // 1e9)
        nanosec = int(ns - (sec * 1e9))
        header = Header()
        header.stamp.sec = sec
        header.stamp.nanosec = nanosec
        return header

    def destroy_node(self):
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedAutonomy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
