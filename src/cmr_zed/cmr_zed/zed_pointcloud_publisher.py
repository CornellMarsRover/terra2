#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import pyzed.sl as sl
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TwistStamped, TransformStamped
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
import tf2_ros


class ZedPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('zed_pointcloud_publisher')

        # Publishers
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/camera/points', 10)
        self.pose_publisher = self.create_publisher(TwistStamped, '/zed/pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize and open the ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Cannot open ZED camera: {status}')
            raise RuntimeError("ZED Camera open failed.")

        # Enable positional tracking
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_imu_fusion = True
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
        status = self.zed.enable_positional_tracking(tracking_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to enable positional tracking: {status}')
            self.zed.close()
            raise RuntimeError("Positional tracking initialization failed.")

        # Store the initial pose
        self.initial_pose = sl.Pose()
        self.zed.get_position(self.initial_pose, sl.REFERENCE_FRAME.WORLD)

        # Resolution for retrieving the point cloud
        self.res = sl.Resolution(width=720, height=404)
        self.point_cloud = sl.Mat(self.res.width, self.res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)

        # Timer for publishing data (10 Hz)
        self.timer_ = self.create_timer(0.1, self.publish_data)

        self.get_logger().info('ZedPointCloudPublisher node has been started.')

    def publish_data(self):
        """Publishes the point cloud, pose, and transform."""
        self.publish_pointcloud()
        self.publish_pose()
        self.publish_transform()

    def publish_pointcloud(self):
        """Captures the point cloud and publishes it."""
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, self.res)
            pc2_msg = self.convert_sl_mat_to_pointcloud2(self.point_cloud)
            self.pointcloud_publisher.publish(pc2_msg)

    def publish_pose(self):
        """Publishes the camera's relative pose as a TwistStamped message."""
        current_pose = sl.Pose()
        if self.zed.get_position(current_pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:

            # Extract translation
            translation = current_pose.get_translation(sl.Translation())

            # Extract rotation as Euler angles
            rotation = current_pose.get_rotation_vector()
            self.get_logger().info(f"Translation: {translation.get()}")
            self.get_logger().info(f"Rotation: {rotation}")

            '''# Create and publish the TwistStamped message
            pose_msg = TwistStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"  # World frame

            # Set linear velocities (relative translation)
            pose_msg.twist.linear.x = translation.x
            pose_msg.twist.linear.y = translation.y
            pose_msg.twist.linear.z = translation.z

            # Set angular velocities (relative rotation)
            #pose_msg.twist.angular.x = roll
            #pose_msg.twist.angular.y = pitch
            #pose_msg.twist.angular.z = yaw

            self.pose_publisher.publish(pose_msg)'''

    def publish_transform(self):
        """Publishes a static transform from `map` to `zed_camera_frame`."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Parent frame
        t.child_frame_id = "zed_camera_frame"  # Camera frame

        # Static translation (camera is 0.5m above the map)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.5

        # Identity quaternion (no rotation)
        t.transform.rotation.x = 0.7071068
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.7071068

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

    def destroy_node(self):
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
