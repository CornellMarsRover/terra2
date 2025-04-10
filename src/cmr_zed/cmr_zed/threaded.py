#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import pyzed.sl as sl
import numpy as np
import threading
import time
from collections import deque

from cmr_msgs.msg import GroundPlaneStamped
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TwistStamped, TransformStamped
from std_msgs.msg import Header
import tf2_ros


class FrequencyTracker:
    def __init__(self):
        self.timestamps = []
        self.last_n_times = deque(maxlen=10)

    def update(self):
        now = time.time()
        self.timestamps.append(now)
        self.last_n_times.append(now)

    def average_frequency(self):
        if len(self.timestamps) < 2:
            return 0.0
        total_time = self.timestamps[-1] - self.timestamps[0]
        return (len(self.timestamps) - 1) / total_time if total_time > 0 else 0.0

    def recent_frequency(self):
        if len(self.last_n_times) < 2:
            return 0.0
        delta = self.last_n_times[-1] - self.last_n_times[0]
        return (len(self.last_n_times) - 1) / delta if delta > 0 else 0.0


class ZedAutonomy(Node):
    def __init__(self):
        super().__init__('zed_autonomy')

        # Publishers
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/camera/points', 10)
        self.pose_publisher = self.create_publisher(TwistStamped, '/zed/pose', 10)
        self.ground_publisher = self.create_publisher(GroundPlaneStamped, '/camera/ground_plane', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ZED camera setup
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Cannot open ZED camera: {status}')
            raise RuntimeError("ZED Camera open failed.")

        # Positional tracking
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
        status = self.zed.enable_positional_tracking(tracking_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to enable positional tracking: {status}')
            self.zed.close()
            raise RuntimeError("Positional tracking initialization failed.")

        # Data buffers
        self.res = sl.Resolution(width=360, height=202)
        self.point_cloud = sl.Mat(self.res.width, self.res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)
        self.current_pose = sl.Pose()
        self.ground_plane = sl.Plane()
        self.plane_parameters = sl.PlaneDetectionParameters()
        self.plane_parameters.normal_similarity_threshold = 6
        self.plane_parameters.max_distance_threshold = 0.05

        # Threading Locks
        self.pc_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        self.ground_lock = threading.Lock()

        # Data copies
        self.pc_data = None
        self.pc_stamp = None
        self.pose_data = None
        self.pose_stamp = None
        self.ground_data = None
        self.ground_stamp = None

        # Frequency trackers
        self.pc_tracker = FrequencyTracker()
        self.pose_tracker = FrequencyTracker()
        self.ground_tracker = FrequencyTracker()
        self.publish_pc_tracker = FrequencyTracker()
        self.publish_pose_tracker = FrequencyTracker()
        self.publish_ground_tracker = FrequencyTracker()

        # Start ZED worker thread
        self.running = True
        self.worker_thread = threading.Thread(target=self.zed_worker_loop, daemon=True)
        self.worker_thread.start()

        # Timers for publishing
        self.pointcloud_timer = self.create_timer(0.1, self.publish_pointcloud)
        self.pose_timer = self.create_timer(0.1, self.publish_pose_data)
        self.ground_timer = self.create_timer(0.1, self.publish_ground)

        self.get_logger().info('ZedAutonomy node has been started.')

    def zed_worker_loop(self):
        """Worker thread to safely acquire all camera data."""
        hits = [[600, 600], [300, 600], [1000, 600]]

        while self.running:
            success = False
            # Point cloud
            with self.pc_lock:
                if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
                    self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZ, sl.MEM.CPU, self.res)
                    self.pc_data = self.point_cloud.get_data().copy()
                    self.pc_stamp = self.get_clock().now().to_msg()
                    self.pc_tracker.update()
                    self.log_frequency("PointCloud", self.pc_tracker)
                    success = True
            
            # Pose
            with self.pose_lock:
                if success and self.zed.get_position(self.current_pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.pose_data = self.current_pose
                    self.pose_stamp = self.get_clock().now().to_msg()
                    self.pose_tracker.update()
                    self.log_frequency("Pose", self.pose_tracker)

            # Ground plane
            with self.ground_lock:
                if success:
                    for hit in hits:
                        status = self.zed.find_plane_at_hit(hit, self.ground_plane, self.plane_parameters)
                        if status == sl.ERROR_CODE.SUCCESS and self.ground_plane.type == sl.PLANE_TYPE.HORIZONTAL:
                            self.ground_data = self.ground_plane.get_bounds().copy()
                            self.ground_stamp = self.get_clock().now().to_msg()
                            self.ground_tracker.update()
                            self.log_frequency("GroundPlane", self.ground_tracker)
                            break
            # Sleep to allow data access
            time.sleep(0.01)

    def log_frequency(self, label, tracker: FrequencyTracker):
        avg_freq = tracker.average_frequency()
        recent_freq = tracker.recent_frequency()
        self.get_logger().info(
            f"{label} - Avg: {avg_freq:.2f} Hz | Last 10: {recent_freq:.2f} Hz"
        )

    def publish_pointcloud(self):
        with self.pc_lock:
            pc_data = self.pc_data
            pc_stamp = self.pc_stamp

        if self.pc_data is not None:
            #msg = self.convert_array_to_pointcloud2(pc_data, pc_stamp)
            #self.pointcloud_publisher.publish(msg)
            self.publish_pc_tracker.update()
            self.log_frequency("PC publish", self.publish_pc_tracker)

    def publish_ground(self):
        with self.ground_lock:
            ground_data = self.ground_data
            ground_stamp = self.ground_stamp

        if ground_data is not None:
            self.publish_ground_plane(ground_data, ground_stamp)
            self.publish_ground_tracker.update()
            self.log_frequency("Ground plane publish", self.publish_ground_tracker)

    def publish_pose_data(self):
        with self.pose_lock:
            pose_data = self.pose_data
            pose_stamp = self.pose_stamp

        if pose_data is not None:
            self.publish_pose(pose_data, pose_stamp)
            self.publish_transform(pose_stamp)
            self.publish_pose_tracker.update()
            self.log_frequency("Pose publish", self.publish_pose_tracker)

    def publish_pose(self, pose, stamp):
        translation = pose.get_translation(sl.Translation()).get()
        rotation = pose.get_rotation_vector()

        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "zed_camera_frame"
        msg.twist.linear.x = translation[0]
        msg.twist.linear.y = translation[1]
        msg.twist.linear.z = translation[2]
        msg.twist.angular.x = rotation[0]
        msg.twist.angular.y = rotation[1]
        msg.twist.angular.z = rotation[2]
        self.pose_publisher.publish(msg)

    def publish_ground_plane(self, ground_bounds, stamp):
        msg = GroundPlaneStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "zed_camera_frame"
        msg.x = [float(x) for x in ground_bounds[:, 0]]
        msg.y = [float(y) for y in ground_bounds[:, 1]]
        self.ground_publisher.publish(msg)

    def publish_transform(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "zed_camera_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.1
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def convert_array_to_pointcloud2(self, pc_array, stamp):
        height, width, channels = pc_array.shape
        assert channels == 4, "Expecting 4 channels (X, Y, Z, RGBA)."

        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = "zed_camera_frame"
        msg.height = height
        msg.width = width
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * width
        msg.is_dense = False

        msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))

        msg.data = pc_array.tobytes()
        return msg

    def destroy_node(self):
        self.running = False
        self.worker_thread.join(timeout=1.0)
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedAutonomy()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
