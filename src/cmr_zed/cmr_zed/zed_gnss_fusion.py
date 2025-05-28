#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import pyzed.sl as sl
import numpy as np
import math

from sensor_msgs.msg import PointCloud2, PointField, NavSatFix
from geometry_msgs.msg import TwistStamped, TransformStamped
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
import tf2_ros


class ZedAutonomy(Node):
    def __init__(self):
        super().__init__('zed_autonomy')

        # GPS subscriber
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/navsatfix',
            self.update_gps,
            10
        )

        # Publishers
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/camera/points', 10)
        # NOTE: For now just publish zed pose to the localization topic
        self.pose_publisher = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize and open the ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        init_params.svo_real_time_mode = True
        init_params.sdk_verbose = 1
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Cannot open ZED camera: {status}')
            raise RuntimeError("ZED Camera open failed.")

        # Enable positional tracking
        '''tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_imu_fusion = True
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
        status = self.zed.enable_positional_tracking(tracking_params)'''
        status = self.zed.enable_positional_tracking()
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to enable positional tracking: {status}')
            self.zed.close()
            raise RuntimeError("Positional tracking initialization failed.")
        
        # Create Fusion object:
        self.fusion = sl.Fusion()
        init_fusion_param = sl.InitFusionParameters()
        init_fusion_param.coordinate_units = sl.UNIT.METER
        fusion_init_code = self.fusion.init(init_fusion_param)
        if fusion_init_code != sl.FUSION_ERROR_CODE.SUCCESS:
            self.get_logger().error("Failed to initialize fusion :"+repr(fusion_init_code)+". Exit program")
            exit()

        # Enable odometry publishing:
        configuration = sl.CommunicationParameters()
        self.zed.start_publishing(configuration)

        # Subscribe to odometry:
        uuid = sl.CameraIdentifier(self.zed.get_camera_information().serial_number)
        self.fusion.subscribe(uuid,configuration,sl.Transform(0,0,0))

        # Enable positional tracking for Fusion object:
        positional_tracking_fusion_parameters = sl.PositionalTrackingFusionParameters()
        positional_tracking_fusion_parameters.enable_GNSS_fusion = True 
        gnss_calibration_parameters = sl.GNSSCalibrationParameters()
        gnss_calibration_parameters.target_yaw_uncertainty = 0.1
        gnss_calibration_parameters.enable_translation_uncertainty_target = False
        gnss_calibration_parameters.enable_reinitialization = True
        gnss_calibration_parameters.gnss_vio_reinit_threshold = 5
        positional_tracking_fusion_parameters.gnss_calibration_parameters = gnss_calibration_parameters
        self.fusion.enable_positionnal_tracking(positional_tracking_fusion_parameters)

        # Store the initial pose
        self.initial_pose = sl.Pose()
        self.zed.get_position(self.initial_pose, sl.REFERENCE_FRAME.WORLD)

        # Resolution for retrieving the point cloud
        self.res = sl.Resolution(width=720, height=404)
        self.point_cloud = sl.Mat(self.res.width, self.res.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU)

        self.initial_lat = 38.161479
        self.initial_lon = -122.454630
        self.initial_altitude = 488.0
        self.curr_lat = 38.161479
        self.curr_lon = -122.454630
        self.curr_altitude = 488.0

        self.current_gnss = sl.GNSSData(in_radian=False)
        self.current_gnss.set_coordinates(self.initial_lat, self.initial_lon, self.initial_altitude, in_radian=False)
        self.current_gnss.latitude_std = 0.1
        self.current_gnss.longitude_std = 0.1
        self.current_gnss.altitude_std = 0.1
        self.current_gnss.position_covariances = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        self.current_gnss.gnss_status = sl.GNSS_STATUS.SINGLE.value
        self.set_to_current_timestamp()

        # Timer for publishing data (10 Hz)
        self.timer_ = self.create_timer(0.1, self.publish_data)

        self.get_logger().info('ZedAutonomy node has been started.')

    def set_to_current_timestamp(self):
        """
        Helper method to set sl.Timestamp object to current time
        SHOULD BE REPLACED WHEN REAL ROVER TESTING
        """
        curr_time = self.get_clock().now().to_msg()
        ns = int((curr_time.sec * 1e9) + curr_time.nanosec)
        ts = sl.Timestamp()
        ts.set_nanoseconds(ns)
        self.current_gnss.ts = ts

    def update_gps(self, msg):
        """
        Update current gps coordinate
        """
        self.current_gnss.set_coordinates(msg.latitude, msg.longitude, msg.altitude, in_radian=False)
        self.curr_lat = msg.latitude
        self.curr_lon = msg.longitude
        self.curr_altitude = msg.altitude
        self.set_to_current_timestamp()
        self.get_logger().info(f"Cam (ns): {self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).data_ns}")
        self.current_gnss.ts = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
        self.get_logger().info(f"GPS (ns): {self.current_gnss.ts.data_ns}")


    def publish_data(self):
        """Publishes the point cloud, pose, and transform."""
        #self.publish_pointcloud()
        self.publish_pose()
        #self.publish_transform()

    def publish_pointcloud(self):
        """Captures the point cloud and publishes it."""
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, self.res)
            pc2_msg = self.convert_sl_mat_to_pointcloud2(self.point_cloud)
            self.pointcloud_publisher.publish(pc2_msg)

    def publish_pose(self):
        """Publishes the camera's relative pose as a TwistStamped message."""
        # Grab camera:
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed_pose = sl.Pose()
            # You can still use the classical getPosition for your application, just not that the position returned by this method
            # is the position without any GNSS/cameras fusion
            self.zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            #self.get_logger().info(f"Zed position: {zed_pose.get_translation().get()}")

        # Publish GNSS data to Fusion
        input_gnss = sl.GNSSData(in_radian=False)
        input_gnss.set_coordinates(self.curr_lat, self.curr_lon, self.curr_altitude, in_radian=False)
        input_gnss.latitude_std = 0.1
        input_gnss.longitude_std = 0.1
        input_gnss.altitude_std = 0.1
        input_gnss.position_covariances = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        input_gnss.gnss_status = sl.GNSS_STATUS.SINGLE.value
        input_gnss.gnss_mode = sl.GNSS_MODE.FIX_3D.value
        input_gnss.ts = self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)

        ingest_error = self.fusion.ingest_gnss_data(input_gnss)
        if ingest_error != sl.FUSION_ERROR_CODE.SUCCESS:
            self.get_logger().info(f"Error occurred when ingesting GNSSData: {ingest_error}")
        else:
            self.get_logger().info("\nSUCCESSFUL FUSION\n")

        fusion_error = self.fusion.process()
        if fusion_error == sl.FUSION_ERROR_CODE.SUCCESS:
            fused_position = sl.Pose()
            self.get_logger().info(f"Fused position: {fused_position}")
            current_state = self.fusion.get_position(fused_position)
            self.get_logger().info(f"Current state: {current_state}")
            current_geopose = sl.GeoPose()
            self.get_logger().info(f"Current geopose: {current_geopose}")
        else:
            self.get_logger().info(f"Fusion error: {fusion_error}")

        '''current_pose = sl.Pose()
        if self.zed.get_position(current_pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:

            # Extract translation
            translation = current_pose.get_translation(sl.Translation())
            translation = translation.get()

            # Extract rotation
            rotation = current_pose.get_rotation_vector()
            #self.get_logger().info(f"Translation: {translation}")
            #self.get_logger().info(f"Rotation: {rotation}")

            # Create and publish the TwistStamped message
            pose_msg = TwistStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"  # World frame

            # Set linear velocities (relative translation)
            pose_msg.twist.linear.x = translation[2]
            pose_msg.twist.linear.y = -1.0 * translation[0]
            pose_msg.twist.linear.z = translation[1]

            # Set angular velocities (relative rotation)
            #pose_msg.twist.angular.x = roll
            #pose_msg.twist.angular.y = pitch
            pose_msg.twist.angular.z = -1.0 * rotation[1]

            self.pose_publisher.publish(pose_msg)'''

    def publish_transform(self):
        """
        Publishes a static transform from `map` to `zed_camera_frame`
        for visualization in RVIZ
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Parent frame
        t.child_frame_id = "zed_camera_frame"  # Camera frame

        # Static translation (camera is 1.395m above the ground)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.6

        # Quaternion rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.70710678
        t.transform.rotation.z = -0.70710678
        t.transform.rotation.w = 0.0

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
    
    def get_north_west_meters(self, lat1, lon1, lat2, lon2):
        """
        Converts latitude and longitude to north and west displacements in meters.
        """
        R = 6378137.0
        lat1_rad, lat2_rad = map(math.radians, [lat1, lat2])
        lon1_rad, lon2_rad = map(math.radians, [lon1, lon2])

        delta_lat = lat2_rad - lat1_rad
        delta_lon = lon2_rad - lon1_rad
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        delta_north = delta_lat * R
        delta_east = delta_lon * R * math.cos(mean_lat)
        return delta_north, -delta_east  # West is negative east
    
    def destroy_node(self):
        self.zed.close()
        self.fusion.close()
    
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
