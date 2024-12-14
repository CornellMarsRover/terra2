import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import math
import numpy as np

class ZEDCameraPublisher(Node):
    def __init__(self):
        super().__init__('zed_camera_publisher')

        # Create publishers for left and right images
        self.publisher_left = self.create_publisher(Image, '/zed/image_left', 10)
        self.publisher_right = self.create_publisher(Image, '/zed/image_right', 10)
        self.plane_publisher = self.create_publisher(Float32MultiArray, '/zed/plane/angles', 10)
        self.plane_request = self.create_subscription(
            Int32MultiArray,
            '/zed/plane/pixel',
            self.plane_request_callback,
            10
        )
        
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.AUTO  # Automatic resolution
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Use ULTRA depth mode
        # Use a right-handed Z-up coordinate system
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.camera_fps = 30  # Set fps to 30

        # Open the camera
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Failed to open ZED camera. Exiting.")
            rclpy.shutdown()

        # Enable positional tracking with default parameters.
        # Positional tracking needs to be enabled before using spatial mapping
        py_transform = sl.Transform()
        tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
        err = self.zed.enable_positional_tracking(tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Enable positional tracking : {repr(err)}. Exit program.")
            self.zed.close()
            exit()

        # Enable spatial mapping
        mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.MESH)
        err = self.zed.enable_spatial_mapping(mapping_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            print("Enable spatial mapping : "+repr(err)+". Exit program.")
            self.zed.close()
            exit(1)

        # Create Mat objects for images
        self.image_left = sl.Mat()
        self.image_right = sl.Mat()

        # Runtime parameters
        self.runtime_params = sl.RuntimeParameters()

        # Create a timer to publish images
        self.timer = self.create_timer(0.100, self.publish_images)  # 10 FPS (100ms interval)

    def publish_images(self):
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Retrieve images from ZED
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)

            # Convert images to numpy arrays
            left_image_np = self.image_left.get_data()
            right_image_np = self.image_right.get_data()

            # Convert numpy arrays to ROS Image messages
            left_image_msg = self.bridge.cv2_to_imgmsg(left_image_np, encoding="bgra8")
            right_image_msg = self.bridge.cv2_to_imgmsg(right_image_np, encoding="bgra8")

            # Publish the images
            self.publisher_left.publish(left_image_msg)
            self.publisher_right.publish(right_image_msg)
            
            #self.get_logger().info("Published left and right images.")

        else:
            self.get_logger().warning("Failed to grab images from ZED camera.")

    def plane_request_callback(self,msg):
        
        message_data = []
        message = Float32MultiArray()

        for i in range(0, len(msg.data), 3):
            index = float(msg.data[i]) # Aruco marker index
            coord = [msg.data[i+1], msg.data[i+2]] # Coordinates from the full size image
            
            plane = sl.Plane() # Structure that stores the estimated plane
            pose = sl.Pose()
            plane_parameters = sl.PlaneDetectionParameters()

            if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
                tracking_state = self.zed.get_position(pose) # Get the tracking state of the camera

            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                # Detect the plane passing by the depth value of pixel coord
                find_plane_status = self.zed.find_plane_at_hit(coord, plane, plane_parameters)
                if find_plane_status == sl.ERROR_CODE.SUCCESS:
                    normal = plane.get_normal() # Get the normal vector of the detected plane
                    #plane_equation = plane.get_plane_equation() # Get (a,b,c,d) where ax+by+cz=d
                    #self.get_logger().info(f"Plane normal: {normal}")
                    roll, pitch, yaw = self.compute_roll_pitch_yaw(normal)
                    self.get_logger().info(f"ARUCO ID: {index}   Yaw: {yaw}, Pitch: {pitch}, Roll: {roll}")
                    message_data.extend([index, yaw, pitch, roll])
                    
        if message_data != []:
            message.data = message_data
            self.plane_publisher.publish(message)


    def compute_roll_pitch_yaw(self, normal):
        """
        Compute roll, pitch, and yaw required to align a plane's normal vector to [0, 0, 1].

        Parameters:
            normal (list or np.ndarray): The normal vector of the rotated plane [nx, ny, nz].

        Returns:
            roll, pitch, yaw (floats): The angles in degrees.
        """
        # Normalize the input normal vector
        normal = normal / np.linalg.norm(normal)
        nx, ny, nz = normal

        # Compute pitch (rotation about Y-axis)
        pitch = np.arcsin(-nx)

        # Compute yaw (rotation about Z-axis)
        yaw = np.arctan2(ny, nz)

        # Roll is assumed to be zero as it doesn't affect the normal vector alignment
        roll = 0.0

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)



    '''def depth_request_callback(self,msg):
        depth = sl.Mat()
        point_cloud = sl.Mat()
        self.zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        # Retrieve colored point cloud. Point cloud is aligned on the left image.
        self.zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

        # Get and print distance value in mm at the center of the image
        # We measure the distance camera - object using Euclidean distance
        x = msg.data[0]
        y = msg.data[1]
        dx = [1, -1, 0, 0]
        dy = [0, 0, 1, -1]
        for i in range(4):
            err, point_cloud_value = point_cloud.get_value(x, y)
            
            if math.isfinite(point_cloud_value[2]):
                self.get_logger().info(f"Publishing cartesian point: {point_cloud_value}")
                coords = [point_cloud_value[0], point_cloud_value[1], point_cloud_value[2], float(msg.data[0]), float(msg.data[1])]
                cartesian_msg = Float32MultiArray()
                cartesian_msg.data = coords
                self.depth_publisher.publish(cartesian_msg)
                return
            x += dx[i]
            y += dy[i]

        message = Float32MultiArray()
        message.data = None
        self.depth_publisher.publish(message)
        self.get_logger().info("Unable to retrieve point cloud data for requested point")'''

    def destroy_node(self):
        # Close the ZED camera when the node is destroyed
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZEDCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
