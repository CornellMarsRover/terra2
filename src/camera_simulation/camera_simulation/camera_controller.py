#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#from webots_ros2_msgs.msg import Float64Stamped
import numpy as np
import rclpy

class CameraController:
    def __init__(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # Get the camera device
        self.__camera = self.__robot.getDevice('camera')
        self.__camera.enable(self.__timestep)

        # Initial position and orientation
        self.__position = [0.0, 0.0, 0.0]
        self.__direction = [1.0, 0.0, 0.0]  # Pointing along +x
        self.__rotation = [1.0, 0.0, 0.0, 0.0]

        rclpy.init(args=None)
        # Initialize the CV Bridge
        self.__bridge = CvBridge()
        self.__node = rclpy.create_node('camera_controller')
        # Subscribers and Publishers
        self.__node.create_subscription(
            Float32MultiArray,
            '/arm/end_effector/pose',
            self.__pose_callback,
            10
        )
        self.__image_publisher = self.__node.create_publisher(
            Image,
            '/camA/image_raw',
            10
        )

    def __pose_callback(self, msg):
        data = msg.data
        if len(data) != 6:
            self.__node.get_logger().error('Pose message must contain 6 values.')
            return

        self.__position = data[:3]
        self.__direction = data[3:]
        self.__update_camera_pose()

    def __publish_image(self):
        # Capture and publish the image
        image = self.__camera.getImage()
        if image:
            img_msg = self.bridge.cv2_to_imgmsg(
                np.frombuffer(image, np.uint8).reshape(
                    (self.camera.getHeight(), self.camera.getWidth(), 4)
                )[:, :, :3],
                encoding='bgr8'
            )
            self.__image_publisher.publish(img_msg)

    def __update_camera_pose(self):
        # Compute the rotation quaternion from direction vector
        direction = np.array(self.__direction)
        direction = direction / np.linalg.norm(direction)

        # Assuming up vector is [0, 0, 1]
        up = np.array([0, 0, 1])
        right = np.cross(direction, up)
        up = np.cross(right, direction)

        rotation_matrix = np.array([right, direction, up]).T
        self.__rotation = self.__rotation_from_matrix(rotation_matrix)


    def __rotation_from_matrix(self, matrix):
        # Convert rotation matrix to axis-angle representation
        angle = np.arccos((np.trace(matrix) - 1) / 2)
        if angle == 0:
            axis = [1, 0, 0]
        else:
            axis = [(matrix[2,1] - matrix[1,2]),
                    (matrix[0,2] - matrix[2,0]),
                    (matrix[1,0] - matrix[0,1])]
            axis = axis / (2 * np.sin(angle))
        return [axis[0], axis[1], axis[2], angle]

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__publish_image()
        # Update the robot's position and orientation
        self.__robot.getSelf().getField('translation').setSFVec3f(self.__position)
        self.__robot.getSelf().getField('rotation').setSFRotation(self.__rotation)