#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading
import time
import math

def rotation_matrix_from_ypr(yaw_deg, pitch_deg, roll_deg):
    """
    Returns a 3x3 rotation matrix for given yaw, pitch, roll (in degrees).
    - Yaw about Z axis
    - Pitch about Y axis
    - Roll about X axis
    """
    # Convert degrees to radians
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    # Rotation about X (roll)
    Rx = np.array([
        [1,           0,            0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll),  math.cos(roll)]
    ], dtype=np.float32)

    # Rotation about Y (pitch)
    Ry = np.array([
        [ math.cos(pitch), 0, math.sin(pitch)],
        [               0, 1,              0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ], dtype=np.float32)

    # Rotation about Z (yaw)
    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw),  math.cos(yaw), 0],
        [           0,              0, 1]
    ], dtype=np.float32)

    # Combined rotation: Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    return R

def compute_image_to_ground_homography(K, R, t):
    """
    Compute the 3x3 homography that maps (u,v) in the *undistorted* image
    to (x,y) on the ground plane z=0, in meters.

    Ground plane normal = [0,0,1].
    """
    # For the plane z = 0, a 3D point is (X, Y, 0).
    # The camera extrinsic transforms it to camera coords = R*[X,Y,0]^T + t.
    # Then image coords = K * camera_coords.
    #
    # => ground->image is H_gi = K * [R_2x2  t_xy]
    # We'll invert that to get image->ground.
    #
    # More explicitly:
    #   ground->image:  [u, v, 1]^T ~ H * [X, Y, 1]^T
    #   H = K * [r1 r2 t]
    #   (r1, r2) are the 1st two columns of R
    #   t is the translation
    #
    # We want image->ground, so H_ig = H_gi^{-1}.
    
    r1 = R[:,0:1]  # first column, shape (3,1)
    r2 = R[:,1:2]  # second column
    # t is shape (3,1)

    # Construct [r1 r2 t]
    Rt = np.hstack([r1, r2, t])  # shape (3,3)

    # ground->image
    H_ground_to_image = K @ Rt

    # image->ground
    H_image_to_ground = np.linalg.inv(H_ground_to_image)
    return H_image_to_ground

def compute_ground_to_mosaic_transform(m_per_pix=0.01, image_size=(300,300)):
    """
    Returns a 3x3 transform that maps (x,y) in ground meters
    to (u,v) in a mosaic image of given size.
    - m_per_pix: scale in meters/pixel (e.g. 3m / 300px = 0.01).
      Inverse of that is pix_per_m.
    - image_size = (width, height).

    We center (x=0, y=0) at the center of the mosaic.
    So output = (u, v) = (center_u + x*pix_per_m, center_v - y*pix_per_m).
      (minus sign if we want +Y up in the mosaic, or +Y down—adjust as you prefer)
    """
    w, h = image_size
    pix_per_m = 1.0 / m_per_pix
    cx, cy = w/2.0, h/2.0

    # If we define (x forward => +u, y left => +v),
    # Then (u = cx + x*ppm, v = cy + y*ppm).
    # We'll keep y going downward for simplicity or invert it if you want.
    #
    # Let's keep y left => +v:
    #   u = cx + x*ppm
    #   v = cy + y*ppm
    #
    # The 3x3 transform:
    #   [pix_per_m,       0, cx]
    #   [0,       pix_per_m, cy]
    #   [0,               0,  1]

    T = np.array([
        [ pix_per_m, 0,         cx],
        [0,          pix_per_m, cy],
        [0,          0,         1 ]
    ], dtype=np.float32)

    return T

class BirdseyePublisher(Node):
    def __init__(self):
        super().__init__('birdseye_publisher')

        # === Camera intrinsics & distortion (example) ===
        self.K = np.array([
            [1102.3128,  0.0,         1239.31013],
            [0.0,        880.524776,  576.711558],
            [0.0,        0.0,         1.0]
        ], dtype=np.float32)

        self.D = np.array([
            -0.19872862,
             1.00295506,
            -2.21112719,
             1.71318476
        ], dtype=np.float32)

        # We will use the same resolution for all cameras
        self.resolution = (640, 480)
        self.publish_rate = 10

        self.bridge = CvBridge()
        self.running = True

        # We'll set up 4 cameras
        self.camera_ids = [0, 2, 4, 6]  # front, left, back, right
        self.frames = {}
        self.capture_threads = {}

        # Publishers for corrected camera images
        self.rect_publishers = {
            cam_id: self.create_publisher(Image, f"/camera_{cam_id}/image_rect", 10)
            for cam_id in self.camera_ids
        }

        # 1) We'll compute each camera's extrinsics (R,t)
        # 2) Compute image->ground
        # 3) Compute ground->mosaic
        # 4) Combine => image->mosaic
        self.H_matrices = self._compute_all_homographies()

        # Start capture threads
        for cam_id in self.camera_ids:
            thr = threading.Thread(target=self._capture_loop, args=(cam_id,), daemon=True)
            thr.start()
            self.capture_threads[cam_id] = thr

        # Publisher for final mosaic
        self.birdseye_pub = self.create_publisher(Image, "/birdseye/image_raw", 10)

        # Timer for fusing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._publish_birdseye)

        self.get_logger().info("BirdseyePublisher initialized.")

    def _capture_loop(self, cam_id):
        cap = cv2.VideoCapture(cam_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        cap.set(cv2.CAP_PROP_FPS,         self.publish_rate)

        if not cap.isOpened():
            self.get_logger().error(f"Failed to open /dev/video{cam_id}")
            return
        
        rate = 1.0 / self.publish_rate
        mapx, mapy = cv2.initUndistortRectifyMap(self.K, self.D, None, self.K, self.resolution, cv2.CV_32FC1)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K, self.D, np.eye(3), self.K, self.resolution, cv2.CV_16SC2)
        
        while self.running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f"Camera {cam_id}: failed to read frame.")
                time.sleep(rate)
                continue

            # Undistort
            undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            # Rotate image 180 degrees
            undistorted = cv2.rotate(undistorted, cv2.ROTATE_180)

            # Store for stitching
            self.frames[cam_id] = undistorted

            # Publish the rectified image
            try:
                msg = self.bridge.cv2_to_imgmsg(undistorted, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"camera_{cam_id}"
                self.rect_publishers[cam_id].publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error publishing rectified image from camera {cam_id}: {e}")

            time.sleep(rate)

        cap.release()
        self.get_logger().info(f"Camera {cam_id}: capture loop stopped.")


    def _compute_all_homographies(self):
        """
        Compute the final image->mosaic homography for each camera, given:
         - The known (yaw,pitch,roll) and translation for each camera
         - A 3x3m mosaic, scaled to some pixel size
        """
        # 3x3 meter area => let's pick 300x300 pixels => 0.01 m/pixel
        m_per_pix = 0.01
        mosaic_size = (300, 300)

        # The ground->mosaic transform
        M_ground_to_mosaic = compute_ground_to_mosaic_transform(m_per_pix, mosaic_size)

        # We'll define the positions/orientations for each camera
        # 1) front
        front_pose = {
            'translation': (0.35, 0.0, 0.42),
            'yaw':   0.0,
            'pitch': -11.0,  # 11° downward
            'roll':  0.0
        }
        # 2) left
        left_pose = {
            'translation': (0.0, 0.35, 0.42),
            'yaw':   90.0,
            'pitch': -11.0,
            'roll':  0.0
        }
        # 3) back
        back_pose = {
            'translation': (-0.35, 0.0, 0.42),
            'yaw':   180.0,
            'pitch': -11.0,
            'roll':  0.0
        }
        # 4) right
        right_pose = {
            'translation': (0.0, -0.35, 0.42),
            'yaw':   -90.0,
            'pitch': -11.0,
            'roll':  0.0
        }

        # Build dictionary keyed by camera_id
        # (You can reorder if your actual camera->id mapping is different.)
        camera_poses = {
            0: front_pose,
            2: left_pose,
            4: back_pose,
            6: right_pose
        }

        H_matrices = {}
        for cam_id, pose in camera_poses.items():
            # 1) Rotation matrix
            R_cam = rotation_matrix_from_ypr(
                yaw_deg=pose['yaw'],
                pitch_deg=pose['pitch'],
                roll_deg=pose['roll']
            )
            # 2) Translation vector (3x1)
            tx, ty, tz = pose['translation']
            t_cam = np.array([[tx], [ty], [tz]], dtype=np.float32)

            # 3) image->ground
            H_img_to_ground = compute_image_to_ground_homography(self.K, R_cam, t_cam)

            # 4) Combine with ground->mosaic => image->mosaic
            H_img_to_mosaic = M_ground_to_mosaic @ H_img_to_ground

            H_matrices[cam_id] = H_img_to_mosaic

        return H_matrices

    def _publish_birdseye(self):
        # We need frames from all cameras
        if len(self.frames) < len(self.camera_ids):
            return

        # Create mosaic background
        mosaic_w, mosaic_h = 300, 300
        mosaic = np.ones((mosaic_h, mosaic_w, 3), dtype=np.uint8) * 255

        # Draw a black square in the center to represent the robot,
        # which is 70 cm on a side => 0.70 m => 70 px if scale=0.01 m/px
        # 70 cm => 70 px at 0.01 m/px
        robot_size_px = int(0.70 / 0.01)  # 70
        center_x, center_y = mosaic_w // 2, mosaic_h // 2

        cv2.rectangle(
            mosaic,
            (center_x - robot_size_px//2, center_y - robot_size_px//2),
            (center_x + robot_size_px//2, center_y + robot_size_px//2),
            (0,0,0),
            thickness=-1
        )

        # Warp each camera's image into the mosaic
        for cam_id, frame in self.frames.items():
            H = self.H_matrices[cam_id]
            warped = cv2.warpPerspective(
                frame, H, (mosaic_w, mosaic_h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(255,255,255)
            )
            # Simple overlay
            mask = np.any(warped < 250, axis=-1)
            mosaic[mask] = warped[mask]

        # Publish
        ros_img = self.bridge.cv2_to_imgmsg(mosaic, encoding='bgr8')
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "birdseye"
        self.birdseye_pub.publish(ros_img)

    def stop_all(self):
        self.running = False
        for cam_id, thr in self.capture_threads.items():
            if thr.is_alive():
                thr.join()
        self.get_logger().info("All capture threads stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = BirdseyePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
