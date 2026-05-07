#!/usr/bin/env python3
"""
Autonomous Typing Mission: detects keyboard via ArUco tags and publishes
key poses in camera_frame so TF2 can transform them to base_link via
the live end_effector → base_link chain.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import cv2
import numpy as np
import math
from collections import deque

from ament_index_python.packages import get_package_share_directory
import os
import json


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def load_key_positions(filename):
    pkg_share = get_package_share_directory('autonomous_typing_package')
    with open(os.path.join(pkg_share, filename), 'r') as f:
        return json.load(f)


class TypingMission(Node):

    STABILITY_WINDOW   = 20
    VARIANCE_THRESHOLD = 1e-6  # metres² — tight because ArUco gives real units

    def __init__(self):
        super().__init__('typing_mission')

        self.declare_parameter("camera_index",  0)
        self.declare_parameter("camera_device", "")

        # Arducam 8MP (IMX219) at 640x480.
        # Estimated from sensor specs: fx = f_mm / pixel_pitch_mm = 3.04 / 0.00571 ≈ 532
        self.declare_parameter("camera_fx", 532.0)
        self.declare_parameter("camera_fy", 532.0)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)

        # Physical side-length of each ArUco marker in metres.
        # Measure one printed marker with a ruler and set this.
        self.declare_parameter("aruco_marker_size_m", 0.03)

        self.camera_index    = self.get_parameter("camera_index").value
        self.camera_device   = self.get_parameter("camera_device").value
        self.fx              = self.get_parameter("camera_fx").value
        self.fy              = self.get_parameter("camera_fy").value
        self.cx              = self.get_parameter("camera_cx").value
        self.cy              = self.get_parameter("camera_cy").value
        self.marker_size_m   = self.get_parameter("aruco_marker_size_m").value

        self.camera_matrix = np.array([[self.fx, 0,       self.cx],
                                       [0,       self.fy, self.cy],
                                       [0,       0,       1      ]], dtype=np.float64)
        self.dist_coeffs   = np.zeros((4, 1))  # assume no lens distortion

        self.key_positions = load_key_positions("key_position.json")

        self.aruco_dict      = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()

        self.corner_tag_ids = {0, 1, 2, 3}

        # Rolling window of keyboard-centre positions in camera frame (metres)
        self.origin_history  = deque(maxlen=self.STABILITY_WINDOW)
        self.reference_tvec  = None  # most recent 3-D keyboard centre (camera frame, metres)

        self.pose_locked      = False
        self.mission_complete = False

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/key_location', 10)

        self.create_subscription(String, '/coordinator_state', self._coordinator_state_cb, 10)
        self._last_coordinator_state = ''

        self.hardcoded_key     = "CMR"
        self.current_key_index = 0

    # ------------------------------------------------------------------

    def _coordinator_state_cb(self, msg: String):
        state = msg.data
        if state == 'IDLE' and self._last_coordinator_state not in ('IDLE', ''):
            self.pose_locked    = False
            self.origin_history.clear()
            self.reference_tvec = None
            self.current_key_index += 1
            if self.current_key_index >= len(self.hardcoded_key):
                self.get_logger().info("All keys typed. Mission complete.")
                self.mission_complete = True
            else:
                self.get_logger().info(
                    f"Key typed. Next: '{self.hardcoded_key[self.current_key_index]}'"
                )
        self._last_coordinator_state = state

    # ------------------------------------------------------------------

    def detect_and_estimate(self, frame):
        """
        Detect ArUco tags and run pose estimation.
        Returns (corners, ids, rvecs, tvecs) — all None on failure.
        """
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.detector_params)
        if ids is None or len(ids) == 0:
            return None, None, None, None

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size_m, self.camera_matrix, self.dist_coeffs)

        return corners, ids, rvecs, tvecs

    def update_reference_frame(self, ids, tvecs):
        """
        Average the tvecs of the 4 corner ArUco tags to get the keyboard
        centre position in camera frame (real metres from ArUco pose estimation).
        """
        if ids is None:
            return False

        ids_flat = ids.flatten().tolist()
        if not self.corner_tag_ids.issubset(set(ids_flat)):
            return False

        corner_tvecs = []
        for i, marker_id in enumerate(ids_flat):
            if marker_id in self.corner_tag_ids:
                corner_tvecs.append(tvecs[i][0])  # shape (3,)

        centre = np.mean(np.array(corner_tvecs), axis=0)  # (x, y, z) in metres
        self.origin_history.append(centre)
        self.reference_tvec = centre
        return True

    # ------------------------------------------------------------------

    def compute_origin_variance(self):
        if len(self.origin_history) < self.STABILITY_WINDOW:
            return float('inf'), float('inf')
        history = np.array(self.origin_history)   # (N, 3)
        return float(np.var(history[:, 0])), float(np.var(history[:, 1]))

    def is_confident(self, var_x, var_y):
        return max(var_x, var_y) < self.VARIANCE_THRESHOLD

    # ------------------------------------------------------------------

    def compute_key_pose_stamped(self, key, var_x, var_y):
        if self.reference_tvec is None:
            self.get_logger().error("Reference frame not initialised.")
            return None
        if key not in self.key_positions:
            self.get_logger().error(f"Key '{key}' not in key_positions.json.")
            return None

        # key_position.json is in centimetres; convert to metres
        key_x_m = self.key_positions[key]["x"] * 0.01
        key_y_m = self.key_positions[key]["y"] * 0.01

        # Reference tvec is the keyboard centre in camera frame (metres).
        # Key offsets are along camera X (right) and camera Y (down),
        # which matches the key_position.json convention.
        x = float(self.reference_tvec[0]) + key_x_m
        y = float(self.reference_tvec[1]) + key_y_m
        z = float(self.reference_tvec[2])

        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z

        roll, pitch, yaw = 0.0, math.pi, 0.0
        qx, qy, qz, qw  = quat_from_rpy(roll, pitch, yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance[0]  = var_x
        msg.pose.covariance[7]  = var_y
        msg.pose.covariance[14] = 0.0

        return msg

    def publish_key_pose(self, key):
        var_x, var_y = self.compute_origin_variance()
        msg = self.compute_key_pose_stamped(key, var_x, var_y)
        if msg is None:
            return
        self.pub_pose.publish(msg)
        if self.is_confident(var_x, var_y):
            self.get_logger().info(
                f"High-confidence pose for '{key}' "
                f"(var_x={var_x:.2e}, var_y={var_y:.2e}). Locking.")
            self.pose_locked = True
        else:
            self.get_logger().debug(
                f"Still accumulating for '{key}' "
                f"(var_x={var_x:.2e}, var_y={var_y:.2e}).")

    # ------------------------------------------------------------------

    def get_camera_source(self):
        if self.camera_device:
            return str(self.camera_device)
        try:
            return int(self.camera_index)
        except (TypeError, ValueError):
            return 0

    def run(self):
        source = self.get_camera_source()
        self.get_logger().info(f"Opening camera {source} at 640x480")
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f"Could not open camera {source}.")
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue

                rclpy.spin_once(self, timeout_sec=0.0)

                _, ids, rvecs, tvecs = self.detect_and_estimate(frame)
                self.update_reference_frame(ids, tvecs)

                if (self.reference_tvec is not None
                        and not self.pose_locked
                        and not self.mission_complete):
                    self.publish_key_pose(self.hardcoded_key[self.current_key_index])

                # Overlay
                var_x, var_y = self.compute_origin_variance()
                if self.mission_complete:
                    status = "DONE"
                elif self.pose_locked:
                    status = "LOCKED"
                elif self.is_confident(var_x, var_y):
                    status = "HIGH CONF"
                else:
                    status = "ACCUMULATING"
                key_label = (self.hardcoded_key[self.current_key_index]
                             if not self.mission_complete else '-')
                cv2.putText(
                    frame,
                    f"Key: {key_label}  {status}  var=({var_x:.1e},{var_y:.1e})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )

                cv2.imshow("Camera Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = TypingMission()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
