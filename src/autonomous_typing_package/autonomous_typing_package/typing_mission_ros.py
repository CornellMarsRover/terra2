#!/usr/bin/env python3
"""
Autonomous Typing Mission: Publishes PoseWithCovarianceStamped messages for robotic arm movement.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import cv2
import numpy as np
import math
from collections import deque

from ament_index_python.packages import get_package_share_directory
import os
import json

# --- Helper: Quaternion from RPY ---
def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + cr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

def load_key_positions(filename):
    pkg_share = get_package_share_directory('autonomous_typing_package')
    filepath = os.path.join(pkg_share, filename)
    with open(filepath, 'r') as f:
        return json.load(f)


# --- Main Typing Mission ---
class TypingMission(Node):

    # How many frames to track for stability
    STABILITY_WINDOW = 20

    # Maximum allowed per-axis variance (in pixels squared) to consider the
    # reference origin stable. Tune this to your camera resolution / use-case.
    VARIANCE_THRESHOLD = 2.0

    def __init__(self):
        super().__init__('typing_mission')

        # Load key positions from JSON
        self.key_positions = load_key_positions("key_position.json")

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters_create()

        # Reference frame state
        self.top_left_tag_id = 0
        self.reference_origin = None
        self.reference_depth = 0.2  # meters

        # Rolling window of recent reference origins for stability estimation
        # Each entry is a (x, y) pixel coordinate of the top-left tag centre
        self.origin_history = deque(maxlen=self.STABILITY_WINDOW)

        # Once we lock a high-confidence pose, we stop publishing
        self.pose_locked = False

        # Publisher — PoseWithCovarianceStamped so the coordinator can read
        # confidence and tf2 can transform using the header frame_id
        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/key_location', 10)

        # Hardcoded key for testing
        self.hardcoded_key = "CMR"

    # ------------------------------------------------------------------
    # ArUco helpers
    # ------------------------------------------------------------------

    def detect_aruco_tags(self, frame):
        """Detect ArUco tags and return their IDs and corners."""
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.detector_params)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return corners, ids

    def initialize_reference_frame(self, corners, ids):
        """
        Update the rolling window with the current top-left tag position.
        Returns True if the tag was found this frame.
        """
        if ids is None or len(ids) < 4:
            # Not all tags visible — don't add to history so variance stays high
            return False

        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == self.top_left_tag_id:
                origin = np.mean(corners[i].reshape(4, 2), axis=0)
                self.origin_history.append(origin)
                # Keep reference_origin as the latest observation for pose
                # computation; the coordinator decides when to trust it.
                self.reference_origin = origin
                return True

        return False

    # ------------------------------------------------------------------
    # Confidence estimation
    # ------------------------------------------------------------------

    def compute_origin_variance(self):
        """
        Return (var_x, var_y) of the reference origin over the history window.
        Returns large values if the window isn't full yet.
        """
        if len(self.origin_history) < self.STABILITY_WINDOW:
            # Not enough data — report high uncertainty
            return float('inf'), float('inf')

        history = np.array(self.origin_history)   # shape (N, 2)
        var_x = float(np.var(history[:, 0]))
        var_y = float(np.var(history[:, 1]))
        return var_x, var_y

    def is_confident(self, var_x, var_y):
        return max(var_x, var_y) < self.VARIANCE_THRESHOLD

    # ------------------------------------------------------------------
    # Pose computation & publishing
    # ------------------------------------------------------------------

    def compute_key_pose_stamped(self, key, var_x, var_y):
        """
        Build a PoseWithCovarianceStamped for the given key.
        frame_id is set to 'camera_frame' so tf2 can transform it.
        Diagonal covariance entries reflect measured pixel variance
        (caller should convert to metres if needed; kept as pixels here
        for simplicity — adjust to your calibration).
        """
        if self.reference_origin is None:
            self.get_logger().error("Reference frame not initialised.")
            return None

        if key not in self.key_positions:
            self.get_logger().error(f"Key '{key}' not found in key_positions.json.")
            return None

        rel_x = self.key_positions[key]["x"]
        rel_y = self.key_positions[key]["y"]

        global_x = self.reference_origin[0] + rel_x
        global_y = self.reference_origin[1] + rel_y
        global_z = self.reference_depth

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_frame"  # tf2 transforms FROM this frame

        msg.pose.pose.position.x = global_x
        msg.pose.pose.position.y = global_y
        msg.pose.pose.position.z = global_z

        roll, pitch, yaw = 0.0, math.pi, 0.0
        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # 6x6 covariance matrix (row-major: x, y, z, rx, ry, rz)
        # We only populate the position diagonal; orientation left as zero.
        msg.pose.covariance[0]  = var_x   # x variance
        msg.pose.covariance[7]  = var_y   # y variance
        msg.pose.covariance[14] = 0.0     # z variance — depth is fixed/known

        return msg

    def publish_key_pose(self, key):
        """Publish pose with current confidence. Locks after threshold is met."""
        var_x, var_y = self.compute_origin_variance()
        confident = self.is_confident(var_x, var_y)

        msg = self.compute_key_pose_stamped(key, var_x, var_y)
        if msg is None:
            return

        self.pub_pose.publish(msg)

        if confident:
            self.get_logger().info(
                f"High-confidence pose published for '{key}' "
                f"(var_x={var_x:.4f}, var_y={var_y:.4f}). Locking.")
            self.pose_locked = True
        else:
            self.get_logger().debug(
                f"Low confidence for '{key}' "
                f"(var_x={var_x:.4f}, var_y={var_y:.4f}). Still accumulating.")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Could not open camera.")
            return

        keys_to_type = self.hardcoded_key
        current_index = 0

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue

                corners, ids = self.detect_aruco_tags(frame)
                self.initialize_reference_frame(corners, ids)

                # Only publish if reference is initialised and pose not yet locked
                if self.reference_origin is not None and not self.pose_locked:
                    current_key = keys_to_type[current_index]
                    self.publish_key_pose(current_key)

                # Overlay confidence info on the frame
                var_x, var_y = self.compute_origin_variance()
                status = "LOCKED" if self.pose_locked else (
                    "HIGH CONF" if self.is_confident(var_x, var_y) else "ACCUMULATING")
                cv2.putText(frame, f"Status: {status}  var=({var_x:.2f},{var_y:.2f})",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

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