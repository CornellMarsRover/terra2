#!/usr/bin/env python3
"""
Autonomous Typing Mission: Publishes Pose messages for robotic arm movement.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import json
import math

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

# --- Load Key Positions ---
def load_key_positions(filename):
    with open(filename, "r") as f:
        return json.load(f)

# --- Main Typing Mission ---
class TypingMission(Node):
    def __init__(self):
        super().__init__('typing_mission')

        # Load key positions from JSON
        self.key_positions = load_key_positions("key_position.json")

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()

        # Reference frame initialization
        self.corner_tag_ids = {0, 1, 2, 3}   # <-- set these to your 4 corner IDs
        self.reference_center = None
        self.reference_depth = 0.2  # Example depth (meters)

        # Publisher for Pose messages
        self.pub_pose = self.create_publisher(Pose, '/arm_target_pose', 10)

        # Hardcoded key for testing
        self.hardcoded_key = "CMR"  # Replace with the desired key

    def detect_aruco_tags(self, frame):
        """Detect ArUco tags and return their IDs and corners."""
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.detector_params)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return corners, ids

    def initialize_reference_frame(self, corners, ids):
        """Initialize the reference frame using the center of the 4 corner ArUco tags."""
        if ids is None:
            self.get_logger().error("No ArUco tags detected.")
            return False

        ids_flat = ids.flatten().tolist()

        # Need all 4 corner tags
        if not self.corner_tag_ids.issubset(set(ids_flat)):
            self.get_logger().error("Not all four corner ArUco tags detected.")
            return False

        # Compute each tag center (mean of its 4 corner points), then average the 4 tag centers
        tag_centers = []
        for i, marker_id in enumerate(ids_flat):
            if marker_id in self.corner_tag_ids:
                c = np.mean(corners[i].reshape(4, 2), axis=0)  # (x,y) in image coords
                tag_centers.append(c)

        self.reference_center = np.mean(np.array(tag_centers), axis=0)
        self.get_logger().info(f"Initialized reference frame to keyboard center from tags {sorted(self.corner_tag_ids)}.")
        return True    

    def compute_key_pose(self, key):
        """Compute the Pose message for the given key."""
        if self.reference_center is None:
            self.get_logger().error("Reference frame not initialized.")
            return None

        if key not in self.key_positions:
            self.get_logger().error(f"Key '{key}' not found in key_positions.json.")
            return None

        # Get relative key position
        rel_x, rel_y = self.key_positions[key]["x"], self.key_positions[key]["y"]

        # Compute global coordinates (relative to top-left ArUco tag)
        global_x = self.reference_center[0] + rel_x
        global_y = self.reference_center[1] + rel_y
        global_z = self.reference_depth  # Fixed depth for now

        # Create Pose message
        pose = Pose()
        pose.position.x = global_x
        pose.position.y = global_y
        pose.position.z = global_z

        # Fixed orientation (pointing down)
        roll, pitch, yaw = 0.0, math.pi, 0.0  # Pointing down
        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return pose

    def publish_key_pose(self, key):
        """Publish the Pose message for the given key."""
        pose = self.compute_key_pose(key)
        if pose:
            self.pub_pose.publish(pose)
            self.get_logger().info(f"Published Pose for key '{key}': {pose}")

    def run(self):
        """Main loop: Detect ArUco tags and publish key poses."""
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Could not open camera.")
            return

        # Hardcoded string to type
        keys_to_type = self.hardcoded_key
        current_index = 0

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue

                # Detect ArUco tags
                corners, ids = self.detect_aruco_tags(frame)

                # Initialize reference frame if not already done
                if self.reference_center is None:
                    if self.initialize_reference_frame(corners, ids):
                        self.get_logger().info("Reference frame initialized.")

                # Publish the pose for the current key
                if current_index < len(keys_to_type):
                    current_key = keys_to_type[current_index]
                    self.publish_key_pose(current_key)
                    current_index += 1  # Move to the next key

                # Show the camera feed
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