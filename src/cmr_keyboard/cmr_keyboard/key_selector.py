#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from geometry_msgs.msg import Twist
import time


class KeySelector(Node):
    def __init__(self):
        super().__init__('key_selector')

        # --- ROS pubs/subs --------------------------------------------------
        self.subscription = self.create_subscription(
            Image,
            '/zed/image_left',
            self.image_callback,
            10
        )
        self.keyboard_pub = self.create_publisher(Float32MultiArray, '/zed/keyboard/key_positions', 10)
        self.plane_pixel_pub = self.create_publisher(Int32MultiArray, '/zed/plane/pixel', 10)
        self.plane_equation_sub = self.create_subscription(
            Float32MultiArray,
            '/zed/plane/equation',
            self.plane_equation_callback,
            10
        )
        self.point_pixel_pub = self.create_publisher(Int32MultiArray, '/zed/point/pixel', 10)
        self.coordinate_sub = self.create_subscription(
            Float32MultiArray,
            '/zed/point/coordinate',
            self.coordinate_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.increment_pub = self.create_publisher(Float32MultiArray, '/arm/joint_increment', 10)

        self.bridge = CvBridge()
        self.latest_image = None

        # --- Data structures ------------------------------------------------
        # For user-marked keys:
        self.selected_keys = {}            # letter -> (pixel_x, pixel_y)
        self.plane_equations = {}          # letter -> list of plane eqs (a,b,c,d)

        # For the ArUco marker on the end-effector:
        self.marker_equations = {}         # marker_id -> list of plane eqs (a,b,c,d)
        self.marker_centers = {}           # marker_id -> (pixel_x, pixel_y)
        self.target_marker_id = 5

        # Current text we want to type:
        self.text_to_type = "ballsack"
        self.current_letter_index = 0
        self.current_char = self.text_to_type[self.current_letter_index]

        # Flag for whether we have started the autonomous typing
        self.typing = False

        # Use a named window and a mouse callback to mark keys
        cv2.namedWindow("Key Selector", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Key Selector", self.mouse_callback)

        # ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()

        self.current_letter = None

        # Timer used for periodic actions (rendering, etc.)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.k = 0

        # For end effector offset (new orientation => +x is forward, +z is up)
        # Adjust this as needed for your actual physical marker->EE geometry:
        self.end_effector_offset = np.array([0.11, 0.0, -0.20])

        # Where we store the final keyboard positions in the same global coords:
        self.key_positions_global = None

        # We'll store a 3D position for the marker => end effector
        self.end_effector_tip = None
        # We'll store a rotation matrix for the marker => global
        self.end_effector_rotation = None

        # We keep track of the last time we published a movement command:
        self.last_command_time = None

        # We also track the yaw or any angles from the marker plane normal
        self.yaw = None

        self.ee_updated = False


    # -----------------------------------------------------------------------
    # ------------------- SUBSCRIBER CALLBACKS -------------------------------
    # -----------------------------------------------------------------------
    def image_callback(self, msg):
        """Convert the incoming ROS image message to an OpenCV BGR image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def plane_equation_callback(self, msg):
        """
        Process plane equation messages from /zed/plane/equation.
        Format: [code, a, b, c, d]
        """
        data = msg.data
        if len(data) < 5:
            self.get_logger().error("Plane equation msg has insufficient data.")
            return

        code = int(data[0])
        a, b, c, d = data[1:5]

        # Marker plane eq (code >= 1000)
        if code >= 1000:
            marker_id = code - 1000
            if marker_id != self.target_marker_id:
                return
            # Store the plane eq
            if marker_id not in self.marker_equations:
                self.marker_equations[marker_id] = []
            self.marker_equations[marker_id].append((a, b, c, d))

            # Keep only the most recent 5 or 6 to avoid ballooning
            if len(self.marker_equations[marker_id]) > 6:
                self.marker_equations[marker_id].pop(0)

            # If we have at least 5, do a least-squares solve for the marker position
            if len(self.marker_equations[marker_id]) >= 5:
                self.update_marker_pose_from_planes(marker_id)

        else:
            # Letter keys (code < 1000)
            letter = chr(code).upper()
            if letter not in self.plane_equations:
                self.get_logger().warning(f"Received plane eq. for key '{letter}' with no pending request.")
                return
            self.plane_equations[letter].append((a, b, c, d))
            self.get_logger().info(
                f"Received plane eq. for key '{letter}': {a:.3f}x + {b:.3f}y + {c:.3f}z = {d:.3f}"
            )

    def coordinate_callback(self, msg):
        """
        If you are also using coordinate queries from the ZED (point cloud),
        this updates the end effector position from /zed/point/coordinate:
        Format: [code, x, y, z]
        """
        data = msg.data
        if len(data) < 4:
            self.get_logger().error("Coordinate msg has insufficient data.")
            return

        code = int(data[0])
        x, y, z = data[1:4]

        if code >= 1000:
            marker_id = code - 1000
            if marker_id == self.target_marker_id:
                # In the new coordinate system, we do NOT invert x,y,z:
                p_marker = np.array([x, y, z])

                # The tip is offset from the marker center
                self.end_effector_tip = p_marker + self.end_effector_offset
                self.get_logger().info(f"coordinate: {self.end_effector_tip}")
                self.ee_updated = True

    # -----------------------------------------------------------------------
    # ------------------- KEYBOARD / ARUCO LOGIC -----------------------------
    # -----------------------------------------------------------------------
    def mouse_callback(self, event, x, y, flags, param):
        """
        When the left mouse button is clicked while a letter key is active,
        record that pixel as that letter's key center and request plane eq.
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.current_letter is not None and self.current_letter.isalpha():
                letter = self.current_letter.upper()
                self.selected_keys[letter] = (x, y)
                self.get_logger().info(f"Marked key '{letter}' at pixel ({x}, {y})")
                self.request_plane_equations(letter, x, y)
            else:
                self.get_logger().info("Hold a letter key (A–Z) while clicking to mark a key.")

    def request_plane_equations(self, letter, x, y):
        """
        Publish 5 requests (center + neighbors) to /zed/plane/pixel to get plane eq for that letter key.
        We encode the letter as code=ord(letter).
        """
        self.plane_equations[letter] = []
        offsets = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]
        for dx, dy in offsets:
            msg = Int32MultiArray()
            msg.data = [ord(letter), x + dx, y + dy]
            self.plane_pixel_pub.publish(msg)
            self.get_logger().info(
                f"Requested plane eq. for key '{letter}' at pixel ({x+dx}, {y+dy})"
            )

    def request_marker_plane_equation(self, marker_id, x, y):
        """
        For the ArUco marker, publish 5 requests to /zed/plane/pixel so we can
        do an LSE solution for that marker’s plane in 3D.
        We encode marker IDs as 1000 + marker_id.
        """
        if marker_id != self.target_marker_id:
            return
        offsets = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]
        for dx, dy in offsets:
            msg = Int32MultiArray()
            msg.data = [1000 + marker_id, x + dx, y + dy]
            self.plane_pixel_pub.publish(msg)

    def request_marker_coordinate(self, marker_id, x, y):
        """
        For an ArUco marker, request the 3D coordinate from /zed/point/pixel.
        We'll also store that in the same marker-based offset approach.
        """
        if marker_id != self.target_marker_id:
            return
        offsets = [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)]
        for dx, dy in offsets:
            msg = Int32MultiArray()
            msg.data = [1000 + marker_id, x + dx, y + dy]
            self.point_pixel_pub.publish(msg)

    def update_marker_pose_from_planes(self, marker_id):
        """
        Given ~5 plane equations for the marker, do a least-squares solve to get
        the marker’s 3D center, compute orientation from the average normal, and
        store the end effector tip + rotation.
        """
        eqs = self.marker_equations[marker_id]
        A = []
        d_vals = []
        for (a, b, c, d_val) in eqs:
            A.append([a, b, c])
            d_vals.append(d_val)
        A = np.array(A)
        d_vals = np.array(d_vals)

        # Solve for point (x,y,z) such that a*x + b*y + c*z = d
        try:
            point, _, _, _ = np.linalg.lstsq(A, d_vals, rcond=None)
        except Exception as e:
            self.get_logger().error(f"Failed LSE on marker plane eq: {e}")
            return

        # Normal is average of the plane normals (a,b,c) or just use the first
        normal = np.mean(A, axis=0)
        normal = normal / np.linalg.norm(normal)

        # Rotation matrix so that 'normal' is along the global +Z axis
        # Because we want x forward, y left, z up
        # We'll treat normal as z-axis, pick any x-axis "forward" near horizontal
        z_axis = normal
        # A naive approach: we want x-axis to be "some horizontal direction"
        # We can attempt to keep x in horizontal plane by crossing z with global up or so
        # but a simpler approach is to pick global X and re-orthonormalize
        # We'll do a quick approach with a second guess:
        dummy_x = np.array([1, 0, 0])
        if abs(np.dot(z_axis, dummy_x)) > 0.99:
            dummy_x = np.array([0, 1, 0])
        x_axis = np.cross(dummy_x, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        R = np.column_stack((x_axis, y_axis, z_axis))

        self.end_effector_rotation = R

        # Marker center in global coords => end effector tip => offset in new coords
        # Just treat 'point' as the marker center:
        marker_center = point

        # Now the tip is marker_center + offset
        self.end_effector_tip = marker_center + self.end_effector_offset
        self.get_logger().info(f"plane: {self.end_effector_tip}")
        self.ee_updated = True

        # Save some angles if desired (roll, pitch, yaw)
        roll, pitch, yaw = self.compute_roll_pitch_yaw(z_axis)
        self.yaw = yaw

        self.get_logger().info(
            f"Marker updated. Center={marker_center}, normal={z_axis}, yaw={math.degrees(yaw):.2f} deg"
        )

    # -----------------------------------------------------------------------
    # ------------------- MAIN TIMER (Rendering, etc.) -----------------------
    # -----------------------------------------------------------------------
    def timer_callback(self):
        if self.latest_image is None:
            return

        display_image = self.latest_image.copy()
        self.k += 1
        # Detect ArUco markers:
        gray = cv2.cvtColor(display_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            for i, marker_id_arr in enumerate(ids):
                marker_id = int(marker_id_arr[0])
                if marker_id == self.target_marker_id:
                    marker_corners = corners[i]
                    center_x = int(np.mean(marker_corners[0][:, 0]))
                    center_y = int(np.mean(marker_corners[0][:, 1]))

                    cv2.polylines(display_image, [marker_corners.astype(np.int32)],
                                  True, (0, 0, 255), 2)
                    cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)
                    cv2.putText(display_image, f"ID: {marker_id}",
                                (center_x + 10, center_y), cv2.FONT_HERSHEY_SIMPLEX,
                                0.8, (0, 0, 255), 2)

                    # Always request coordinate from the pointcloud
                    self.request_marker_coordinate(marker_id, center_x, center_y)

                    # While not typing, also request plane eq to refine orientation
                    if not self.typing:
                        self.request_marker_plane_equation(marker_id, center_x, center_y)

        # Draw user-marked keys
        for letter, (px, py) in self.selected_keys.items():
            cv2.circle(display_image, (px, py), 5, (0, 255, 0), -1)
            cv2.putText(display_image, letter, (px + 10, py), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2)

        cv2.imshow("Key Selector", display_image)
        key = cv2.waitKey(1) & 0xFF

        # If we are in the middle of autonomous typing, do the next step
        if self.typing:
            if self.last_command_time is None:
                self.last_command_time = self.timestamp_to_float(self.get_clock().now().to_msg())

            target_pos = self.key_positions_global[self.current_char.upper()]
            pressed = self.type_char(target_pos)
            if pressed:
                # Move to next char
                self.current_letter_index += 1
                if self.current_letter_index >= len(self.text_to_type):
                    # Done typing, shut down
                    rclpy.shutdown()
                    cv2.destroyAllWindows()
                    return
                self.current_char = self.text_to_type[self.current_letter_index]
            return

        # Otherwise, we are in "calibration" mode (select keys, or press ENTER)
        if (65 <= key <= 90) or (97 <= key <= 122):
            self.current_letter = chr(key)
        elif key in [13, 10] and not self.typing:
            # Pressed Enter => compute keyboard positions from plane eq, start typing
            if len(self.selected_keys) > 0:
                self.process_key_selection()
                if self.key_positions_global is not None:
                    self.typing = True
                    self.get_logger().info("Starting autonomous typing...")
            else:
                self.get_logger().info("No keys have been marked yet.")

    # -----------------------------------------------------------------------
    # ------------------- HELPER FUNCTIONS -----------------------------------
    # -----------------------------------------------------------------------
    def process_key_selection(self):
        """
        Take the plane equations for each selected key, do an LSE to find 3D points,
        then compute a full US QWERTY layout in 3D.
        """
        self.get_logger().info("Processing key selection and extrapolating 3D positions...")
        measured_points = {}  # letter -> 3D position (np.array)

        for letter, _pixel in self.selected_keys.items():
            if letter not in self.plane_equations or len(self.plane_equations[letter]) < 3:
                self.get_logger().warning(f"Insufficient plane eq. data for '{letter}'.")
                continue
            A = []
            d_vals = []
            for (a, b, c, d_val) in self.plane_equations[letter]:
                A.append([a, b, c])
                d_vals.append(d_val)
            A = np.array(A)
            d_vals = np.array(d_vals)
            try:
                point, _, _, _ = np.linalg.lstsq(A, d_vals, rcond=None)
                measured_points[letter] = point
                self.get_logger().info(f"Computed 3D for key '{letter}': {point}")
            except Exception as e:
                self.get_logger().error(f"LSE failure for key '{letter}': {e}")

        if len(measured_points) < 2:
            self.get_logger().warning("At least two marked keys with plane eq are required.")
            return

        # A simplified US QWERTY layout in mm
        row1 = ['Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P']
        row2 = ['A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L']
        row3 = ['Z', 'X', 'C', 'V', 'B', 'N', 'M']

        layout = {}
        row1_width = 171.0
        row2_width = 152.0
        row3_width = 114.0
        vertical_spacing = 18.25
        row2_offset = 5.0
        row3_offset = 13.0

        spacing1 = row1_width / (len(row1) - 1) if len(row1) > 1 else 0
        spacing2 = row2_width / (len(row2) - 1) if len(row2) > 1 else 0
        spacing3 = row3_width / (len(row3) - 1) if len(row3) > 1 else 0

        for i, lt in enumerate(row1):
            layout[lt] = np.array([i * spacing1, 0.0])
        for i, lt in enumerate(row2):
            layout[lt] = np.array([row2_offset + i * spacing2, vertical_spacing])
        for i, lt in enumerate(row3):
            layout[lt] = np.array([row3_offset + i * spacing3, 2.0 * vertical_spacing])

        # Find two common keys to anchor the transform
        common_keys = [k for k in measured_points if k in layout]
        if len(common_keys) < 2:
            self.get_logger().warning("Need at least 2 common layout keys for calibration.")
            return

        key1 = common_keys[0]
        key2 = common_keys[1]
        lp1 = layout[key1] / 1000.0  # mm -> m
        lp2 = layout[key2] / 1000.0
        wp1 = measured_points[key1]
        wp2 = measured_points[key2]

        vec_layout = lp2 - lp1
        if np.linalg.norm(vec_layout) == 0:
            self.get_logger().error("Selected keys have identical layout coords.")
            return

        # Construct a local coordinate system for the keyboard plane
        horizontal_axis = (wp2 - wp1)
        horizontal_axis /= np.linalg.norm(horizontal_axis)

        # Assume an 'up' of [0,0,1], cross to get the vertical axis
        assumed_up = np.array([0, 0, 1])
        plane_normal = np.cross(horizontal_axis, assumed_up)
        if np.linalg.norm(plane_normal) == 0:
            plane_normal = np.array([0, 1, 0])
        else:
            plane_normal /= np.linalg.norm(plane_normal)
        vertical_axis = np.cross(plane_normal, horizontal_axis)
        vertical_axis /= np.linalg.norm(vertical_axis)

        R = np.column_stack((horizontal_axis, vertical_axis, plane_normal))
        t = wp1 - R.dot(np.array([lp1[0], lp1[1], 0.0]))

        key_positions = {}
        for lt, xy in layout.items():
            xy_m = np.array([xy[0] / 1000.0, xy[1] / 1000.0, 0.0])
            world_pt = R.dot(xy_m) + t
            key_positions[lt] = world_pt

        self.key_positions_global = key_positions

        # Publish the entire keyboard:
        msg = Float32MultiArray()
        data = []
        for letter, pos in key_positions.items():
            data.extend([float(ord(letter)), float(pos[0]), float(pos[1]), float(pos[2])])
        msg.data = data
        self.keyboard_pub.publish(msg)

        self.get_logger().info("Published 3D key positions:")
        for letter, pos in key_positions.items():
            self.get_logger().info(f"  {letter}: {pos}")

    def type_char(self, char_position):
        """
        Move end effector towards the desired char_position (in global coords).
        Return True if the key is "pressed", i.e. we are sufficiently close.
        """
        now_time = self.timestamp_to_float(self.get_clock().now().to_msg())
        if self.last_command_time is not None and (now_time - self.last_command_time < 0.25):
            return False
        self.last_command_time = now_time

        # If we have no current EE tip estimate, skip
        if self.end_effector_tip is None:
            return False

        # In the new coordinate system, going from tip -> char means delta = (target - tip)
        delta = char_position - self.end_effector_tip
        dist = np.linalg.norm(delta)

        self.get_logger().info(f"Moving EE to {char_position}, current={self.end_effector_tip}, dist={dist:.3f}")

        # If close enough to press:
        if dist < 0.03:
            # pretend we pressed the key
            return True

        # Otherwise, step at most 5 mm
        max_step = 0.005
        move = delta
        mag = np.linalg.norm(move)
        if mag > max_step:
            move = (move / mag) * max_step

        # We'll interpret move.x => linear.x, move.y => ???,
        # but we only have Twist: linear.x, linear.y, linear.z in standard geometry_msgs
        # Suppose we treat "move.x => linear.x" and "move.z => linear.z" if we want to lift up/down
        # We'll try ignoring Y for direct end-effector moves, or we do base rotation for Y
        # (depending on your robot's control approach).
        arm_move = Twist()
        arm_move.linear.x = move[0]
        # If you want to do left as +y in Twist, do:
        arm_move.linear.y = move[1]
        arm_move.linear.z = move[2]
        self.cmd_vel_pub.publish(arm_move)

        # Alternatively, if you want to rotate the base for Y:
        # if abs(delta[1]) > 0.0:
        #     # rotate base by some fraction of delta[1], etc.

        # For demonstration, we just do linear.x=move[0], y=move[1], z=move[2].
        # Update local estimate
        self.end_effector_tip += move
        return False

    def compute_roll_pitch_yaw(self, normal):
        """
        Compute approximate roll, pitch, yaw from a normal vector
        if we want that for debugging. We'll define +Z as up.
        """
        normal = normal / np.linalg.norm(normal)
        nx, ny, nz = normal

        # pitch about Y => arcsin(-nx)
        pitch = np.arcsin(-nx)
        # yaw about Z => atan2(ny, nz)
        yaw = np.arctan2(ny, nz)
        roll = 0.0
        return roll, pitch, yaw

    def timestamp_to_float(self, timestamp):
        """
        Convert a ROS2 timestamp to float seconds.
        """
        return timestamp.sec + timestamp.nanosec * 1e-9

    def rotate_base(self, angle):
        """
        Publish a small rotation increment around the base joint
        (if your robot has a base rotation in /arm/joint_increment).
        """
        msg = Float32MultiArray()
        data = [angle, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.data = data
        self.increment_pub.publish(msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeySelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
