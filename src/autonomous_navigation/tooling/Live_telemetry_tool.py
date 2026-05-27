#!/usr/bin/env python3

import csv
import math
import os
import threading
import time
import tkinter as tk
from tkinter import ttk
import cv2
import yaml
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped, Point, Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String, Bool

from cmr_msgs.msg import AutonomyDrive, Detection


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_WAYPOINT_FILE = os.path.join(PACKAGE_DIR, 'config', 'waypoints_engquad.yaml')

class RealtimeRobotPlotter(Node):
    def __init__(self):
        super().__init__('realtime_robot_plotter')

        self._closing = False
        self.lock = threading.Lock()

        # ---------------- Parameters ----------------
        self.declare_parameter('robot_pose_topic', '/autonomy/pose/robot/global')
        self.declare_parameter('target_topic', '/autonomy/target_object/position')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('ackermann_topic', '/autonomy/move/ackerman')
        self.declare_parameter('point_turn_topic', '/autonomy/move/point_turn')
        self.declare_parameter('state_topic', '/autonomy/state')
        self.declare_parameter('gps_topic', '/rtk/navsatfix_data')
        self.declare_parameter('move_type_topic', '/autonomy/move/move_type')
        self.declare_parameter('image_topic', '/zed/image_left')
        # URC 2026 detection overlay: single best box from the base-station
        # detector and the onboard arrived/detected state.
        self.declare_parameter('detection_topic', '/autonomy/detection/result')
        self.declare_parameter('arrived_topic', '/autonomy/detection/arrived')
        # How long a detection box / banner persists on screen without a fresh
        # message (the detector samples at ~2 Hz, so this bridges the gap).
        self.declare_parameter('detection_overlay_timeout', 2.0)
        self.declare_parameter('robot_pose_type', 'twist_stamped')  # pose_stamped / odometry / twist_stamped
        self.declare_parameter('target_type', 'twist')              # point / pose_stamped / twist
        self.declare_parameter('waypoints_file', DEFAULT_WAYPOINT_FILE)
        self.declare_parameter('debug_logging', True)

        self.declare_parameter('canvas_width', 950)
        self.declare_parameter('canvas_height', 820)
        self.declare_parameter('ui_update_hz', 20.0)
        self.declare_parameter('trail_length', 800)
        self.declare_parameter('zoom_factor', 1.0)
        # Emergency override: if the displayed heading rotates the wrong
        # direction vs. the physical rover (e.g. the IMU sign flip ever
        # regresses), set this True to negate yaw at display time only.
        # rtk_localization should publish CCW-positive yaw in the N/W frame
        # (yaw=0 -> facing N, +pi/2 -> facing W); when that is true, leave
        # this False.
        self.declare_parameter('invert_display_yaw', False)

        # Fallback canvas bounds in local meters (East/North). The first
        # GPS fix triggers auto-bounds-from-waypoints, so these only matter
        # for the brief startup window before the fix lands.
        self.declare_parameter('world_x_min', -25.0)
        self.declare_parameter('world_x_max', 25.0)
        self.declare_parameter('world_y_min', -25.0)
        self.declare_parameter('world_y_max', 25.0)

        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.ackermann_topic = self.get_parameter('ackermann_topic').value
        self.point_turn_topic = self.get_parameter('point_turn_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.move_type_topic = self.get_parameter('move_type_topic').value
        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.arrived_topic = self.get_parameter('arrived_topic').value
        self.detection_overlay_timeout = float(self.get_parameter('detection_overlay_timeout').value)
        self.robot_pose_type = self.get_parameter('robot_pose_type').value
        self.target_type = self.get_parameter('target_type').value
        self.waypoints_file = self.get_parameter('waypoints_file').value or DEFAULT_WAYPOINT_FILE
        self.debug_logging = bool(self.get_parameter('debug_logging').value)

        self.initial_canvas_width = int(self.get_parameter('canvas_width').value)
        self.initial_canvas_height = int(self.get_parameter('canvas_height').value)
        self.ui_update_hz = float(self.get_parameter('ui_update_hz').value)
        self.trail_length = int(self.get_parameter('trail_length').value)
        self.zoom_factor = max(0.05, float(self.get_parameter('zoom_factor').value))

        self.invert_display_yaw = bool(self.get_parameter('invert_display_yaw').value)

        self.world_x_min = float(self.get_parameter('world_x_min').value)
        self.world_x_max = float(self.get_parameter('world_x_max').value)
        self.world_y_min = float(self.get_parameter('world_y_min').value)
        self.world_y_max = float(self.get_parameter('world_y_max').value)
        # Canvas convention: world_x = East (m), world_y = North (m). This is a
        # standard "north-up, east-right" map. The autonomy stack publishes the
        # rover pose in (north, west, yaw_NWU_CCW); we convert to (east, north)
        # at the entry points so the canvas stays in the conventional frame.
        self.world_x_label_name = 'East (m)'
        self.world_y_label_name = 'North (m)'
        self.data_world_x_min = self.world_x_min
        self.data_world_x_max = self.world_x_max
        self.data_world_y_min = self.world_y_min
        self.data_world_y_max = self.world_y_max
        self.base_world_x_min = self.world_x_min
        self.base_world_x_max = self.world_x_max
        self.base_world_y_min = self.world_y_min
        self.base_world_y_max = self.world_y_max
        self.pan_center_x = None
        self.pan_center_y = None
        self.gps_reference_lat = None
        self.gps_reference_lon = None

        # ---------------- State ----------------
        # Canvas-frame (East, North) for plotting. Computed from the autonomy
        # pose msg (linear.x = north_m, linear.y = west_m) at the callback
        # boundary so the rest of the rendering can stay in standard map coords.
        self.robot_x = None
        self.robot_y = None
        # Native autonomy frame (North, West) kept around for the side panel
        # readout so the user can cross-reference what state_machine sees.
        self.robot_north = None
        self.robot_west = None
        self.robot_yaw = 0.0
        self.robot_pose_frame_id = ''

        # Canvas-frame (East, North) target. target_north/_west kept for panel.
        self.target_x = None
        self.target_y = None
        self.target_north = None
        self.target_west = None
        self.current_state = '--'
        self.current_move_type = '--'
        self.gps_lat = None
        self.gps_lon = None

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.speed = 0.0
        self.cmd_source = 'none'
        self.ackermann_speed = 0.0
        self.ackermann_fl = 0.0
        self.ackermann_fr = 0.0
        self.point_turn_wz = 0.0
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_stamp = None
        self.latest_image_size = None

        # === URC object-detection subsystem (overview: cmr_cams/DETECTION_TESTING.md) ===
        # Overlay state. Drawn by draw_detection_overlay(); fed by detection_cb()
        # and arrived_cb(). Last NON-empty detection box: (label, conf, x1, y1,
        # x2, y2) normalized
        # to [0, 1]; kept until it ages out so the box doesn't flicker between
        # the detector's samples.
        self.latest_box = None
        self.latest_box_stamp = None
        # Latest per-frame "detected" flag (drives the banner together with the
        # onboard arrived state).
        self.detection_detected = False
        self.detection_detected_stamp = None
        # Onboard arrived/detected state from the state machine (held through
        # the detection dwell). Authoritative source for the banner / future LED.
        self.arrived_state = False
        self.latest_image_tk = None
        self.image_topics = []
        self.last_image_topic_refresh = 0.0

        self.robot_path = []
        self.waypoints = []
        # If the YAML uses (latitude, longitude) format, we stash the raw
        # pairs here and defer conversion to local meters until the first
        # GPS fix lands -- that way the telemetry frame is anchored at the
        # rover's actual starting GPS, matching state_machine and
        # rtk_localization (instead of waypoints[0]).
        self.waypoint_gps_pairs = []
        self.msg_counts = {
            'robot_pose': 0,
            'target': 0,
            'cmd_vel': 0,
            'ackermann': 0,
            'point_turn': 0,
            'state': 0,
            'gps': 0,
            'move_type': 0,
            'image': 0,
        }
        self.last_seen = {key: None for key in self.msg_counts}
        self.topic_configs = {
            'robot_pose': self.robot_pose_topic,
            'target': self.target_topic,
            'cmd_vel': self.cmd_vel_topic,
            'ackermann': self.ackermann_topic,
            'point_turn': self.point_turn_topic,
            'state': self.state_topic,
            'gps': self.gps_topic,
            'move_type': self.move_type_topic,
            'image': self.image_topic,
        }
        self.last_debug_log = 0.0
        self.debug_period = 2.0

        # logging
        self.is_logging = False
        self.logged_rows = []
        self.last_log_time = 0.0
        self.log_period = 0.1  # 10 Hz logging

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.default_csv_name = 'live_telemetry_log.csv'

        self.get_logger().info(f'Waypoint file being used: {self.waypoints_file}')
        self.load_waypoints()

        # ---------------- Subscribers ----------------
        if self.robot_pose_type == 'pose_stamped':
            self.robot_sub = self.create_subscription(
                PoseStamped, self.robot_pose_topic, self.robot_pose_pose_stamped_cb, 10
            )
        elif self.robot_pose_type == 'twist_stamped':
            self.robot_sub = self.create_subscription(
                TwistStamped, self.robot_pose_topic, self.robot_pose_twist_stamped_cb, 10
            )
        elif self.robot_pose_type == 'odometry':
            self.robot_sub = self.create_subscription(
                Odometry, self.robot_pose_topic, self.robot_pose_odom_cb, 10
            )
        else:
            raise ValueError(f"Unsupported robot_pose_type: {self.robot_pose_type}")

        if self.target_type == 'point':
            self.target_sub = self.create_subscription(
                Point, self.target_topic, self.target_point_cb, 10
            )
        elif self.target_type == 'twist':
            self.target_sub = self.create_subscription(
                Twist, self.target_topic, self.target_twist_cb, 10
            )
        elif self.target_type == 'pose_stamped':
            self.target_sub = self.create_subscription(
                PoseStamped, self.target_topic, self.target_pose_stamped_cb, 10
            )
        else:
            raise ValueError(f"Unsupported target_type: {self.target_type}")

        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_cb, 10
        )
        self.ackermann_sub = self.create_subscription(
            AutonomyDrive, self.ackermann_topic, self.ackermann_cb, 10
        )
        self.point_turn_sub = self.create_subscription(
            Twist, self.point_turn_topic, self.point_turn_cb, 10
        )
        self.state_sub = self.create_subscription(
            String, self.state_topic, self.state_cb, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self.gps_cb, 10
        )
        self.move_type_sub = self.create_subscription(
            String, self.move_type_topic, self.move_type_cb, 10
        )
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_cb, 10
        )
        self.detection_sub = self.create_subscription(
            Detection, self.detection_topic, self.detection_cb, 10
        )
        self.arrived_sub = self.create_subscription(
            Bool, self.arrived_topic, self.arrived_cb, 10
        )

        self.get_logger().info('Realtime Robot Plotter started.')
        self.get_logger().info(f'Robot pose topic: {self.robot_pose_topic} ({self.robot_pose_type})')
        self.get_logger().info(f'Target topic: {self.target_topic} ({self.target_type})')
        self.get_logger().info(f'Cmd vel topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'Ackermann topic: {self.ackermann_topic}')
        self.get_logger().info(f'Point turn topic: {self.point_turn_topic}')
        self.get_logger().info(f'State topic: {self.state_topic}')
        self.get_logger().info(f'GPS topic: {self.gps_topic}')
        self.get_logger().info(f'Image topic: {self.image_topic}')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoint(s) from config.')

        # ---------------- Tkinter UI ----------------
        self.root = tk.Tk()
        self.root.title("ROS 2 Live Telemetry Tool")
        self.root.geometry(f"{self.initial_canvas_width + 430}x{self.initial_canvas_height + 360}")
        self.root.minsize(1150, 760)

        main_frame = ttk.Frame(self.root, padding=8)
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.main_pane = tk.PanedWindow(
            main_frame,
            orient=tk.HORIZONTAL,
            sashwidth=8,
            sashrelief=tk.RAISED,
            showhandle=True,
            bd=0
        )
        self.main_pane.pack(fill=tk.BOTH, expand=True)

        left_frame = ttk.Frame(self.main_pane)
        self.main_pane.add(left_frame, minsize=520, stretch='always')

        self.left_pane = tk.PanedWindow(
            left_frame,
            orient=tk.VERTICAL,
            sashwidth=8,
            sashrelief=tk.RAISED,
            showhandle=True,
            bd=0
        )
        self.left_pane.grid(row=0, column=0, sticky='nsew')

        map_box = ttk.LabelFrame(self.left_pane, text='Map', padding=4)

        map_controls = ttk.Frame(map_box)
        map_controls.pack(fill='x', pady=(0, 4))
        ttk.Button(map_controls, text='Zoom +', command=self.zoom_in).pack(side='left', padx=(0, 4))
        ttk.Button(map_controls, text='Zoom -', command=self.zoom_out).pack(side='left', padx=(0, 4))
        ttk.Button(map_controls, text='Reset View', command=self.reset_view).pack(side='left')
        self.zoom_label = ttk.Label(map_controls, text='Zoom: 1.00x')
        self.zoom_label.pack(side='right')

        self.canvas = tk.Canvas(
            map_box,
            width=self.initial_canvas_width,
            height=self.initial_canvas_height,
            bg='white'
        )
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<MouseWheel>', self.on_map_mousewheel)
        self.canvas.bind('<Button-4>', self.on_map_mousewheel)
        self.canvas.bind('<Button-5>', self.on_map_mousewheel)

        self.left_pane.add(map_box, minsize=220, stretch='always')

        camera_box = ttk.LabelFrame(self.left_pane, text='Camera Feed', padding=8)
        self.left_pane.add(camera_box, minsize=180, stretch='always')

        camera_controls = ttk.Frame(camera_box)
        camera_controls.pack(fill='x', pady=(0, 6))

        ttk.Label(camera_controls, text='Image topic:').grid(row=0, column=0, sticky='w')
        self.image_topic_var = tk.StringVar(value=self.image_topic)
        self.image_topic_combo = ttk.Combobox(
            camera_controls,
            textvariable=self.image_topic_var,
            state='readonly'
        )
        self.image_topic_combo.grid(row=0, column=1, sticky='ew', padx=6)
        self.image_topic_combo.bind('<<ComboboxSelected>>', self.on_image_topic_selected)

        self.refresh_topics_btn = ttk.Button(
            camera_controls,
            text='Refresh Topics',
            command=self.refresh_image_topics
        )
        self.refresh_topics_btn.grid(row=0, column=2, sticky='e')
        camera_controls.columnconfigure(1, weight=1)

        # URC 2026 detection overlay toggle. On = draw the single detected box
        # + the "OBJECT DETECTED" banner over the ZED feed.
        self.overlay_enabled_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            camera_controls,
            text='Show detection overlay',
            variable=self.overlay_enabled_var,
        ).grid(row=1, column=0, columnspan=3, sticky='w', pady=(4, 0))

        self.image_canvas = tk.Canvas(camera_box, bg='black', highlightthickness=0)
        self.image_canvas.pack(fill='both', expand=True)
        self.image_status_label = ttk.Label(camera_box, text='Topic: -- | Image: waiting for frames', wraplength=700)
        self.image_status_label.pack(anchor='w')

        self.side_container = ttk.Frame(self.main_pane)
        self.main_pane.add(self.side_container, minsize=260, stretch='never')

        self.side_canvas = tk.Canvas(self.side_container, highlightthickness=0, width=340)
        self.side_scrollbar = ttk.Scrollbar(
            self.side_container,
            orient=tk.VERTICAL,
            command=self.side_canvas.yview
        )
        self.side_canvas.configure(yscrollcommand=self.side_scrollbar.set)
        self.side_canvas.grid(row=0, column=0, sticky='nsew')
        self.side_scrollbar.grid(row=0, column=1, sticky='ns')
        self.side_container.columnconfigure(0, weight=1)
        self.side_container.rowconfigure(0, weight=1)

        self.side_frame = ttk.Frame(self.side_canvas)
        self.side_canvas_window = self.side_canvas.create_window(
            (0, 0),
            window=self.side_frame,
            anchor='nw'
        )
        self.side_frame.bind('<Configure>', self.on_side_frame_configure)
        self.side_canvas.bind('<Configure>', self.on_side_canvas_configure)
        self.side_container.bind('<Enter>', self.bind_side_mousewheel)
        self.side_container.bind('<Leave>', self.unbind_side_mousewheel)
        self.side_canvas.bind('<Enter>', self.bind_side_mousewheel)
        self.side_canvas.bind('<Leave>', self.unbind_side_mousewheel)
        self.side_frame.bind('<Enter>', self.bind_side_mousewheel)
        self.side_frame.bind('<Leave>', self.unbind_side_mousewheel)

        # ---- Heading box (absolute heading from rtk_localization)
        heading_box = ttk.LabelFrame(self.side_frame, text='Heading (absolute)', padding=8)
        heading_box.pack(fill='x', pady=4)
        self.heading_canvas = tk.Canvas(heading_box, width=120, height=120, bg='#1e1e1e',
                                        highlightthickness=1, highlightbackground='#555555')
        self.heading_canvas.pack(anchor='center', pady=(2, 4))
        self.heading_label = ttk.Label(heading_box, text='Yaw: -- (--)',
                                       font=('TkFixedFont', 11, 'bold'),
                                       anchor='center')
        self.heading_label.pack(fill='x')
        self.heading_hint_label = ttk.Label(
            heading_box,
            text='N/W-frame, CCW-positive: 0=N, +90=W, -90=E',
            wraplength=280, foreground='gray40'
        )
        self.heading_hint_label.pack(anchor='w')

        # ---- Drive command direction (rover-relative)
        dir_box = ttk.LabelFrame(self.side_frame, text='Drive Command Direction', padding=8)
        dir_box.pack(fill='x', pady=4)
        self.direction_canvas = tk.Canvas(dir_box, width=120, height=120, bg='#1e1e1e',
                                          highlightthickness=1, highlightbackground='#555555')
        self.direction_canvas.pack(anchor='center', pady=(2, 4))
        self.direction_label = ttk.Label(dir_box, text='Stopped', font=('TkFixedFont', 11, 'bold'),
                                         anchor='center')
        self.direction_label.pack(fill='x')

        # ---- Robot box
        robot_box = ttk.LabelFrame(self.side_frame, text='Robot State', padding=8)
        robot_box.pack(fill='x', pady=4)
        # Autonomy-stack (north, west) frame -- matches rtk_localization,
        # state_machine, controller logs.
        self.robot_north_label = ttk.Label(robot_box, text='North: -- m')
        self.robot_north_label.pack(anchor='w')
        self.robot_west_label = ttk.Label(robot_box, text='West:  -- m')
        self.robot_west_label.pack(anchor='w')
        # Same position re-expressed in canvas (east, north) for cross-ref.
        self.robot_canvas_label = ttk.Label(robot_box, text='Canvas (E, N): --',
                                            foreground='gray40')
        self.robot_canvas_label.pack(anchor='w')
        self.speed_label = ttk.Label(robot_box, text='Speed: -- m/s')
        self.speed_label.pack(anchor='w')
        self.gps_lat_label = ttk.Label(robot_box, text='GPS Lat: --')
        self.gps_lat_label.pack(anchor='w')
        self.gps_lon_label = ttk.Label(robot_box, text='GPS Lon: --')
        self.gps_lon_label.pack(anchor='w')

        # ---- GPS origin box
        origin_box = ttk.LabelFrame(self.side_frame, text='GPS Origin', padding=8)
        origin_box.pack(fill='x', pady=4)
        self.origin_status_label = ttk.Label(origin_box, text='Origin: waiting for first fix...',
                                             font=('TkDefaultFont', 9, 'bold'),
                                             foreground='#c0392b')
        self.origin_status_label.pack(anchor='w')
        self.origin_lat_label = ttk.Label(origin_box, text='Lat: --')
        self.origin_lat_label.pack(anchor='w')
        self.origin_lon_label = ttk.Label(origin_box, text='Lon: --')
        self.origin_lon_label.pack(anchor='w')

        # ---- Target box
        target_box = ttk.LabelFrame(self.side_frame, text='Target State', padding=8)
        target_box.pack(fill='x', pady=4)
        self.target_north_label = ttk.Label(target_box, text='Target North: --')
        self.target_north_label.pack(anchor='w')
        self.target_west_label = ttk.Label(target_box, text='Target West:  --')
        self.target_west_label.pack(anchor='w')
        self.target_canvas_label = ttk.Label(target_box, text='Canvas (E, N): --',
                                             foreground='gray40')
        self.target_canvas_label.pack(anchor='w')
        self.dist_label = ttk.Label(target_box, text='Distance: --')
        self.dist_label.pack(anchor='w')
        self.bearing_label = ttk.Label(target_box, text='Bearing: --')
        self.bearing_label.pack(anchor='w')

        # ---- Command box
        cmd_box = ttk.LabelFrame(self.side_frame, text='cmd_vel Telemetry', padding=8)
        cmd_box.pack(fill='x', pady=4)
        self.cmd_vx_label = ttk.Label(cmd_box, text='vx: --')
        self.cmd_vx_label.pack(anchor='w')
        self.cmd_vy_label = ttk.Label(cmd_box, text='vy: --')
        self.cmd_vy_label.pack(anchor='w')
        self.cmd_wz_label = ttk.Label(cmd_box, text='wz: --')
        self.cmd_wz_label.pack(anchor='w')
        self.cmd_source_label = ttk.Label(cmd_box, text='Source: --')
        self.cmd_source_label.pack(anchor='w')
        self.ack_label = ttk.Label(cmd_box, text='Ackermann: vel=-- fl=-- fr=--')
        self.ack_label.pack(anchor='w')
        self.point_turn_label = ttk.Label(cmd_box, text='Point turn wz: --')
        self.point_turn_label.pack(anchor='w')

        autonomy_box = ttk.LabelFrame(self.side_frame, text='Autonomy Status', padding=8)
        autonomy_box.pack(fill='x', pady=4)
        self.state_value_label = ttk.Label(autonomy_box, text='Current state: --')
        self.state_value_label.pack(anchor='w')
        self.move_type_label = ttk.Label(autonomy_box, text='Move type: --')
        self.move_type_label.pack(anchor='w')
        self.topic_health_label = ttk.Label(autonomy_box, text='Topic health: --', wraplength=280)
        self.topic_health_label.pack(anchor='w')
        self.publisher_health_label = ttk.Label(autonomy_box, text='Publishers: --', wraplength=280)
        self.publisher_health_label.pack(anchor='w')

        # ---- Map box
        map_box = ttk.LabelFrame(self.side_frame, text='Map / Plot Info', padding=8)
        map_box.pack(fill='x', pady=4)
        self.wp_label = ttk.Label(map_box, text=f'Waypoints: {len(self.waypoints)}')
        self.wp_label.pack(anchor='w')
        self.bounds_label = ttk.Label(map_box, text='')
        self.bounds_label.pack(anchor='w')
        self.grid_label = ttk.Label(map_box, text='')
        self.grid_label.pack(anchor='w')
        self.robot_size_label = ttk.Label(map_box, text='')
        self.robot_size_label.pack(anchor='w')
        self.canvas_size_label = ttk.Label(map_box, text='')
        self.canvas_size_label.pack(anchor='w')

        # ---- Logging box
        log_box = ttk.LabelFrame(self.side_frame, text='Logging', padding=8)
        log_box.pack(fill='x', pady=4)

        self.logging_status_label = ttk.Label(log_box, text='Logging: OFF')
        self.logging_status_label.pack(anchor='w', pady=(0, 6))

        btn_frame = ttk.Frame(log_box)
        btn_frame.pack(fill='x')

        self.start_btn = ttk.Button(btn_frame, text='Start Logging', command=self.start_logging)
        self.start_btn.grid(row=0, column=0, padx=2, pady=2, sticky='ew')

        self.stop_btn = ttk.Button(btn_frame, text='Stop Logging', command=self.stop_logging)
        self.stop_btn.grid(row=0, column=1, padx=2, pady=2, sticky='ew')

        self.save_btn = ttk.Button(btn_frame, text='Save CSV', command=self.save_csv)
        self.save_btn.grid(row=1, column=0, columnspan=2, padx=2, pady=2, sticky='ew')

        btn_frame.columnconfigure(0, weight=1)
        btn_frame.columnconfigure(1, weight=1)

        self.log_count_label = ttk.Label(log_box, text='Rows logged: 0')
        self.log_count_label.pack(anchor='w', pady=(6, 0))

        self.log_file_label = ttk.Label(
            log_box,
            text=f'Save path: {os.path.join(self.script_dir, self.default_csv_name)}',
            wraplength=280
        )
        self.log_file_label.pack(anchor='w', pady=(4, 0))

        # ---- Status box
        status_box = ttk.LabelFrame(self.side_frame, text='Status', padding=8)
        status_box.pack(fill='x', pady=4)
        self.status_label = ttk.Label(status_box, text='Running')
        self.status_label.pack(anchor='w')

        left_frame.columnconfigure(0, weight=1)
        left_frame.rowconfigure(0, weight=1)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.ui_period_ms = max(1, int(1000.0 / max(self.ui_update_hz, 1.0)))
        self.refresh_image_topics()
        self.root.after(100, self.set_initial_pane_sizes)
        self.root.after(self.ui_period_ms, self.update_ui)

    def set_initial_pane_sizes(self):
        try:
            total_width = self.main_pane.winfo_width()
            total_height = self.left_pane.winfo_height()
            if total_width > 0:
                # Reserve 360 px on the right for the telemetry side panel so
                # the heading/state/origin readouts have room without clipping.
                self.main_pane.sash_place(0, max(520, total_width - 360), 0)
            if total_height > 0:
                self.left_pane.sash_place(0, 0, int(total_height * 0.72))
        except Exception:
            pass

    def on_side_frame_configure(self, _event=None):
        self.side_canvas.configure(scrollregion=self.side_canvas.bbox('all'))

    def on_side_canvas_configure(self, event):
        self.side_canvas.itemconfigure(self.side_canvas_window, width=event.width)

    def bind_side_mousewheel(self, _event=None):
        self.side_canvas.bind_all('<MouseWheel>', self.on_side_mousewheel)
        self.side_canvas.bind_all('<Button-4>', self.on_side_mousewheel)
        self.side_canvas.bind_all('<Button-5>', self.on_side_mousewheel)

    def unbind_side_mousewheel(self, _event=None):
        self.side_canvas.unbind_all('<MouseWheel>')
        self.side_canvas.unbind_all('<Button-4>')
        self.side_canvas.unbind_all('<Button-5>')

    def on_side_mousewheel(self, event):
        if getattr(event, 'num', None) == 4 or getattr(event, 'delta', 0) > 0:
            self.side_canvas.yview_scroll(-3, 'units')
        else:
            self.side_canvas.yview_scroll(3, 'units')

    # ---------------- Canvas helpers ----------------
    def get_canvas_size(self):
        width = max(100, self.canvas.winfo_width())
        height = max(100, self.canvas.winfo_height())
        return width, height

    # ---------------- Waypoint Loader ----------------
    def load_waypoints(self):
        if not self.waypoints_file:
            self.get_logger().warn('No waypoints file provided.')
            return

        try:
            with open(self.waypoints_file, 'r') as f:
                data = yaml.safe_load(f)

            loaded = data.get('waypoints', [])
            self.waypoints = []

            if not loaded:
                self.get_logger().warn(f'Waypoint file loaded but no waypoints found: {self.waypoints_file}')
                return

            if all(('x' in wp and 'y' in wp) for wp in loaded):
                self.world_x_label_name = 'X'
                self.world_y_label_name = 'Y'
                for wp in loaded:
                    self.waypoints.append((float(wp['x']), float(wp['y'])))
                self.get_logger().info(f'Waypoints parsed successfully: {len(self.waypoints)}')
                self.auto_set_bounds_from_waypoints()
                return

            if all(('latitude' in wp and 'longitude' in wp) for wp in loaded):
                # Canvas axes are East (right) and North (up); see the
                # frame-convention comment in __init__.
                self.world_x_label_name = 'East (m)'
                self.world_y_label_name = 'North (m)'
                # Stash raw (lat, lon) pairs and defer the conversion to
                # local meters until gps_cb captures the rover's first GPS
                # fix as the reference origin. self.waypoints stays empty
                # until then, so the map renders without waypoints for the
                # first second or two of operation.
                self.waypoint_gps_pairs = [
                    (float(wp['latitude']), float(wp['longitude']))
                    for wp in loaded
                ]
                self.get_logger().info(
                    f'Stashed {len(self.waypoint_gps_pairs)} GPS waypoint(s); '
                    f'will convert to local meters once the first fix on '
                    f'{self.gps_topic} sets the reference origin.'
                )
                return

            self.get_logger().warn(
                f'Unsupported waypoint format in {self.waypoints_file}. '
                f'Expected x/y or latitude/longitude.'
            )

        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints file {self.waypoints_file}: {e}')
            self.waypoints = []

    def auto_set_bounds_from_waypoints(self):
        if not self.waypoints:
            return

        xs = [p[0] for p in self.waypoints]
        ys = [p[1] for p in self.waypoints]

        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)

        self.set_data_bounds(min_x, max_x, min_y, max_y)

        self.get_logger().info(
            f'Auto plot bounds from waypoints -> '
            f'{self.world_x_label_name}:[{self.format_axis_value(self.base_world_x_min, self.world_x_label_name)}, '
            f'{self.format_axis_value(self.base_world_x_max, self.world_x_label_name)}] '
            f'{self.world_y_label_name}:[{self.format_axis_value(self.base_world_y_min, self.world_y_label_name)}, '
            f'{self.format_axis_value(self.base_world_y_max, self.world_y_label_name)}]'
        )

    def set_data_bounds(self, x_min, x_max, y_min, y_max):
        self.data_world_x_min = x_min
        self.data_world_x_max = x_max
        self.data_world_y_min = y_min
        self.data_world_y_max = y_max
        self.recompute_base_bounds_from_data()
        self.pan_center_x = (self.base_world_x_min + self.base_world_x_max) / 2.0
        self.pan_center_y = (self.base_world_y_min + self.base_world_y_max) / 2.0
        self.apply_zoom_to_bounds()

    def recompute_base_bounds_from_data(self):
        if self.is_geospatial_plot():
            x_span = max(1e-6, self.data_world_x_max - self.data_world_x_min)
            y_span = max(1e-6, self.data_world_y_max - self.data_world_y_min)
            margin_x = max(0.00005, 0.15 * x_span)
            margin_y = max(0.00005, 0.15 * y_span)
        else:
            x_span = max(1.0, self.data_world_x_max - self.data_world_x_min)
            y_span = max(1.0, self.data_world_y_max - self.data_world_y_min)
            margin_x = max(5.0, 0.15 * x_span)
            margin_y = max(5.0, 0.15 * y_span)

        self.base_world_x_min = self.data_world_x_min - margin_x
        self.base_world_x_max = self.data_world_x_max + margin_x
        self.base_world_y_min = self.data_world_y_min - margin_y
        self.base_world_y_max = self.data_world_y_max + margin_y

    def apply_zoom_to_bounds(self):
        base_x_span = max(1e-6, self.base_world_x_max - self.base_world_x_min)
        base_y_span = max(1e-6, self.base_world_y_max - self.base_world_y_min)
        center_x = self.pan_center_x
        center_y = self.pan_center_y
        if center_x is None or center_y is None:
            center_x = (self.base_world_x_min + self.base_world_x_max) / 2.0
            center_y = (self.base_world_y_min + self.base_world_y_max) / 2.0

        x_span = base_x_span / max(self.zoom_factor, 0.05)
        y_span = base_y_span / max(self.zoom_factor, 0.05)
        self.world_x_min = center_x - x_span / 2.0
        self.world_x_max = center_x + x_span / 2.0
        self.world_y_min = center_y - y_span / 2.0
        self.world_y_max = center_y + y_span / 2.0

    def zoom_in(self):
        self.set_zoom(self.zoom_factor * 1.25)

    def zoom_out(self):
        self.set_zoom(self.zoom_factor / 1.25)

    def reset_view(self):
        self.zoom_factor = 1.0
        self.pan_center_x = (self.base_world_x_min + self.base_world_x_max) / 2.0
        self.pan_center_y = (self.base_world_y_min + self.base_world_y_max) / 2.0
        self.apply_zoom_to_bounds()

    def set_zoom(self, zoom_factor):
        self.zoom_factor = min(80.0, max(0.05, zoom_factor))
        self.apply_zoom_to_bounds()

    def on_map_mousewheel(self, event):
        if getattr(event, 'num', None) == 5 or getattr(event, 'delta', 0) < 0:
            self.zoom_out()
        else:
            self.zoom_in()

    def gps_to_north_west(self, lat, lon):
        """Match rtk_localization/state_machine: returns (north_m, west_m)
        relative to the captured GPS reference. Used internally and for the
        side-panel readouts that mirror the autonomy stack frame."""
        if self.gps_reference_lat is None or self.gps_reference_lon is None:
            return None, None

        radius = 6378137.0
        lat_rad = math.radians(lat)
        ref_lat_rad = math.radians(self.gps_reference_lat)
        lon_rad = math.radians(lon)
        ref_lon_rad = math.radians(self.gps_reference_lon)
        delta_lat = ref_lat_rad - lat_rad
        delta_lon = ref_lon_rad - lon_rad
        mean_lat = (lat_rad + ref_lat_rad) / 2.0

        north = -1.0 * delta_lat * radius
        west = delta_lon * radius * math.cos(mean_lat)
        return north, west

    def gps_to_canvas_xy(self, lat, lon):
        """Return (east_m, north_m) for canvas plotting. east = -west so the
        canvas's +x axis points east."""
        north, west = self.gps_to_north_west(lat, lon)
        if north is None:
            # No reference yet -> fall back to raw (lon, lat) which is
            # nonsensical for the canvas but matches the previous behavior.
            return lon, lat
        return -west, north

    @staticmethod
    def north_west_to_canvas_xy(north, west):
        """Same flip used everywhere a (north, west) pair needs to be
        rendered on the East/North canvas."""
        if north is None or west is None:
            return None, None
        return -west, north

    def looks_like_lon_lat(self, x, y, frame_id=''):
        frame_id = (frame_id or '').lower()
        near_reference = False
        if self.gps_reference_lat is not None and self.gps_reference_lon is not None:
            near_reference = (
                abs(x - self.gps_reference_lon) < 1.0
                and abs(y - self.gps_reference_lat) < 1.0
            )
        return (
            'gps' in frame_id
            or 'navsat' in frame_id
            or near_reference
        )

    def normalize_position_for_plot(self, x, y, frame_id=''):
        if x is None or y is None:
            return None, None

        if self.gps_reference_lat is not None and self.looks_like_lon_lat(x, y, frame_id):
            # x = longitude, y = latitude here (we pass them in that order).
            return self.gps_to_canvas_xy(y, x)

        return x, y

    def expand_base_bounds_to_include(self, points):
        valid_points = [(x, y) for x, y in points if x is not None and y is not None]
        if not valid_points:
            return

        min_x = min([self.data_world_x_min] + [p[0] for p in valid_points])
        max_x = max([self.data_world_x_max] + [p[0] for p in valid_points])
        min_y = min([self.data_world_y_min] + [p[1] for p in valid_points])
        max_y = max([self.data_world_y_max] + [p[1] for p in valid_points])

        changed = (
            min_x < self.data_world_x_min
            or max_x > self.data_world_x_max
            or min_y < self.data_world_y_min
            or max_y > self.data_world_y_max
        )
        if not changed:
            return

        self.data_world_x_min = min_x
        self.data_world_x_max = max_x
        self.data_world_y_min = min_y
        self.data_world_y_max = max_y
        self.recompute_base_bounds_from_data()
        if self.zoom_factor <= 1.01:
            self.pan_center_x = (self.base_world_x_min + self.base_world_x_max) / 2.0
            self.pan_center_y = (self.base_world_y_min + self.base_world_y_max) / 2.0
        self.apply_zoom_to_bounds()

    # ---------------- ROS Callbacks ----------------
    def mark_seen(self, key):
        self.msg_counts[key] += 1
        self.last_seen[key] = time.time()

    def _store_robot_pose_north_west(self, north, west, yaw, frame_id):
        """Single sink for incoming pose msgs. The autonomy stack speaks
        (north, west, yaw_NWU_CCW); the canvas speaks (east, north). Convert
        once here so the rest of the rendering is unambiguous."""
        canvas_east, canvas_north = self.north_west_to_canvas_xy(north, west)
        with self.lock:
            self.robot_north = north
            self.robot_west = west
            self.robot_x = canvas_east
            self.robot_y = canvas_north
            self.robot_pose_frame_id = frame_id
            self.robot_yaw = yaw
        self.mark_seen('robot_pose')

    def robot_pose_pose_stamped_cb(self, msg: PoseStamped):
        # PoseStamped path is rarely used in the autonomy stack; treat
        # position.x/y as (north, west) for consistency with the rest of
        # the system (rtk_localization, state_machine, controller).
        yaw = self.quaternion_to_yaw(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        self._store_robot_pose_north_west(
            msg.pose.position.x,
            msg.pose.position.y,
            yaw,
            msg.header.frame_id,
        )

    def robot_pose_twist_stamped_cb(self, msg: TwistStamped):
        # rtk_localization packs (north, west, yaw_NWU_CCW) into
        # (linear.x, linear.y, angular.z) of a TwistStamped on
        # /autonomy/pose/robot/global.
        self._store_robot_pose_north_west(
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z,
            msg.header.frame_id,
        )

    def robot_pose_odom_cb(self, msg: Odometry):
        yaw = self.quaternion_to_yaw(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self._store_robot_pose_north_west(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            msg.header.frame_id,
        )

    def _store_target_north_west(self, north, west):
        """Mirror image of the rover-pose sink for incoming target msgs."""
        canvas_east, canvas_north = self.north_west_to_canvas_xy(north, west)
        with self.lock:
            self.target_north = north
            self.target_west = west
            self.target_x = canvas_east
            self.target_y = canvas_north
        self.mark_seen('target')

    def target_point_cb(self, msg: Point):
        self._store_target_north_west(msg.x, msg.y)

    def target_twist_cb(self, msg: Twist):
        self._store_target_north_west(msg.linear.x, msg.linear.y)

    def target_pose_stamped_cb(self, msg: PoseStamped):
        self._store_target_north_west(msg.pose.position.x, msg.pose.position.y)

    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.cmd_vx = msg.linear.x
            self.cmd_vy = msg.linear.y
            self.cmd_wz = msg.angular.z
            self.speed = math.sqrt(self.cmd_vx ** 2 + self.cmd_vy ** 2)
            self.cmd_source = 'cmd_vel'
        self.mark_seen('cmd_vel')

    def ackermann_cb(self, msg: AutonomyDrive):
        with self.lock:
            self.ackermann_speed = msg.vel
            self.ackermann_fl = msg.fl_angle
            self.ackermann_fr = msg.fr_angle
            self.cmd_vx = msg.vel
            self.cmd_vy = 0.0
            self.cmd_wz = 0.0
            self.speed = abs(msg.vel)
            self.cmd_source = 'ackermann'
        self.mark_seen('ackermann')

    def point_turn_cb(self, msg: Twist):
        with self.lock:
            self.point_turn_wz = msg.angular.z
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_wz = msg.angular.z
            self.speed = 0.0
            self.cmd_source = 'point_turn'
        self.mark_seen('point_turn')

    def state_cb(self, msg: String):
        with self.lock:
            self.current_state = msg.data
        self.mark_seen('state')

    def move_type_cb(self, msg: String):
        with self.lock:
            self.current_move_type = msg.data
        self.mark_seen('move_type')

    def image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Failed to convert image from {self.image_topic}: {e}')
            return

        with self.lock:
            self.latest_image = cv_image
            self.latest_image_stamp = time.time()
            self.latest_image_size = (msg.width, msg.height)
        self.mark_seen('image')

    def detection_cb(self, msg: Detection):
        """Latest single detection from the base-station detector."""
        now = time.time()
        with self.lock:
            if msg.label:
                self.latest_box = (
                    msg.label, float(msg.confidence),
                    float(msg.x1), float(msg.y1), float(msg.x2), float(msg.y2),
                )
                self.latest_box_stamp = now
            self.detection_detected = bool(msg.detected)
            self.detection_detected_stamp = now

    def arrived_cb(self, msg: Bool):
        """Onboard arrived/detected state (held through the detection dwell)."""
        with self.lock:
            self.arrived_state = bool(msg.data)

    def gps_cb(self, msg: NavSatFix):
        with self.lock:
            self.gps_lat = msg.latitude
            self.gps_lon = msg.longitude
        self.mark_seen('gps')
        self._maybe_capture_gps_reference(msg.latitude, msg.longitude)

    def _maybe_capture_gps_reference(self, lat, lon):
        """
        On the first GPS fix, latch (lat, lon) as the local-frame origin.
        This matches the rover's actual starting position used by
        rtk_localization and state_machine, so plotted waypoints share the
        same frame as the rover pose published on
        /autonomy/pose/robot/global. If the YAML was in (lat, lon) form,
        convert the stashed waypoints to local meters now.
        """
        if self.gps_reference_lat is not None:
            return

        self.gps_reference_lat = lat
        self.gps_reference_lon = lon
        self.get_logger().info(
            f'Captured GPS reference origin from first fix on '
            f'{self.gps_topic}: lat={lat:.7f} lon={lon:.7f}'
        )

        if self.waypoint_gps_pairs and not self.waypoints:
            self._convert_waypoint_gps_to_local()
            self.auto_set_bounds_from_waypoints()

    def _convert_waypoint_gps_to_local(self):
        """Convert the stashed (lat, lon) waypoint pairs to canvas-frame
        (east_m, north_m) tuples using gps_reference_lat/lon as the origin.
        The canvas uses East/North so this matches the rover-pose plot."""
        self.waypoints = [
            self.gps_to_canvas_xy(lat, lon)
            for lat, lon in self.waypoint_gps_pairs
        ]
        self.get_logger().info(
            f'Converted {len(self.waypoints)} GPS waypoint(s) to local meters '
            f'(east, north) against rover-start origin.'
        )

    # ---------------- Helpers ----------------
    def append_robot_path(self, x, y):
        if self.robot_path and self.robot_path[-1] == (x, y):
            return
        self.robot_path.append((x, y))
        if len(self.robot_path) > self.trail_length:
            self.robot_path.pop(0)

    def refresh_image_topics(self):
        image_topics = []
        for topic_name, topic_types in self.get_topic_names_and_types():
            if 'sensor_msgs/msg/Image' in topic_types:
                image_topics.append(topic_name)

        image_topics = sorted(set(image_topics))
        if self.image_topic and self.image_topic not in image_topics:
            image_topics.insert(0, self.image_topic)

        self.image_topics = image_topics
        if hasattr(self, 'image_topic_combo'):
            self.image_topic_combo['values'] = self.image_topics
            if self.image_topic:
                self.image_topic_var.set(self.image_topic)
        self.last_image_topic_refresh = time.time()

    def on_image_topic_selected(self, _event=None):
        selected_topic = self.image_topic_var.get().strip()
        if selected_topic:
            self.set_image_topic(selected_topic)

    def set_image_topic(self, topic_name):
        if topic_name == self.image_topic:
            return

        try:
            self.destroy_subscription(self.image_sub)
        except Exception:
            pass

        self.image_topic = topic_name
        self.topic_configs['image'] = topic_name
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_cb, 10
        )

        with self.lock:
            self.latest_image = None
            self.latest_image_stamp = None
            self.latest_image_size = None

        if hasattr(self, 'image_topic_var'):
            self.image_topic_var.set(topic_name)

        self.get_logger().info(f'Switched image topic to {topic_name}')

    def render_camera_feed(self):
        canvas_width = max(160, self.image_canvas.winfo_width())
        canvas_height = max(120, self.image_canvas.winfo_height())
        self.image_canvas.delete('all')

        with self.lock:
            frame = None if self.latest_image is None else self.latest_image.copy()
            image_stamp = self.latest_image_stamp
            image_size = self.latest_image_size

        if frame is None:
            self.image_status_label.config(text=f'Topic: {self.image_topic} | Image: waiting for frames')
            self.image_canvas.create_text(
                canvas_width / 2,
                canvas_height / 2,
                text='Waiting for image frames',
                fill='white',
                font=('TkDefaultFont', 12)
            )
            return

        frame_height, frame_width = frame.shape[:2]
        if frame_width <= 0 or frame_height <= 0:
            return

        scale = min(canvas_width / frame_width, canvas_height / frame_height)
        draw_width = max(1, int(frame_width * scale))
        draw_height = max(1, int(frame_height * scale))

        resized = cv2.resize(frame, (draw_width, draw_height), interpolation=cv2.INTER_AREA)
        rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        ppm_header = f'P6\n{draw_width} {draw_height}\n255\n'.encode('ascii')
        ppm_data = ppm_header + rgb_image.tobytes()
        self.latest_image_tk = tk.PhotoImage(data=ppm_data, format='PPM')

        offset_x = (canvas_width - draw_width) / 2
        offset_y = (canvas_height - draw_height) / 2
        self.image_canvas.create_image(offset_x, offset_y, anchor='nw', image=self.latest_image_tk)

        self.draw_detection_overlay(offset_x, offset_y, draw_width, draw_height, canvas_width)

        age_text = '--'
        if image_stamp is not None:
            age_text = f'{time.time() - image_stamp:.1f}s ago'

        size_text = '--'
        if image_size is not None:
            size_text = f'{image_size[0]}x{image_size[1]}'

        self.image_status_label.config(
            text=f'Topic: {self.image_topic} | Image: {size_text} | Last frame: {age_text}'
        )

    def draw_detection_overlay(self, offset_x, offset_y, draw_width, draw_height, canvas_width):
        """Draw EXACTLY ONE detection box (the base detector already reports
        only the single highest-confidence object) plus a large, obvious
        "OBJECT DETECTED" banner when the rover has arrived/stopped. This is the
        on-screen proof for the C2 telemetry panel. Toggled by the checkbox."""
        if not self.overlay_enabled_var.get():
            return

        now = time.time()
        with self.lock:
            box = self.latest_box
            box_stamp = self.latest_box_stamp
            detected = self.detection_detected
            detected_stamp = self.detection_detected_stamp
            arrived = self.arrived_state
        timeout = self.detection_overlay_timeout

        # The single bounding box, scaled from normalized coords onto the canvas.
        box_fresh = box is not None and box_stamp is not None and (now - box_stamp) < timeout
        box_label = ''
        if box_fresh:
            label, conf, nx1, ny1, nx2, ny2 = box
            box_label = label
            px1 = offset_x + min(nx1, nx2) * draw_width
            py1 = offset_y + min(ny1, ny2) * draw_height
            px2 = offset_x + max(nx1, nx2) * draw_width
            py2 = offset_y + max(ny1, ny2) * draw_height
            self.image_canvas.create_rectangle(px1, py1, px2, py2, outline='#00ff00', width=3)
            caption = f'{label} {conf * 100:.0f}%'
            tag_w = 9 * len(caption) + 10
            tag_top = max(offset_y, py1 - 20)
            self.image_canvas.create_rectangle(
                px1, tag_top, px1 + tag_w, tag_top + 20, fill='#00ff00', outline='')
            self.image_canvas.create_text(
                px1 + 5, tag_top + 10, anchor='w', text=caption,
                fill='black', font=('TkDefaultFont', 11, 'bold'))

        # Banner: driven by the onboard arrived state (held through the dwell),
        # or a recent detected flag if running detection without the nav stack.
        banner = arrived or (
            detected and detected_stamp is not None and (now - detected_stamp) < timeout)
        if banner:
            text = 'OBJECT DETECTED'
            if box_label:
                text += f': {box_label.upper()}'
            text += '  —  ROVER STOPPED'
            self.image_canvas.create_rectangle(
                offset_x, offset_y, offset_x + draw_width, offset_y + 46,
                fill='#cc0000', outline='')
            self.image_canvas.create_text(
                offset_x + draw_width / 2, offset_y + 23, text=text,
                fill='white', font=('TkDefaultFont', 18, 'bold'))

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_cardinal(yaw_radians):
        """Map an N/W-frame yaw (CCW-positive, 0=N, +pi/2=W) to one of 8
        cardinal labels. Sectors are 45 degrees wide and centered on each
        cardinal."""
        deg = math.degrees(yaw_radians) % 360.0
        labels = ['N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE']
        idx = int((deg + 22.5) // 45.0) % 8
        return labels[idx]

    @staticmethod
    def normalize_yaw_degrees(yaw_radians):
        """N/W-frame yaw in degrees, wrapped into (-180, 180]."""
        deg = math.degrees(yaw_radians)
        while deg > 180.0:
            deg -= 360.0
        while deg <= -180.0:
            deg += 360.0
        return deg

    def draw_heading_compass(self, yaw_radians):
        """Tiny absolute-heading compass: N is fixed up, W left, E right,
        S down. Arrow rotates with the rover's heading."""
        c = self.heading_canvas
        c.delete('all')
        cx, cy, r = 60, 60, 48
        c.create_oval(cx - r, cy - r, cx + r, cy + r,
                      outline='#666666', width=1, fill='#1e1e1e')

        # Cardinal labels at fixed positions.
        cardinals = [
            ('N', 0,    '#e74c3c'),
            ('E', 90,   '#aaaaaa'),
            ('S', 180,  '#aaaaaa'),
            ('W', 270,  '#aaaaaa'),
        ]
        for label, deg_from_north_cw, color in cardinals:
            theta = math.radians(-90 + deg_from_north_cw)
            lx = cx + (r - 10) * math.cos(theta)
            ly = cy + (r - 10) * math.sin(theta)
            c.create_text(lx, ly, text=label,
                          font=('TkFixedFont', 9, 'bold'), fill=color)

        # Arrow shows where the rover is pointing.
        # N/W yaw -> compass position: N is up, W is left, E is right.
        # Convert (-sin yaw, cos yaw) (east, north components) into screen
        # pixel deltas (right, up) -> (right, -y) for tk canvas.
        display_yaw = -yaw_radians if self.invert_display_yaw else yaw_radians
        ax = cx + (r - 14) * (-math.sin(display_yaw))
        ay = cy - (r - 14) * math.cos(display_yaw)
        c.create_line(cx, cy, ax, ay, fill='#2ecc71', width=3, arrow=tk.LAST)
        c.create_oval(cx - 3, cy - 3, cx + 3, cy + 3,
                      fill='#2ecc71', outline='')

    def compute_distance_to_target(self):
        if self.robot_x is None or self.robot_y is None or self.target_x is None or self.target_y is None:
            return None
        return math.sqrt((self.target_x - self.robot_x) ** 2 + (self.target_y - self.robot_y) ** 2)

    def format_topic_health(self):
        parts = []
        now = time.time()
        display_order = [
            ('robot_pose', 'pose'),
            ('target', 'target'),
            ('ackermann', 'ack'),
            ('point_turn', 'turn'),
            ('state', 'state'),
            ('gps', 'gps'),
            ('move_type', 'move'),
            ('image', 'image'),
        ]
        for key, label in display_order:
            last = self.last_seen[key]
            if last is None:
                parts.append(f'{label}:never')
            else:
                parts.append(f'{label}:{now - last:.1f}s')
        return ' | '.join(parts)

    def format_publisher_health(self):
        parts = []
        display_order = [
            ('robot_pose', 'pose'),
            ('target', 'target'),
            ('ackermann', 'ack'),
            ('point_turn', 'turn'),
            ('state', 'state'),
            ('gps', 'gps'),
            ('move_type', 'move'),
            ('image', 'image'),
        ]
        for key, label in display_order:
            topic = self.topic_configs[key]
            count = len(self.get_publishers_info_by_topic(topic))
            parts.append(f'{label}:{count}')
        return ' | '.join(parts)

    def maybe_emit_debug_log(self):
        if not self.debug_logging:
            return

        now = time.time()
        if now - self.last_debug_log < self.debug_period:
            return

        with self.lock:
            yaw_deg = self.normalize_yaw_degrees(self.robot_yaw)
            cardinal = self.yaw_to_cardinal(self.robot_yaw)
            summary = (
                f"state={self.current_state} move={self.current_move_type} "
                f"pose_NW=(N={self.robot_north}, W={self.robot_west}) "
                f"pose_canvas_EN=({self.robot_x}, {self.robot_y}) "
                f"yaw={yaw_deg:+.1f}deg ({cardinal}) "
                f"target_NW=(N={self.target_north}, W={self.target_west}) "
                f"cmd_source={self.cmd_source} "
                f"cmd=({self.cmd_vx:.3f}, {self.cmd_vy:.3f}, {self.cmd_wz:.3f}) "
                f"gps=({self.gps_lat}, {self.gps_lon}) "
                f"origin=({self.gps_reference_lat}, {self.gps_reference_lon}) "
                f"counts={self.msg_counts}"
            )

        self.get_logger().info(
            f'[telemetry debug] {summary} publishers=({self.format_publisher_health()})'
        )
        self.last_debug_log = now

    def current_world_spans(self):
        x_span = max(1e-6, self.world_x_max - self.world_x_min)
        y_span = max(1e-6, self.world_y_max - self.world_y_min)
        return x_span, y_span

    def is_geospatial_plot(self):
        return self.world_x_label_name == 'Longitude' and self.world_y_label_name == 'Latitude'

    def world_to_canvas(self, x, y):
        canvas_width, canvas_height = self.get_canvas_size()
        x_span, y_span = self.current_world_spans()
        x_norm = (x - self.world_x_min) / x_span
        y_norm = (y - self.world_y_min) / y_span

        cx = x_norm * canvas_width
        cy = canvas_height - (y_norm * canvas_height)
        return cx, cy

    def world_dx_to_canvas(self, dx):
        canvas_width, _ = self.get_canvas_size()
        x_span, _ = self.current_world_spans()
        return (dx / x_span) * canvas_width

    def world_dy_to_canvas(self, dy):
        _, canvas_height = self.get_canvas_size()
        _, y_span = self.current_world_spans()
        return (dy / y_span) * canvas_height

    def choose_grid_step(self, span):
        target_lines = 8
        raw = max(1e-9, span / target_lines)
        power = 10 ** math.floor(math.log10(raw))
        mults = [1, 2, 5, 10]
        for m in mults:
            step = m * power
            if step >= raw:
                return step
        return 10 * power

    def compute_robot_world_size(self):
        x_span, y_span = self.current_world_spans()
        base = 0.025 * min(x_span, y_span)
        if self.is_geospatial_plot():
            return max(0.00001, base)
        return max(0.8, base)

    def format_axis_value(self, value, axis_name):
        if axis_name in ('Latitude', 'Longitude'):
            return f'{value:.6f}'
        return f'{value:.1f}'

    # ---------------- Logging ----------------
    def start_logging(self):
        self.is_logging = True
        self.status_label.config(text='Logging started')
        self.logging_status_label.config(text='Logging: ON')

    def stop_logging(self):
        self.is_logging = False
        self.status_label.config(text='Logging stopped')
        self.logging_status_label.config(text='Logging: OFF')

    def maybe_log_row(self):
        if not self.is_logging:
            return

        now = time.time()
        if now - self.last_log_time < self.log_period:
            return

        with self.lock:
            row = {
                'timestamp': now,
                'robot_x': self.robot_x,
                'robot_y': self.robot_y,
                'robot_yaw': self.robot_yaw,
                'target_x': self.target_x,
                'target_y': self.target_y,
                'cmd_vx': self.cmd_vx,
                'cmd_vy': self.cmd_vy,
                'cmd_wz': self.cmd_wz,
                'cmd_source': self.cmd_source,
                'current_state': self.current_state,
                'current_move_type': self.current_move_type,
                'gps_lat': self.gps_lat,
                'gps_lon': self.gps_lon,
                'speed': self.speed,
                'distance_to_target': self.compute_distance_to_target(),
            }

        self.logged_rows.append(row)
        self.last_log_time = now

    def save_csv(self):
        filename = os.path.join(self.script_dir, self.default_csv_name)
        fieldnames = [
            'timestamp',
            'robot_x', 'robot_y', 'robot_yaw',
            'target_x', 'target_y',
            'cmd_vx', 'cmd_vy', 'cmd_wz',
            'cmd_source', 'current_state', 'current_move_type',
            'gps_lat', 'gps_lon',
            'speed', 'distance_to_target'
        ]

        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                for row in self.logged_rows:
                    writer.writerow(row)

            self.status_label.config(text=f'CSV saved: {filename}')
        except Exception as e:
            self.status_label.config(text=f'CSV save failed: {e}')

    # ---------------- Drawing ----------------
    def draw_grid(self):
        canvas_width, canvas_height = self.get_canvas_size()
        x_span, y_span = self.current_world_spans()
        x_step = self.choose_grid_step(x_span)
        y_step = self.choose_grid_step(y_span)

        x0 = math.floor(self.world_x_min / x_step) * x_step
        x = x0
        while x <= self.world_x_max + 1e-9:
            cx, _ = self.world_to_canvas(x, self.world_y_min)
            color = 'gray75' if abs(x) < 1e-9 else 'gray88'
            width = 2 if abs(x) < 1e-9 else 1
            self.canvas.create_line(cx, 0, cx, canvas_height, fill=color, width=width)

            if 0 <= cx <= canvas_width:
                self.canvas.create_text(
                    cx + 22,
                    12,
                    text=self.format_axis_value(x, self.world_x_label_name),
                    fill='gray35',
                    font=('TkDefaultFont', 8)
                )
            x += x_step

        y0 = math.floor(self.world_y_min / y_step) * y_step
        y = y0
        while y <= self.world_y_max + 1e-9:
            _, cy = self.world_to_canvas(self.world_x_min, y)
            color = 'gray75' if abs(y) < 1e-9 else 'gray88'
            width = 2 if abs(y) < 1e-9 else 1
            self.canvas.create_line(0, cy, canvas_width, cy, fill=color, width=width)

            if 0 <= cy <= canvas_height:
                self.canvas.create_text(
                    42,
                    cy - 8,
                    text=self.format_axis_value(y, self.world_y_label_name),
                    fill='gray35',
                    font=('TkDefaultFont', 8)
                )
            y += y_step

        self.grid_label.config(
            text=(
                f'Grid step: {self.world_x_label_name}={self.format_axis_value(x_step, self.world_x_label_name)}, '
                f'{self.world_y_label_name}={self.format_axis_value(y_step, self.world_y_label_name)}'
            )
        )

    def draw_border(self):
        canvas_width, canvas_height = self.get_canvas_size()
        self.canvas.create_rectangle(
            2, 2, canvas_width - 2, canvas_height - 2,
            outline='black'
        )

    def draw_waypoints(self):
        for i, (wx, wy) in enumerate(self.waypoints):
            cx, cy = self.world_to_canvas(wx, wy)
            r = 5

            self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r,
                fill='lime',
                outline='green4',
                width=2
            )

            self.canvas.create_text(
                cx + 16, cy - 10,
                text=f'W{i+1}',
                fill='green4',
                font=('TkDefaultFont', 9, 'bold')
        )

    def draw_target(self, x, y):
        cx, cy = self.world_to_canvas(x, y)
        scale_size = self.compute_robot_world_size()
        s = max(8, int(0.5 * min(abs(self.world_dx_to_canvas(scale_size)),
                                  abs(self.world_dy_to_canvas(scale_size)))))
        self.canvas.create_line(cx - s, cy, cx + s, cy, fill='green4', width=2)
        self.canvas.create_line(cx, cy - s, cx, cy + s, fill='green4', width=2)
        self.canvas.create_text(cx + 24, cy - 12, text='Target', fill='green4')

    def compute_drive_direction(self, move_type, speed, fl, fr, pt_wz):
        mt = move_type.lower() if move_type else ''
        if 'point_turn' in mt:
            if abs(pt_wz) < 0.01:
                return 'Stopped', None
            return ('Rotating Left', None) if pt_wz > 0 else ('Rotating Right', None)
        if 'ackerman' in mt or 'stanley' in mt:
            if abs(speed) < 0.01:
                return 'Stopped', None
            steer = (fl + fr) / 2.0
            angle_deg = steer if speed > 0 else (180.0 + steer)
            # map angle to 8-sector label (0° = forward, CW positive)
            sectors = [
                'Front', 'Front-Right', 'Right', 'Back-Right',
                'Back', 'Back-Left', 'Left', 'Front-Left',
            ]
            idx = int((angle_deg % 360 + 22.5) / 45.0) % 8
            return sectors[idx], angle_deg
        return 'Stopped', None

    def draw_direction_compass(self, direction, angle_deg):
        c = self.direction_canvas
        c.delete('all')
        cx, cy, r = 60, 60, 48
        labels = ['F', 'FR', 'R', 'BR', 'B', 'BL', 'L', 'FL']
        colors = {
            'Front': '#2ecc71', 'Front-Right': '#2ecc71', 'Right': '#2ecc71',
            'Back-Right': '#e74c3c', 'Back': '#e74c3c', 'Back-Left': '#e74c3c',
            'Left': '#3498db', 'Front-Left': '#3498db',
            'Rotating Left': '#f39c12', 'Rotating Right': '#f39c12',
            'Stopped': '#95a5a6',
        }
        highlight = colors.get(direction, '#95a5a6')
        sector_map = {
            'Front': 0, 'Front-Right': 1, 'Right': 2, 'Back-Right': 3,
            'Back': 4, 'Back-Left': 5, 'Left': 6, 'Front-Left': 7,
        }
        for i, lbl in enumerate(labels):
            theta = math.radians(-90 + i * 45)
            lx = cx + (r - 10) * math.cos(theta)
            ly = cy + (r - 10) * math.sin(theta)
            seg_active = sector_map.get(direction) == i
            c.create_text(lx, ly, text=lbl, font=('TkFixedFont', 7, 'bold'),
                          fill=highlight if seg_active else '#aaaaaa')
        c.create_oval(cx - r, cy - r, cx + r, cy + r, outline='#555555', width=1)
        # draw arrow
        if direction in ('Rotating Left', 'Rotating Right'):
            arc_start = 30 if direction == 'Rotating Left' else 120
            c.create_arc(cx - 28, cy - 28, cx + 28, cy + 28,
                         start=arc_start, extent=300, style=tk.ARC,
                         outline=highlight, width=2)
            tip_theta = math.radians(-90 + (300 if direction == 'Rotating Left' else -300))
        elif angle_deg is not None:
            arrow_theta = math.radians(-90 + angle_deg)
            ax = cx + (r - 14) * math.cos(arrow_theta)
            ay = cy + (r - 14) * math.sin(arrow_theta)
            c.create_line(cx, cy, ax, ay, fill=highlight, width=3, arrow=tk.LAST)
        if direction == 'Stopped':
            c.create_text(cx, cy, text='●', fill='#95a5a6', font=('TkFixedFont', 14))

    def draw_robot_path(self):
        if len(self.robot_path) < 2:
            return

        pts = []
        for x, y in self.robot_path:
            cx, cy = self.world_to_canvas(x, y)
            pts.extend([cx, cy])

        self.canvas.create_line(*pts, fill='orange', width=2)

    def draw_robot(self, x, y, yaw):
        cx, cy = self.world_to_canvas(x, y)

        robot_world_size = self.compute_robot_world_size()
        half_w_px = max(5, abs(self.world_dx_to_canvas(robot_world_size)) / 2.0)
        half_h_px = max(5, abs(self.world_dy_to_canvas(robot_world_size)) / 2.0)

        self.canvas.create_rectangle(
            cx - half_w_px,
            cy - half_h_px,
            cx + half_w_px,
            cy + half_h_px,
            fill='red',
            outline='black'
        )

        # Heading arrow.
        # yaw is published in the autonomy's N/W convention (CCW-positive
        # looking down: 0 = facing N, +pi/2 = facing W). The canvas world
        # frame is (east, north) with +x going right and +y going up, so the
        # rover's facing-direction unit vector in canvas coords is:
        #   east_component  = -sin(yaw)
        #   north_component =  cos(yaw)
        # Apply the canvas y-flip (cy decreases when going up on screen).
        display_yaw = -yaw if self.invert_display_yaw else yaw
        heading_len = max(16, 1.2 * max(half_w_px, half_h_px))
        hx = cx + heading_len * (-math.sin(display_yaw))
        hy = cy - heading_len * math.cos(display_yaw)

        self.canvas.create_line(cx, cy, hx, hy, fill='black', width=2, arrow=tk.LAST)
        self.canvas.create_text(cx + 22, cy + 15, text='Robot', fill='red')

        self.robot_size_label.config(
            text=f'Robot size (world): {self.format_axis_value(robot_world_size, self.world_x_label_name)}'
        )

    # ---------------- UI ----------------
    def update_ui(self):
        if self._closing:
            return

        if time.time() - self.last_image_topic_refresh > 3.0:
            self.refresh_image_topics()

        self.maybe_log_row()
        self.maybe_emit_debug_log()

        with self.lock:
            robot_x = self.robot_x          # canvas East (m)
            robot_y = self.robot_y          # canvas North (m)
            robot_north = self.robot_north  # autonomy frame North (m)
            robot_west = self.robot_west    # autonomy frame West (m)
            robot_yaw = self.robot_yaw
            robot_pose_frame_id = self.robot_pose_frame_id
            target_x = self.target_x          # canvas East (m)
            target_y = self.target_y          # canvas North (m)
            target_north = self.target_north  # autonomy frame North (m)
            target_west = self.target_west    # autonomy frame West (m)
            speed = self.speed
            cmd_vx = self.cmd_vx
            cmd_vy = self.cmd_vy
            cmd_wz = self.cmd_wz
            cmd_source = self.cmd_source
            ackermann_speed = self.ackermann_speed
            ackermann_fl = self.ackermann_fl
            ackermann_fr = self.ackermann_fr
            point_turn_wz = self.point_turn_wz
            current_state = self.current_state
            current_move_type = self.current_move_type
            gps_lat = self.gps_lat
            gps_lon = self.gps_lon
            gps_origin_lat = self.gps_reference_lat
            gps_origin_lon = self.gps_reference_lon

        plot_robot_x, plot_robot_y = self.normalize_position_for_plot(
            robot_x,
            robot_y,
            robot_pose_frame_id
        )
        gps_plot_x, gps_plot_y = self.normalize_position_for_plot(gps_lon, gps_lat, 'gps')
        if plot_robot_x is None or plot_robot_y is None:
            plot_robot_x = gps_plot_x
            plot_robot_y = gps_plot_y

        plot_target_x, plot_target_y = self.normalize_position_for_plot(target_x, target_y)
        self.expand_base_bounds_to_include([
            (plot_robot_x, plot_robot_y),
            (plot_target_x, plot_target_y),
        ])

        self.canvas.delete('all')
        self.draw_grid()
        self.draw_border()
        self.draw_waypoints()
        self.draw_robot_path()
        self.render_camera_feed()

        if plot_target_x is not None and plot_target_y is not None:
            self.draw_target(plot_target_x, plot_target_y)

        if plot_robot_x is not None and plot_robot_y is not None:
            self.append_robot_path(plot_robot_x, plot_robot_y)
            self.draw_robot(plot_robot_x, plot_robot_y, robot_yaw)

        # ---- Heading panel (absolute heading from rtk_localization)
        yaw_deg = self.normalize_yaw_degrees(robot_yaw)
        cardinal = self.yaw_to_cardinal(robot_yaw)
        invert_note = ' (display inverted)' if self.invert_display_yaw else ''
        self.heading_label.config(
            text=f'Yaw: {yaw_deg:+6.1f}° ({cardinal}){invert_note}'
        )
        self.draw_heading_compass(robot_yaw)

        # ---- Robot State (autonomy + canvas frames side by side)
        if robot_north is not None and robot_west is not None:
            self.robot_north_label.config(text=f'North: {robot_north:+7.2f} m')
            self.robot_west_label.config(text=f'West:  {robot_west:+7.2f} m')
            self.robot_canvas_label.config(
                text=f'Canvas (E, N): ({-robot_west:+7.2f}, {robot_north:+7.2f}) m'
            )
        else:
            self.robot_north_label.config(text='North: -- m')
            self.robot_west_label.config(text='West:  -- m')
            self.robot_canvas_label.config(text='Canvas (E, N): --')
        self.speed_label.config(text=f'Speed: {speed:.3f} m/s')
        self.gps_lat_label.config(
            text=f'GPS Lat: {gps_lat:.7f}' if gps_lat is not None else 'GPS Lat: --'
        )
        self.gps_lon_label.config(
            text=f'GPS Lon: {gps_lon:.7f}' if gps_lon is not None else 'GPS Lon: --'
        )

        # ---- GPS origin status box
        if gps_origin_lat is not None and gps_origin_lon is not None:
            self.origin_status_label.config(
                text='Origin: captured', foreground='#27ae60'
            )
            self.origin_lat_label.config(text=f'Lat: {gps_origin_lat:.7f}')
            self.origin_lon_label.config(text=f'Lon: {gps_origin_lon:.7f}')
        else:
            self.origin_status_label.config(
                text='Origin: waiting for first fix...', foreground='#c0392b'
            )
            self.origin_lat_label.config(text='Lat: --')
            self.origin_lon_label.config(text='Lon: --')

        # ---- Target (autonomy + canvas frames + bearing)
        if target_north is not None and target_west is not None:
            self.target_north_label.config(text=f'Target North: {target_north:+7.2f} m')
            self.target_west_label.config(text=f'Target West:  {target_west:+7.2f} m')
            self.target_canvas_label.config(
                text=f'Canvas (E, N): ({-target_west:+7.2f}, {target_north:+7.2f}) m'
            )
        else:
            self.target_north_label.config(text='Target North: --')
            self.target_west_label.config(text='Target West:  --')
            self.target_canvas_label.config(text='Canvas (E, N): --')

        dist = None
        bearing_deg = None
        if (
            plot_robot_x is not None and plot_robot_y is not None
            and plot_target_x is not None and plot_target_y is not None
        ):
            dx_e = plot_target_x - plot_robot_x  # east
            dy_n = plot_target_y - plot_robot_y  # north
            dist = math.hypot(dx_e, dy_n)
            # Bearing in NW frame (0=N, +90=W) so it matches the autonomy
            # controller's heading_error math: angle_to_target = atan2(W,N).
            bearing_deg = math.degrees(math.atan2(-dx_e, dy_n))
        self.dist_label.config(
            text=f'Distance: {dist:.3f} m' if dist is not None else 'Distance: --'
        )
        if bearing_deg is not None:
            self.bearing_label.config(
                text=f'Bearing (N/W): {bearing_deg:+6.1f}° ({self.yaw_to_cardinal(math.radians(bearing_deg))})'
            )
        else:
            self.bearing_label.config(text='Bearing: --')

        direction, angle_deg = self.compute_drive_direction(
            current_move_type, ackermann_speed, ackermann_fl, ackermann_fr, point_turn_wz
        )
        self.direction_label.config(text=direction)
        self.draw_direction_compass(direction, angle_deg)

        self.cmd_vx_label.config(text=f'vx: {cmd_vx:.3f} m/s')
        self.cmd_vy_label.config(text=f'vy: {cmd_vy:.3f} m/s')
        self.cmd_wz_label.config(text=f'wz: {cmd_wz:.3f} rad/s')
        self.cmd_source_label.config(text=f'Source: {cmd_source}')
        self.ack_label.config(
            text=f'Ackermann: vel={ackermann_speed:.3f} fl={ackermann_fl:.2f} fr={ackermann_fr:.2f}'
        )
        self.point_turn_label.config(text=f'Point turn wz: {point_turn_wz:.3f}')
        self.state_value_label.config(text=f'Current state: {current_state}')
        self.move_type_label.config(text=f'Move type: {current_move_type}')
        self.topic_health_label.config(text=f'Topic health: {self.format_topic_health()}')
        publisher_health = self.format_publisher_health()
        self.publisher_health_label.config(text=f'Publishers: {publisher_health}')

        self.wp_label.config(text=f'Waypoints: {len(self.waypoints)}')
        self.bounds_label.config(
            text=(
                f'Bounds:\n'
                f'{self.world_x_label_name} [{self.format_axis_value(self.world_x_min, self.world_x_label_name)}, '
                f'{self.format_axis_value(self.world_x_max, self.world_x_label_name)}]\n'
                f'{self.world_y_label_name} [{self.format_axis_value(self.world_y_min, self.world_y_label_name)}, '
                f'{self.format_axis_value(self.world_y_max, self.world_y_label_name)}]'
            )
        )

        canvas_width, canvas_height = self.get_canvas_size()
        self.canvas_size_label.config(text=f'Canvas: {canvas_width} x {canvas_height}')
        self.zoom_label.config(text=f'Zoom: {self.zoom_factor:.2f}x')
        self.image_status_label.config(wraplength=max(220, self.image_canvas.winfo_width() - 20))
        self.topic_health_label.config(wraplength=max(220, self.side_frame.winfo_width() - 20))
        self.publisher_health_label.config(wraplength=max(220, self.side_frame.winfo_width() - 20))

        self.logging_status_label.config(text=f'Logging: {"ON" if self.is_logging else "OFF"}')
        self.log_count_label.config(text=f'Rows logged: {len(self.logged_rows)}')
        self.status_label.config(text=f'Running | Latest command source: {cmd_source} | {publisher_health}')

        if not self._closing:
            self.root.after(self.ui_period_ms, self.update_ui)

    def on_close(self):
        self.get_logger().info('Closing Tkinter window.')

        if self._closing:
            return

        self._closing = True

        try:
            self.status_label.config(text='Closing...')
        except Exception:
            pass

        try:
            self.root.after(0, self.root.quit)
        except Exception:
            pass


def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(f"[ROS spin thread] Exception: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = RealtimeRobotPlotter()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        try:
            spin_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            node.root.destroy()
        except Exception:
            pass


if __name__ == '__main__':
    main()
