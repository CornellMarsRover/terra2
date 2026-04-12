#!/usr/bin/env python3

import csv
import math
import os
import threading
import time
import tkinter as tk
from tkinter import ttk
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Odometry


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_WAYPOINT_FILE = os.path.join(PACKAGE_DIR, 'config', 'waypoints.yaml')

class RealtimeRobotPlotter(Node):
    def __init__(self):
        super().__init__('realtime_robot_plotter')

        self._closing = False
        self.lock = threading.Lock()

        # ---------------- Parameters ----------------
        self.declare_parameter('robot_pose_topic', '/autonomy/pose/robot/global')
        self.declare_parameter('target_topic', '/autonomy/target_object/position')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_pose_type', 'pose_stamped')   # pose_stamped / odometry
        self.declare_parameter('target_type', 'point')              # point / pose_stamped
        self.declare_parameter('waypoints_file', DEFAULT_WAYPOINT_FILE)

        self.declare_parameter('canvas_width', 950)
        self.declare_parameter('canvas_height', 680)
        self.declare_parameter('ui_update_hz', 20.0)
        self.declare_parameter('trail_length', 800)

        # fallback bounds if no waypoints
        self.declare_parameter('world_x_min', -50.0)
        self.declare_parameter('world_x_max', 50.0)
        self.declare_parameter('world_y_min', -50.0)
        self.declare_parameter('world_y_max', 50.0)

        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.robot_pose_type = self.get_parameter('robot_pose_type').value
        self.target_type = self.get_parameter('target_type').value
        self.waypoints_file = self.get_parameter('waypoints_file').value or DEFAULT_WAYPOINT_FILE

        self.initial_canvas_width = int(self.get_parameter('canvas_width').value)
        self.initial_canvas_height = int(self.get_parameter('canvas_height').value)
        self.ui_update_hz = float(self.get_parameter('ui_update_hz').value)
        self.trail_length = int(self.get_parameter('trail_length').value)

        self.world_x_min = float(self.get_parameter('world_x_min').value)
        self.world_x_max = float(self.get_parameter('world_x_max').value)
        self.world_y_min = float(self.get_parameter('world_y_min').value)
        self.world_y_max = float(self.get_parameter('world_y_max').value)

        # ---------------- State ----------------
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = 0.0

        self.target_x = None
        self.target_y = None

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.speed = 0.0

        self.robot_path = []
        self.waypoints = []

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
        elif self.target_type == 'pose_stamped':
            self.target_sub = self.create_subscription(
                PoseStamped, self.target_topic, self.target_pose_stamped_cb, 10
            )
        else:
            raise ValueError(f"Unsupported target_type: {self.target_type}")

        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_cb, 10
        )

        self.get_logger().info('Realtime Robot Plotter started.')
        self.get_logger().info(f'Robot pose topic: {self.robot_pose_topic} ({self.robot_pose_type})')
        self.get_logger().info(f'Target topic: {self.target_topic} ({self.target_type})')
        self.get_logger().info(f'Cmd vel topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoint(s) from config.')

        # ---------------- Tkinter UI ----------------
        self.root = tk.Tk()
        self.root.title("ROS 2 Live Telemetry Tool")
        self.root.geometry(f"{self.initial_canvas_width + 360}x{self.initial_canvas_height + 40}")

        main_frame = ttk.Frame(self.root, padding=8)
        main_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(
            main_frame,
            width=self.initial_canvas_width,
            height=self.initial_canvas_height,
            bg='white'
        )
        self.canvas.grid(row=0, column=0, rowspan=30, sticky='nsew', padx=(0, 10))

        side = ttk.Frame(main_frame)
        side.grid(row=0, column=1, sticky='nsew')

        # ---- Robot box
        robot_box = ttk.LabelFrame(side, text='Robot State', padding=8)
        robot_box.pack(fill='x', pady=4)
        self.robot_x_label = ttk.Label(robot_box, text='X: --')
        self.robot_x_label.pack(anchor='w')
        self.robot_y_label = ttk.Label(robot_box, text='Y: --')
        self.robot_y_label.pack(anchor='w')
        self.robot_yaw_label = ttk.Label(robot_box, text='Yaw: --')
        self.robot_yaw_label.pack(anchor='w')
        self.speed_label = ttk.Label(robot_box, text='Speed: -- m/s')
        self.speed_label.pack(anchor='w')

        # ---- Target box
        target_box = ttk.LabelFrame(side, text='Target State', padding=8)
        target_box.pack(fill='x', pady=4)
        self.target_x_label = ttk.Label(target_box, text='Target X: --')
        self.target_x_label.pack(anchor='w')
        self.target_y_label = ttk.Label(target_box, text='Target Y: --')
        self.target_y_label.pack(anchor='w')
        self.dist_label = ttk.Label(target_box, text='Distance: --')
        self.dist_label.pack(anchor='w')

        # ---- Command box
        cmd_box = ttk.LabelFrame(side, text='cmd_vel Telemetry', padding=8)
        cmd_box.pack(fill='x', pady=4)
        self.cmd_vx_label = ttk.Label(cmd_box, text='vx: --')
        self.cmd_vx_label.pack(anchor='w')
        self.cmd_vy_label = ttk.Label(cmd_box, text='vy: --')
        self.cmd_vy_label.pack(anchor='w')
        self.cmd_wz_label = ttk.Label(cmd_box, text='wz: --')
        self.cmd_wz_label.pack(anchor='w')

        # ---- Map box
        map_box = ttk.LabelFrame(side, text='Map / Plot Info', padding=8)
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
        log_box = ttk.LabelFrame(side, text='Logging', padding=8)
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
        status_box = ttk.LabelFrame(side, text='Status', padding=8)
        status_box.pack(fill='x', pady=4)
        self.status_label = ttk.Label(status_box, text='Running')
        self.status_label.pack(anchor='w')

        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=0)
        main_frame.rowconfigure(0, weight=1)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.ui_period_ms = max(1, int(1000.0 / max(self.ui_update_hz, 1.0)))
        self.root.after(self.ui_period_ms, self.update_ui)

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
                for wp in loaded:
                    self.waypoints.append((float(wp['x']), float(wp['y'])))
                self.get_logger().info(f'Waypoints parsed successfully: {len(self.waypoints)}')
                self.auto_set_bounds_from_waypoints()
                return

            if all(('latitude' in wp and 'longitude' in wp) for wp in loaded):
                lat0 = float(loaded[0]['latitude'])
                lon0 = float(loaded[0]['longitude'])

                meters_per_deg_lat = 111320.0
                meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))

                for wp in loaded:
                    lat = float(wp['latitude'])
                    lon = float(wp['longitude'])

                    x = (lon - lon0) * meters_per_deg_lon
                    y = (lat - lat0) * meters_per_deg_lat
                    self.waypoints.append((x, y))

                self.get_logger().info(
                    f'Waypoints parsed successfully from GPS format: {len(self.waypoints)}'
                )
                self.auto_set_bounds_from_waypoints()
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

        span_x = max(1.0, max_x - min_x)
        span_y = max(1.0, max_y - min_y)

        margin_x = max(5.0, 0.15 * span_x)
        margin_y = max(5.0, 0.15 * span_y)

        self.world_x_min = min_x - margin_x
        self.world_x_max = max_x + margin_x
        self.world_y_min = min_y - margin_y
        self.world_y_max = max_y + margin_y

        self.get_logger().info(
            f'Auto plot bounds from waypoints -> '
            f'X:[{self.world_x_min:.2f}, {self.world_x_max:.2f}] '
            f'Y:[{self.world_y_min:.2f}, {self.world_y_max:.2f}]'
        )

    # ---------------- ROS Callbacks ----------------
    def robot_pose_pose_stamped_cb(self, msg: PoseStamped):
        with self.lock:
            self.robot_x = msg.pose.position.x
            self.robot_y = msg.pose.position.y
            self.robot_yaw = self.quaternion_to_yaw(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
            self.append_robot_path(self.robot_x, self.robot_y)

    def robot_pose_odom_cb(self, msg: Odometry):
        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_yaw = self.quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            self.append_robot_path(self.robot_x, self.robot_y)

    def target_point_cb(self, msg: Point):
        with self.lock:
            self.target_x = msg.x
            self.target_y = msg.y

    def target_pose_stamped_cb(self, msg: PoseStamped):
        with self.lock:
            self.target_x = msg.pose.position.x
            self.target_y = msg.pose.position.y

    def cmd_vel_cb(self, msg: Twist):
        with self.lock:
            self.cmd_vx = msg.linear.x
            self.cmd_vy = msg.linear.y
            self.cmd_wz = msg.angular.z
            self.speed = math.sqrt(self.cmd_vx ** 2 + self.cmd_vy ** 2)

            print(
                f"[cmd_vel telemetry] "
                f"vx={self.cmd_vx:.3f} m/s, "
                f"vy={self.cmd_vy:.3f} m/s, "
                f"wz={self.cmd_wz:.3f} rad/s, "
                f"speed={self.speed:.3f} m/s"
            )

    # ---------------- Helpers ----------------
    def append_robot_path(self, x, y):
        self.robot_path.append((x, y))
        if len(self.robot_path) > self.trail_length:
            self.robot_path.pop(0)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def compute_distance_to_target(self):
        if self.robot_x is None or self.robot_y is None or self.target_x is None or self.target_y is None:
            return None
        return math.sqrt((self.target_x - self.robot_x) ** 2 + (self.target_y - self.robot_y) ** 2)

    def current_world_spans(self):
        x_span = max(1e-6, self.world_x_max - self.world_x_min)
        y_span = max(1e-6, self.world_y_max - self.world_y_min)
        return x_span, y_span

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
        return max(0.8, base)

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
                self.canvas.create_text(cx + 15, 12, text=f'{x:.1f}', fill='gray35', font=('TkDefaultFont', 8))
            x += x_step

        y0 = math.floor(self.world_y_min / y_step) * y_step
        y = y0
        while y <= self.world_y_max + 1e-9:
            _, cy = self.world_to_canvas(self.world_x_min, y)
            color = 'gray75' if abs(y) < 1e-9 else 'gray88'
            width = 2 if abs(y) < 1e-9 else 1
            self.canvas.create_line(0, cy, canvas_width, cy, fill=color, width=width)

            if 0 <= cy <= canvas_height:
                self.canvas.create_text(30, cy - 8, text=f'{y:.1f}', fill='gray35', font=('TkDefaultFont', 8))
            y += y_step

        self.grid_label.config(text=f'Grid step: dx={x_step:.2f}, dy={y_step:.2f}')

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

        heading_len = max(16, 1.2 * max(half_w_px, half_h_px))
        hx = cx + heading_len * math.cos(yaw)
        hy = cy - heading_len * math.sin(yaw)

        self.canvas.create_line(cx, cy, hx, hy, fill='black', width=2, arrow=tk.LAST)
        self.canvas.create_text(cx + 22, cy + 15, text='Robot', fill='red')

        self.robot_size_label.config(text=f'Robot size (world): {robot_world_size:.2f}')

    # ---------------- UI ----------------
    def update_ui(self):
        if self._closing:
            return

        self.maybe_log_row()

        with self.lock:
            robot_x = self.robot_x
            robot_y = self.robot_y
            robot_yaw = self.robot_yaw
            target_x = self.target_x
            target_y = self.target_y
            speed = self.speed
            cmd_vx = self.cmd_vx
            cmd_vy = self.cmd_vy
            cmd_wz = self.cmd_wz

        self.canvas.delete('all')
        self.draw_grid()
        self.draw_border()
        self.draw_waypoints()
        self.draw_robot_path()

        if target_x is not None and target_y is not None:
            self.draw_target(target_x, target_y)

        if robot_x is not None and robot_y is not None:
            self.draw_robot(robot_x, robot_y, robot_yaw)

        self.robot_x_label.config(text=f'X: {robot_x:.3f}' if robot_x is not None else 'X: --')
        self.robot_y_label.config(text=f'Y: {robot_y:.3f}' if robot_y is not None else 'Y: --')
        self.robot_yaw_label.config(text=f'Yaw: {robot_yaw:.3f} rad')
        self.speed_label.config(text=f'Speed: {speed:.3f} m/s')

        self.target_x_label.config(text=f'Target X: {target_x:.3f}' if target_x is not None else 'Target X: --')
        self.target_y_label.config(text=f'Target Y: {target_y:.3f}' if target_y is not None else 'Target Y: --')

        dist = self.compute_distance_to_target()
        self.dist_label.config(text=f'Distance: {dist:.3f} m' if dist is not None else 'Distance: --')

        self.cmd_vx_label.config(text=f'vx: {cmd_vx:.3f} m/s')
        self.cmd_vy_label.config(text=f'vy: {cmd_vy:.3f} m/s')
        self.cmd_wz_label.config(text=f'wz: {cmd_wz:.3f} rad/s')

        self.wp_label.config(text=f'Waypoints: {len(self.waypoints)}')
        self.bounds_label.config(
            text=(
                f'Bounds:\n'
                f'X [{self.world_x_min:.2f}, {self.world_x_max:.2f}]\n'
                f'Y [{self.world_y_min:.2f}, {self.world_y_max:.2f}]'
            )
        )

        canvas_width, canvas_height = self.get_canvas_size()
        self.canvas_size_label.config(text=f'Canvas: {canvas_width} x {canvas_height}')

        self.logging_status_label.config(text=f'Logging: {"ON" if self.is_logging else "OFF"}')
        self.log_count_label.config(text=f'Rows logged: {len(self.logged_rows)}')

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