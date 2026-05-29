#!/usr/bin/env python3
import math
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


EARTH_RADIUS_M = 6378137.0
STALE_FIX_SECONDS = 3.0


class BasicGpsGui(Node):
    def __init__(self):
        super().__init__('basic_gps_gui')
        self.declare_parameter('gps_topic', '/rtk/navsatfix_data')

        self.gps_topic = self.get_parameter('gps_topic').value
        self.lock = threading.Lock()
        self.latest_lat = None
        self.latest_lon = None
        self.latest_stamp = None
        self.targets = []
        self.target_errors = []
        self.closing = False

        self.subscription = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            10,
        )

        self.root = tk.Tk()
        self.root.title('Basic GNSS Delivery GUI')
        self.root.geometry('820x620')
        self.root.minsize(720, 520)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.build_ui()
        self.root.after(200, self.update_ui)

        self.get_logger().info(f'Basic GNSS Delivery GUI listening on {self.gps_topic}')

    def build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(2, weight=1)

        current_box = ttk.LabelFrame(main, text='Current Rover GNSS', padding=12)
        current_box.grid(row=0, column=0, sticky='ew')
        current_box.columnconfigure(1, weight=1)

        self.lat_value = tk.StringVar(value='--')
        self.lon_value = tk.StringVar(value='--')
        self.status_value = tk.StringVar(value='Waiting for GPS')
        self.age_value = tk.StringVar(value='--')
        self.topic_value = tk.StringVar(value=self.gps_topic)

        big_font = ('TkDefaultFont', 18, 'bold')
        ttk.Label(current_box, text='Latitude:').grid(row=0, column=0, sticky='w', pady=2)
        ttk.Label(current_box, textvariable=self.lat_value, font=big_font).grid(row=0, column=1, sticky='w', pady=2)
        ttk.Label(current_box, text='Longitude:').grid(row=1, column=0, sticky='w', pady=2)
        ttk.Label(current_box, textvariable=self.lon_value, font=big_font).grid(row=1, column=1, sticky='w', pady=2)
        ttk.Label(current_box, text='GPS Status:').grid(row=2, column=0, sticky='w', pady=2)
        ttk.Label(current_box, textvariable=self.status_value).grid(row=2, column=1, sticky='w', pady=2)
        ttk.Label(current_box, text='Last Update:').grid(row=3, column=0, sticky='w', pady=2)
        ttk.Label(current_box, textvariable=self.age_value).grid(row=3, column=1, sticky='w', pady=2)
        ttk.Label(current_box, text='Topic:').grid(row=4, column=0, sticky='w', pady=2)
        ttk.Label(current_box, textvariable=self.topic_value).grid(row=4, column=1, sticky='w', pady=2)

        input_box = ttk.LabelFrame(main, text='Target Coordinates', padding=8)
        input_box.grid(row=1, column=0, sticky='nsew', pady=(10, 0))
        input_box.columnconfigure(0, weight=1)
        input_box.rowconfigure(0, weight=1)

        self.target_text = tk.Text(input_box, height=7, wrap='none')
        self.target_text.grid(row=0, column=0, sticky='nsew')
        self.target_text.insert(
            '1.0',
            'pickup_1, 42.444823, -76.483590\n'
            'delivery_1, 42.445055, -76.483674\n',
        )

        text_scroll = ttk.Scrollbar(input_box, orient=tk.VERTICAL, command=self.target_text.yview)
        text_scroll.grid(row=0, column=1, sticky='ns')
        self.target_text.configure(yscrollcommand=text_scroll.set)

        controls = ttk.Frame(input_box)
        controls.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        ttk.Button(controls, text='Parse Targets', command=self.parse_targets).pack(side=tk.LEFT)
        self.parse_status = tk.StringVar(value='Enter one target per line: name, latitude, longitude')
        ttk.Label(controls, textvariable=self.parse_status).pack(side=tk.LEFT, padx=(12, 0))

        table_box = ttk.LabelFrame(main, text='Parsed Targets', padding=8)
        table_box.grid(row=2, column=0, sticky='nsew', pady=(10, 0))
        table_box.columnconfigure(0, weight=1)
        table_box.rowconfigure(0, weight=1)

        columns = ('name', 'latitude', 'longitude', 'distance')
        self.target_table = ttk.Treeview(table_box, columns=columns, show='headings', height=10)
        self.target_table.heading('name', text='Name')
        self.target_table.heading('latitude', text='Latitude')
        self.target_table.heading('longitude', text='Longitude')
        self.target_table.heading('distance', text='Distance')
        self.target_table.column('name', width=220, anchor='w')
        self.target_table.column('latitude', width=160, anchor='e')
        self.target_table.column('longitude', width=160, anchor='e')
        self.target_table.column('distance', width=120, anchor='e')
        self.target_table.grid(row=0, column=0, sticky='nsew')

        table_scroll = ttk.Scrollbar(table_box, orient=tk.VERTICAL, command=self.target_table.yview)
        table_scroll.grid(row=0, column=1, sticky='ns')
        self.target_table.configure(yscrollcommand=table_scroll.set)

        self.parse_targets()

    def gps_callback(self, msg: NavSatFix):
        lat = msg.latitude
        lon = msg.longitude
        if not self.is_valid_coordinate(lat, lon):
            return

        with self.lock:
            self.latest_lat = lat
            self.latest_lon = lon
            self.latest_stamp = time.time()

    @staticmethod
    def is_valid_coordinate(lat, lon):
        if lat is None or lon is None:
            return False
        if not math.isfinite(lat) or not math.isfinite(lon):
            return False
        return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0

    def parse_targets(self):
        text = self.target_text.get('1.0', tk.END)
        targets = []
        errors = []

        for line_number, raw_line in enumerate(text.splitlines(), start=1):
            line = raw_line.strip()
            if not line:
                continue

            parts = [part.strip() for part in line.split(',')]
            if len(parts) != 3:
                errors.append(f'line {line_number}: expected name, latitude, longitude')
                continue

            name, lat_text, lon_text = parts
            if not name:
                errors.append(f'line {line_number}: missing name')
                continue

            try:
                lat = float(lat_text)
                lon = float(lon_text)
            except ValueError:
                errors.append(f'line {line_number}: invalid latitude/longitude')
                continue

            if not self.is_valid_coordinate(lat, lon):
                errors.append(f'line {line_number}: coordinate out of range')
                continue

            targets.append({'name': name, 'lat': lat, 'lon': lon})

        with self.lock:
            self.targets = targets
            self.target_errors = errors

        self.refresh_target_table()

    def update_ui(self):
        with self.lock:
            lat = self.latest_lat
            lon = self.latest_lon
            stamp = self.latest_stamp

        now = time.time()
        if stamp is None:
            self.lat_value.set('--')
            self.lon_value.set('--')
            self.status_value.set('Waiting for GPS')
            self.age_value.set('--')
        else:
            age = now - stamp
            self.lat_value.set(f'{lat:.8f}')
            self.lon_value.set(f'{lon:.8f}')
            self.status_value.set('Stale GPS' if age > STALE_FIX_SECONDS else 'Receiving GPS')
            self.age_value.set(f'{age:.1f}s ago')

        self.refresh_target_table()
        if not self.closing:
            self.root.after(200, self.update_ui)

    def refresh_target_table(self):
        with self.lock:
            lat = self.latest_lat
            lon = self.latest_lon
            targets = list(self.targets)
            errors = list(self.target_errors)

        for row_id in self.target_table.get_children():
            self.target_table.delete(row_id)

        for target in targets:
            distance = '--'
            if self.is_valid_coordinate(lat, lon):
                distance = f'{self.distance_meters(lat, lon, target["lat"], target["lon"]):.1f} m'

            self.target_table.insert(
                '',
                tk.END,
                values=(
                    target['name'],
                    f'{target["lat"]:.8f}',
                    f'{target["lon"]:.8f}',
                    distance,
                ),
            )

        if errors:
            self.parse_status.set(f'Parsed {len(targets)} target(s); {len(errors)} bad line(s): {errors[0]}')
        else:
            self.parse_status.set(f'Parsed {len(targets)} target(s)')

    @staticmethod
    def distance_meters(lat1, lon1, lat2, lon2):
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        lon1_rad = math.radians(lon1)
        lon2_rad = math.radians(lon2)
        mean_lat = (lat1_rad + lat2_rad) / 2.0

        north = (lat2_rad - lat1_rad) * EARTH_RADIUS_M
        east = (lon2_rad - lon1_rad) * EARTH_RADIUS_M * math.cos(mean_lat)
        return math.sqrt(north * north + east * east)

    def on_close(self):
        self.closing = True
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def spin_node(node):
    while rclpy.ok() and not node.closing:
        rclpy.spin_once(node, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = BasicGpsGui()
    ros_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    ros_thread.start()

    try:
        node.run()
    finally:
        node.closing = True
        ros_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
