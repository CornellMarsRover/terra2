#!/usr/bin/env python3
"""Jetson/NVIDIA ZED SDK point cloud viewer.

Live ZED camera feed + depth + point cloud capture.

Controls:
  q / ESC  Quit
  p        Save current point cloud to PLY
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from http import server

import cv2
import numpy as np

try:
    import pyzed.sl as sl
except ImportError as exc:
    print("Failed to import pyzed.sl. Install the ZED SDK Python API first.")
    print("See: https://www.stereolabs.com/docs/")
    raise SystemExit(1) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream ZED feed, compute depth point cloud, and show live display."
    )
    parser.add_argument(
        "--resolution",
        choices=["HD2K", "HD1080", "HD720", "VGA"],
        default="HD720",
        help="Camera capture resolution (default: HD720).",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Camera FPS (default: 30).",
    )
    parser.add_argument(
        "--depth-mode",
        choices=["NEURAL", "ULTRA", "QUALITY", "PERFORMANCE"],
        default="NEURAL",
        help="Depth quality mode (default: NEURAL).",
    )
    parser.add_argument(
        "--unit",
        choices=["METER", "CENTIMETER", "MILLIMETER"],
        default="METER",
        help="Depth unit used by SDK (default: meter).",
    )
    parser.add_argument(
        "--out-dir",
        default="captures",
        help="Directory to save point cloud snapshots (default: captures).",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable local GUI window output (useful over SSH).",
    )
    parser.add_argument(
        "--stream-port",
        type=int,
        default=8080,
        help="MJPEG HTTP stream port for remote viewing (default: 8080). Set 0 to disable stream.",
    )
    parser.add_argument(
        "--stream-host",
        default="0.0.0.0",
        help="Bind host for MJPEG stream server (default: 0.0.0.0).",
    )
    return parser.parse_args()


def resolution_from_name(name: str) -> sl.RESOLUTION:
    return {
        "HD2K": sl.RESOLUTION.HD2K,
        "HD1080": sl.RESOLUTION.HD1080,
        "HD720": sl.RESOLUTION.HD720,
        "VGA": sl.RESOLUTION.VGA,
    }[name]


def depth_mode_from_name(name: str) -> sl.DEPTH_MODE:
    return {
        "NEURAL": sl.DEPTH_MODE.NEURAL,
        "ULTRA": sl.DEPTH_MODE.ULTRA,
        "QUALITY": sl.DEPTH_MODE.QUALITY,
        "PERFORMANCE": sl.DEPTH_MODE.PERFORMANCE,
    }[name]


def unit_from_name(name: str) -> sl.UNIT:
    return {
        "METER": sl.UNIT.METER,
        "CENTIMETER": sl.UNIT.CENTIMETER,
        "MILLIMETER": sl.UNIT.MILLIMETER,
    }[name]


def bgr_from_sl_mat(mat: sl.Mat) -> np.ndarray:
    frame = mat.get_data()
    if frame is None or frame.size == 0:
        return np.zeros((1, 1, 3), dtype=np.uint8)
    if frame.shape[-1] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def save_point_cloud(point_cloud: sl.Mat, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    ply_path = out_dir / f"zed_point_cloud_{timestamp}.ply"

    status = point_cloud.write(str(ply_path))
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Failed to save point cloud: {repr(status)}")

    return ply_path


def render_pointcloud_preview(
    points_xyz: np.ndarray, colors_bgr: np.ndarray, width: int = 960, height: int = 540, max_points: int = 70_000
) -> np.ndarray:
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    if points_xyz.size == 0:
        return canvas

    n = len(points_xyz)
    if n > max_points:
        idx = np.random.choice(n, size=max_points, replace=False)
        pts = points_xyz[idx]
        cols = colors_bgr[idx]
    else:
        pts = points_xyz
        cols = colors_bgr

    center = np.median(pts, axis=0)
    pts = pts - center

    yaw = np.deg2rad(-20.0)
    pitch = np.deg2rad(12.0)
    ry = np.array(
        [[np.cos(yaw), 0.0, np.sin(yaw)], [0.0, 1.0, 0.0], [-np.sin(yaw), 0.0, np.cos(yaw)]], dtype=np.float32
    )
    rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, np.cos(pitch), -np.sin(pitch)], [0.0, np.sin(pitch), np.cos(pitch)]],
        dtype=np.float32,
    )
    pts = pts @ ry.T
    pts = pts @ rx.T

    z = pts[:, 2]
    z_shift = np.percentile(z, 5)
    pts[:, 2] = z - z_shift + 1.0
    z = pts[:, 2]

    positive = z > 1e-3
    if not np.any(positive):
        return canvas

    pts = pts[positive]
    cols = cols[positive]
    z = pts[:, 2]

    focal = 450.0
    u = (focal * (pts[:, 0] / z) + (width / 2.0)).astype(np.int32)
    v = (focal * (-pts[:, 1] / z) + (height / 2.0)).astype(np.int32)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    if not np.any(inside):
        return canvas

    u = u[inside]
    v = v[inside]
    cols = cols[inside]
    canvas[v, u] = cols
    canvas = cv2.dilate(canvas, np.ones((2, 2), dtype=np.uint8), iterations=1)
    return canvas


class MJPEGStreamer:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._server: server.ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def update(self, frame_bgr: np.ndarray) -> None:
        ok, encoded = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        with self._lock:
            self._jpeg = encoded.tobytes()

    def get_frame(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def start(self) -> None:
        parent = self

        class Handler(server.BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                if self.path in ("/", "/index.html"):
                    body = (
                        "<html><body style='margin:0;background:#111;color:#ddd;'>"
                        "<h3 style='font-family:sans-serif;padding:8px;'>ZED Feed Stream</h3>"
                        "<img src='/stream.mjpg' style='max-width:100%;height:auto;'/>"
                        "</body></html>"
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path != "/stream.mjpg":
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                try:
                    while True:
                        frame = parent.get_frame()
                        if frame is None:
                            time.sleep(0.03)
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(frame)))
                        self.end_headers()
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.03)
                except (BrokenPipeError, ConnectionResetError):
                    return

            def log_message(self, format: str, *args: object) -> None:
                return

        self._server = server.ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        self._thread = None


def main() -> int:
    args = parse_args()

    init_params = sl.InitParameters()
    init_params.camera_resolution = resolution_from_name(args.resolution)
    init_params.camera_fps = args.fps
    init_params.depth_mode = depth_mode_from_name(args.depth_mode)
    init_params.coordinate_units = unit_from_name(args.unit)

    zed = sl.Camera()
    open_status = zed.open(init_params)
    if open_status != sl.ERROR_CODE.SUCCESS:
        print(f"Unable to open ZED camera: {repr(open_status)}")
        return 1

    runtime = sl.RuntimeParameters()
    image = sl.Mat()
    depth_view = sl.Mat()
    point_cloud = sl.Mat()

    out_dir = Path(args.out_dir)
    streamer: MJPEGStreamer | None = None
    if args.stream_port > 0:
        streamer = MJPEGStreamer(args.stream_host, args.stream_port)
        streamer.start()
        print(f"MJPEG stream: http://<jetson-ip>:{args.stream_port}/")

    print("ZED stream started.")
    print("Controls: q or ESC = quit, p = save point cloud (.ply)")
    if args.headless:
        print("Headless mode enabled: GUI window disabled.")

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_image(depth_view, sl.VIEW.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            left_bgr = bgr_from_sl_mat(image)
            depth_bgr = bgr_from_sl_mat(depth_view)
            pc_data = point_cloud.get_data()

            if left_bgr.shape[:2] != depth_bgr.shape[:2]:
                depth_bgr = cv2.resize(
                    depth_bgr, (left_bgr.shape[1], left_bgr.shape[0]), interpolation=cv2.INTER_LINEAR
                )

            display = np.hstack([left_bgr, depth_bgr])

            if pc_data is not None and pc_data.ndim == 3 and pc_data.shape[2] >= 3:
                xyz = pc_data[:, :, :3]
                valid = np.isfinite(xyz[:, :, 0]) & np.isfinite(xyz[:, :, 1]) & np.isfinite(xyz[:, :, 2])
                points_xyz = xyz[valid]
                colors_bgr = left_bgr[valid]
            else:
                points_xyz = np.empty((0, 3), dtype=np.float32)
                colors_bgr = np.empty((0, 3), dtype=np.uint8)

            cloud_preview = render_pointcloud_preview(points_xyz, colors_bgr)
            cv2.putText(
                cloud_preview,
                "Point Cloud Preview",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (220, 220, 220),
                2,
                cv2.LINE_AA,
            )

            stream_cloud = cv2.resize(cloud_preview, (display.shape[1], max(1, display.shape[0] // 2)))
            stream_frame = np.vstack([display, stream_cloud])
            if streamer is not None:
                streamer.update(stream_frame)

            if not args.headless:
                cv2.imshow("ZED Feed (Left | Depth)", display)
                cv2.imshow("Point Cloud", cloud_preview)

            key = cv2.waitKey(1) & 0xFF if not args.headless else -1
            if key in (27, ord("q")):
                break
            if key == ord("p"):
                try:
                    ply_path = save_point_cloud(point_cloud, out_dir)
                    print(f"Saved point cloud: {ply_path}")
                except RuntimeError as err:
                    print(err)

    finally:
        if streamer is not None:
            streamer.stop()
        cv2.destroyAllWindows()
        zed.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
