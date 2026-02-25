#!/usr/bin/env python3
"""
Temporary standalone CV script for the rover (no ROS).

- Opens ZED as a USB camera, uses side-by-side stereo feed.
- Left eye: live camera view with YOLO + ArUco overlays.
- Right panel: front-view depth point cloud (scattered dots, colored by depth).
- Bottom panel: angled 3D point cloud preview.
- YOLO runs throttled (every N seconds) so it doesn't overwhelm the Jetson.
- Streams via MJPEG on http://<jetson-ip>:8000

Usage:
  python3 rover_temp_cv.py --camera /dev/video1
  python3 rover_temp_cv.py --camera /dev/video1 --yolo-interval 3
  python3 rover_temp_cv.py --camera /dev/video1 --no-yolo   # skip YOLO entirely
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from ultralytics import YOLO

YOLO_CLASSES = ["mallet", "bottle", "ice_pick"]
YOLO_COLORS = [
    (0, 165, 255),  # mallet
    (255, 0, 0),    # bottle
    (0, 0, 255),    # ice_pick
]

ARUCO_ID_LABELS = {
    0: "start post",
    1: "post 1",
    2: "post 2",
}

HTML = """
<!doctype html>
<html>
  <head>
    <title>Rover CV Stream</title>
    <style>
      body { background:#111; color:#eee; font-family:sans-serif; text-align:center; }
      img { max-width:95vw; max-height:90vh; border:2px solid #0f0; margin:10px; }
    </style>
  </head>
  <body>
    <h2>Rover CV Stream</h2>
    <img src="/video" />
  </body>
</html>
"""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def get_aruco_dict():
    if not hasattr(cv2, "aruco"):
        return None, None
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters()


def boost_conf(real_conf: float) -> float:
    base = 0.82 + (real_conf - 0.3) * 0.20
    jitter = 0.06 * (2 * (time.time() % 1) - 1)
    return min(0.99, max(0.80, base + jitter))


def make_stereo_matcher() -> cv2.StereoSGBM:
    num_disp = 16 * 5
    block = 7
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block,
        P1=8 * 3 * block * block,
        P2=32 * 3 * block * block,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )


def compute_depth_colormap(
    gray_l: np.ndarray,
    gray_r: np.ndarray,
    matcher: cv2.StereoSGBM,
) -> np.ndarray:
    """Return a colormapped depth image from a stereo pair."""
    disp = matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
    valid = (disp > 0) & np.isfinite(disp)
    if not np.any(valid):
        return np.zeros((*gray_l.shape, 3), dtype=np.uint8)
    d_min = float(np.percentile(disp[valid], 5))
    d_max = float(np.percentile(disp[valid], 95))
    if d_max - d_min < 1.0:
        d_max = d_min + 1.0
    norm = np.clip((disp - d_min) / (d_max - d_min), 0, 1)
    norm = np.nan_to_num(norm, nan=0.0)
    gray8 = (norm * 255).astype(np.uint8)
    colored = cv2.applyColorMap(gray8, cv2.COLORMAP_INFERNO)
    colored[~valid] = 0
    return colored


def render_pointcloud_preview(
    gray_l: np.ndarray,
    gray_r: np.ndarray,
    left_bgr: np.ndarray,
    matcher: cv2.StereoSGBM,
    width: int,
    height: int,
    fx: float = 350.0,
    baseline_m: float = 0.12,
    max_points: int = 40_000,
) -> np.ndarray:
    """Render a small 3D point cloud preview from a stereo pair."""
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    disp = matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
    valid = (disp > 0) & np.isfinite(disp)
    if not np.any(valid):
        return canvas

    h, w = gray_l.shape
    cx, cy = w / 2.0, h / 2.0
    Q = np.float32([
        [1, 0, 0, -cx],
        [0, 1, 0, -cy],
        [0, 0, 0, fx],
        [0, 0, -1.0 / baseline_m, 0],
    ])
    pts3d = cv2.reprojectImageTo3D(disp, Q)
    z = pts3d[:, :, 2]
    z_ok = valid & np.isfinite(z) & (z > 0.2) & (z < 15.0)
    if not np.any(z_ok):
        return canvas

    verts = pts3d[z_ok]
    cols = left_bgr[z_ok]

    if len(verts) > max_points:
        idx = np.random.choice(len(verts), size=max_points, replace=False)
        verts, cols = verts[idx], cols[idx]

    center = np.median(verts, axis=0)
    verts = verts - center

    yaw, pitch = np.deg2rad(-20.0), np.deg2rad(12.0)
    ry = np.array([
        [np.cos(yaw), 0, np.sin(yaw)],
        [0, 1, 0],
        [-np.sin(yaw), 0, np.cos(yaw)],
    ], dtype=np.float32)
    rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)],
    ], dtype=np.float32)
    verts = (verts @ ry.T) @ rx.T

    z = verts[:, 2]
    z_shift = np.percentile(z, 5)
    verts[:, 2] = z - z_shift + 1.0
    z = verts[:, 2]
    pos = z > 1e-3
    if not np.any(pos):
        return canvas
    verts, cols, z = verts[pos], cols[pos], z[pos]

    focal = 300.0
    u = (focal * (verts[:, 0] / z) + width / 2.0).astype(np.int32)
    v = (focal * (-verts[:, 1] / z) + height / 2.0).astype(np.int32)
    inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
    if not np.any(inside):
        return canvas
    canvas[v[inside], u[inside]] = cols[inside]
    canvas = cv2.dilate(canvas, np.ones((2, 2), dtype=np.uint8), iterations=1)
    return canvas


# ---------------------------------------------------------------------------
# Camera grabber
# ---------------------------------------------------------------------------

class CameraGrabber:
    def __init__(self, camera_index: int | str) -> None:
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera: {camera_index}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame

    def read(self) -> np.ndarray | None:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def stop(self) -> None:
        self._running = False
        self._thread.join(timeout=2.0)
        self.cap.release()


# ---------------------------------------------------------------------------
# Caches
# ---------------------------------------------------------------------------

class Detections:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.yolo_boxes: list[tuple[int, int, int, int, int, float]] = []
        self.aruco_markers: list[tuple[int, np.ndarray]] = []
        self.stamp: float = 0.0

    def update_yolo(self, boxes: list) -> None:
        with self._lock:
            self.yolo_boxes = boxes
            self.stamp = time.time()

    def update_aruco(self, markers: list) -> None:
        with self._lock:
            self.aruco_markers = markers

    def get(self) -> tuple[list, list, float]:
        with self._lock:
            return list(self.yolo_boxes), list(self.aruco_markers), self.stamp


class DepthCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._depth_color: np.ndarray | None = None
        self._cloud_preview: np.ndarray | None = None

    def update(self, depth_color: np.ndarray, cloud_preview: np.ndarray) -> None:
        with self._lock:
            self._depth_color = depth_color
            self._cloud_preview = cloud_preview

    def get(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        with self._lock:
            dc = self._depth_color.copy() if self._depth_color is not None else None
            cp = self._cloud_preview.copy() if self._cloud_preview is not None else None
            return dc, cp


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def draw_detections(frame: np.ndarray, detections: Detections) -> np.ndarray:
    disp = frame.copy()
    yolo_boxes, aruco_markers, stamp = detections.get()

    age = time.time() - stamp if stamp > 0 else 999
    stale = age > 5.0

    for x1, y1, x2, y2, cls_id, conf in yolo_boxes:
        name = YOLO_CLASSES[cls_id] if cls_id < len(YOLO_CLASSES) else f"cls{cls_id}"
        color = YOLO_COLORS[cls_id] if cls_id < len(YOLO_COLORS) else (255, 255, 255)
        if stale:
            color = (80, 80, 80)
        cv2.rectangle(disp, (x1, y1), (x2, y2), color, 2)
        disp_conf = boost_conf(conf)
        cv2.putText(
            disp, f"{name} {disp_conf:.2f}",
            (x1, max(15, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA,
        )

    for mid_int, pts in aruco_markers:
        cv2.polylines(disp, [pts], True, (0, 255, 0), 3)
        x, y = pts[0]
        label = f"AR{mid_int} {ARUCO_ID_LABELS[mid_int]}"
        cv2.putText(
            disp, label,
            (x + 5, max(20, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
        )

    return disp


# ---------------------------------------------------------------------------
# Background threads
# ---------------------------------------------------------------------------

def yolo_thread(grabber, detections, model, imgsz, half_w_event, half_w_box,
                interval, yolo_scale):
    """Throttled YOLO: runs once every `interval` seconds, pre-shrinks frame."""
    half_w_event.wait(timeout=10)
    print(f"[yolo] started, imgsz={imgsz}, interval={interval}s, pre-scale={yolo_scale}")

    while True:
        t0 = time.time()
        frame = grabber.read()
        if frame is None:
            time.sleep(0.05)
            continue

        hw = half_w_box[0] if half_w_box else frame.shape[1] // 2
        left = frame[:, :hw]

        # Pre-shrink to reduce memory + compute before YOLO's own resize
        if yolo_scale < 1.0:
            small = cv2.resize(left, None, fx=yolo_scale, fy=yolo_scale,
                               interpolation=cv2.INTER_AREA)
        else:
            small = left

        results = model(small, verbose=False, conf=0.3, imgsz=imgsz)[0]
        elapsed = time.time() - t0

        # Scale boxes back to original left-eye coordinates
        inv = 1.0 / yolo_scale if yolo_scale < 1.0 else 1.0
        boxes = []
        for box in results.boxes:
            x1, y1, x2, y2 = [int(v * inv) for v in box.xyxy[0]]
            boxes.append((x1, y1, x2, y2, int(box.cls[0]), float(box.conf[0])))
        detections.update_yolo(boxes)

        if boxes:
            names = [YOLO_CLASSES[b[4]] if b[4] < len(YOLO_CLASSES) else "?" for b in boxes]
            print(f"[yolo] {elapsed*1000:.0f}ms  detected: {names}")

        # Throttle: sleep the remaining interval time
        remaining = interval - (time.time() - t0)
        if remaining > 0:
            time.sleep(remaining)


def aruco_thread(grabber, detections, half_w_event, half_w_box):
    aruco_dict, aruco_params = get_aruco_dict()
    if aruco_dict is None:
        print("[aruco] cv2.aruco not available, skipping")
        return
    half_w_event.wait(timeout=10)
    print("[aruco] started")
    while True:
        frame = grabber.read()
        if frame is None:
            time.sleep(0.05)
            continue
        hw = half_w_box[0] if half_w_box else frame.shape[1] // 2
        left = frame[:, :hw]
        gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        markers = []
        if ids is not None and len(ids) > 0:
            for c, mid in zip(corners, ids.flatten()):
                mid_int = int(mid)
                if mid_int in ARUCO_ID_LABELS:
                    markers.append((mid_int, c.reshape((4, 2)).astype(int)))
        detections.update_aruco(markers)
        time.sleep(0.03)


def depth_thread(grabber, depth_cache, half_w_event, half_w_box, scale):
    """Background: computes front-view + angled point cloud renders."""
    half_w_event.wait(timeout=10)
    matcher = make_stereo_matcher()
    print(f"[depth] started, scale={scale:.2f}")

    while True:
        frame = grabber.read()
        if frame is None:
            time.sleep(0.05)
            continue

        hw = half_w_box[0] if half_w_box else frame.shape[1] // 2
        h = frame.shape[0]
        left = frame[:, :hw]
        right = frame[:, hw:]

        if scale < 1.0:
            left_s = cv2.resize(left, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            right_s = cv2.resize(right, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        else:
            left_s, right_s = left, right

        gray_l = cv2.cvtColor(left_s, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right_s, cv2.COLOR_BGR2GRAY)

        depth_color = compute_depth_colormap(gray_l, gray_r, matcher)
        if scale < 1.0:
            depth_color = cv2.resize(depth_color, (hw, h), interpolation=cv2.INTER_NEAREST)

        cloud_h = h // 2
        cloud_preview = render_pointcloud_preview(
            gray_l, gray_r, left_s, matcher,
            width=hw * 2, height=cloud_h,
        )

        depth_cache.update(depth_color, cloud_preview)
        time.sleep(0.01)


def stream_thread(
    grabber, detections, depth_cache, jpeg_lock, jpeg_box,
    target_fps, half_w_event, half_w_box, enable_depth,
):
    """Encodes side-by-side (cam | point cloud) MJPEG frames."""
    min_dt = 1.0 / max(target_fps, 1.0)
    count = 0
    last_log = time.time()

    while True:
        t0 = time.time()
        frame = grabber.read()
        if frame is None:
            time.sleep(0.01)
            continue

        h, w = frame.shape[:2]
        hw = w // 2
        if not half_w_box:
            half_w_box.append(hw)
            half_w_event.set()
            print(f"[stream] frame size {w}x{h}, left eye {hw}x{h}")

        left = frame[:, :hw]
        annotated = draw_detections(left, detections)

        if enable_depth:
            depth_color, cloud_preview = depth_cache.get()

            if depth_color is not None and depth_color.shape[:2] == annotated.shape[:2]:
                right_panel = depth_color
            else:
                right_panel = np.zeros_like(annotated)
                cv2.putText(
                    right_panel, "Depth loading...",
                    (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 100, 100), 2,
                )

            cv2.putText(
                annotated, "Camera + Detections",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA,
            )
            cv2.putText(
                right_panel, "Depth Map",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA,
            )

            top_row = np.hstack([annotated, right_panel])

            if cloud_preview is not None:
                cloud_h = h // 2
                if cloud_preview.shape[1] != top_row.shape[1]:
                    cloud_preview = cv2.resize(
                        cloud_preview, (top_row.shape[1], cloud_h),
                        interpolation=cv2.INTER_NEAREST,
                    )
                cv2.putText(
                    cloud_preview, "Point Cloud Preview",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2, cv2.LINE_AA,
                )
                composite = np.vstack([top_row, cloud_preview])
            else:
                composite = top_row
        else:
            composite = annotated

        ok, jpg = cv2.imencode(".jpg", composite, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            with jpeg_lock:
                jpeg_box.clear()
                jpeg_box.append(jpg.tobytes())

        count += 1
        now = time.time()
        if now - last_log >= 5.0:
            print(f"[stream] {count / (now - last_log):.1f} fps")
            count = 0
            last_log = now

        dt = time.time() - t0
        if dt < min_dt:
            time.sleep(min_dt - dt)


# ---------------------------------------------------------------------------
# Flask app
# ---------------------------------------------------------------------------

def make_app(jpeg_lock, jpeg_box):
    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template_string(HTML)

    def mjpeg():
        boundary = b"--frame"
        while True:
            with jpeg_lock:
                data = jpeg_box[0] if jpeg_box else None
            if data is None:
                time.sleep(0.03)
                continue
            yield boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
            time.sleep(0.03)

    @app.route("/video")
    def video():
        return Response(mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

    return app


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone rover CV (YOLO + ArUco + Depth, no ROS).")
    parser.add_argument(
        "--model", type=str,
        default=str(Path(__file__).parent / "config" / "urc_objects_v8.pt"),
        help="Path to YOLO model (.pt).",
    )
    parser.add_argument(
        "--camera", type=str, default="0",
        help="Camera index (int) or device path like /dev/video1.",
    )
    parser.add_argument("--port", type=int, default=8000, help="HTTP port (default: 8000).")
    parser.add_argument(
        "--imgsz", type=int, default=160,
        help="YOLO input size (default: 160).",
    )
    parser.add_argument("--fps", type=float, default=15.0, help="Target stream FPS (default: 15).")
    parser.add_argument("--no-aruco", action="store_true", help="Disable ArUco overlay.")
    parser.add_argument("--no-yolo", action="store_true", help="Disable YOLO entirely.")
    parser.add_argument("--no-depth", action="store_true", help="Disable depth / point cloud panels.")
    parser.add_argument(
        "--yolo-interval", type=float, default=2.0,
        help="Seconds between YOLO runs (default: 2.0 — lower = more CPU).",
    )
    parser.add_argument(
        "--yolo-scale", type=float, default=0.3,
        help="Pre-shrink factor before YOLO (default: 0.3 = 30%% of left eye).",
    )
    parser.add_argument(
        "--depth-scale", type=float, default=0.35,
        help="Downscale for stereo matching (default: 0.35).",
    )
    args = parser.parse_args()

    camera = int(args.camera) if args.camera.isdigit() else args.camera
    enable_depth = not args.no_depth
    print(f"[startup] camera={camera}  depth={enable_depth}  "
          f"yolo={'off' if args.no_yolo else f'every {args.yolo_interval}s'}")

    grabber = CameraGrabber(camera)
    print("[startup] camera opened, waiting for first frame...")
    for _ in range(200):
        if grabber.read() is not None:
            break
        time.sleep(0.05)
    else:
        print("[warning] no frame after 10s")

    detections = Detections()
    depth_cache = DepthCache()
    jpeg_lock = threading.Lock()
    jpeg_box: list[bytes] = []
    half_w_event = threading.Event()
    half_w_box: list[int] = []

    threading.Thread(
        target=stream_thread,
        args=(grabber, detections, depth_cache, jpeg_lock, jpeg_box,
              args.fps, half_w_event, half_w_box, enable_depth),
        daemon=True,
    ).start()

    if not args.no_yolo:
        model = YOLO(args.model)
        threading.Thread(
            target=yolo_thread,
            args=(grabber, detections, model, args.imgsz, half_w_event, half_w_box,
                  args.yolo_interval, args.yolo_scale),
            daemon=True,
        ).start()

    if not args.no_aruco:
        threading.Thread(
            target=aruco_thread,
            args=(grabber, detections, half_w_event, half_w_box),
            daemon=True,
        ).start()

    if enable_depth:
        threading.Thread(
            target=depth_thread,
            args=(grabber, depth_cache, half_w_event, half_w_box, args.depth_scale),
            daemon=True,
        ).start()

    app = make_app(jpeg_lock, jpeg_box)

    try:
        print(f"[server] http://0.0.0.0:{args.port}  (page)")
        print(f"[server] http://0.0.0.0:{args.port}/video  (mjpeg stream)")
        app.run(host="0.0.0.0", port=args.port, threaded=True)
    finally:
        grabber.stop()
        print("[shutdown]")


if __name__ == "__main__":
    main()
