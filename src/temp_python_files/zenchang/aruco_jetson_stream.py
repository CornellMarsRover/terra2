# aruco_jetson_stream.py
import argparse
import os
import signal
import subprocess
import threading
import time
from datetime import datetime

import cv2
from flask import Flask, Response, render_template_string


ID_LABELS = {
    0: "detected {id=0 is start post}",
    1: "detected {id=1 is post 1}",
    2: "detected {id=2 is post 2}",
}


HTML = """
<!doctype html>
<html>
  <head><title>Jetson ArUco Stream</title></head>
  <body style="font-family: sans-serif;">
    <h2>Jetson ArUco Stream (IDs 0/1/2)</h2>
    <p>Video:</p>
    <img src="/video" />
  </body>
</html>
"""


def get_aruco_dict(dict_size: int) -> "cv2.aruco_Dictionary":
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco not found. On Jetson, install OpenCV with aruco or use:\n"
            "  python3 -m pip install opencv-contrib-python\n"
            "(If pip wheels are painful on Jetson, consider JetPack/OpenCV packages.)"
        )

    aruco = cv2.aruco
    mapping = {
        50: aruco.DICT_4X4_50,
        100: aruco.DICT_4X4_100,
        250: aruco.DICT_4X4_250,
        1000: aruco.DICT_4X4_1000,
    }
    if dict_size not in mapping:
        raise ValueError(f"dict_size must be one of {list(mapping.keys())}, got {dict_size}")
    return aruco.getPredefinedDictionary(mapping[dict_size])


def draw_marker(frame, corners, marker_id: int, thickness: int = 4):
    pts = corners.reshape((4, 2)).astype(int)
    color = (0, 255, 0)

    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness)

    x, y = pts[0]
    label = ID_LABELS.get(marker_id, f"detected {{id={marker_id}}}")
    cv2.putText(
        frame,
        label,
        (x + 8, y - 10 if y - 10 > 10 else y + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )


class CameraWorker:
    def __init__(self, camera_index: int, dict_size: int, thickness: int,
                 record_path: str | None, fps: float):
        self.camera_index = camera_index
        self.thickness = thickness
        self.record_path = record_path
        self.target_fps = fps

        self.aruco_dict = get_aruco_dict(dict_size)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {camera_index}")

        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.running = True

        self.writer = None
        if record_path:
            # We'll initialize writer once we know frame size.
            self.record_path = record_path

    def stop(self):
        self.running = False

    def _init_writer_if_needed(self, frame):
        if self.record_path and self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.record_path, fourcc, self.target_fps, (w, h))
            if not self.writer.isOpened():
                raise RuntimeError(f"Failed to open VideoWriter at {self.record_path}")

    def loop(self):
        # Simple pacing to not max CPU
        min_dt = 1.0 / max(self.target_fps, 1.0)

        while self.running:
            t0 = time.time()
            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            # Detect + draw
            corners, ids, _rej = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten()
                for c, mid in zip(corners, ids_flat):
                    mid_int = int(mid)
                    if mid_int in ID_LABELS:
                        draw_marker(frame, c, mid_int, thickness=self.thickness)

            # Record if enabled
            if self.record_path:
                self._init_writer_if_needed(frame)
                self.writer.write(frame)

            # Encode JPEG for MJPEG stream
            ok2, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok2:
                with self.lock:
                    self.latest_jpeg = jpg.tobytes()

            dt = time.time() - t0
            if dt < min_dt:
                time.sleep(min_dt - dt)

        # Cleanup
        self.cap.release()
        if self.writer is not None:
            self.writer.release()

    def get_latest_jpeg(self):
        with self.lock:
            return self.latest_jpeg


def make_app(worker: CameraWorker):
    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template_string(HTML)

    def mjpeg_generator():
        boundary = b"--frame"
        while worker.running:
            frame = worker.get_latest_jpeg()
            if frame is None:
                time.sleep(0.01)
                continue
            yield boundary + b"\r\n" + b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"

    @app.route("/video")
    def video():
        return Response(
            mjpeg_generator(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    return app


def scp_to_host(local_path: str, host_target: str):
    """
    host_target example: user@192.168.1.50:/Users/me/Videos/
    """
    print(f"[scp] Sending {local_path} -> {host_target}")
    subprocess.run(["scp", "-p", local_path, host_target], check=False)


def main():
    parser = argparse.ArgumentParser(description="Jetson headless ArUco stream (MJPEG) + optional recording.")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--dict", type=int, default=50, choices=[50, 100, 250, 1000],
                        help="ArUco dict size for DICT_4X4_* (default: 50)")
    parser.add_argument("--thickness", type=int, default=4, help="Box line thickness (default: 4)")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="Bind port (default: 8000)")
    parser.add_argument("--fps", type=float, default=30.0, help="Capture/record FPS (default: 30)")
    parser.add_argument("--record", action="store_true", help="Enable recording to mp4")
    parser.add_argument("--out", type=str, default="", help="Output mp4 path. If empty, auto-name in ./videos/")
    parser.add_argument("--scp", type=str, default="",
                        help="If set, scp video to this target on exit. Example: user@HOST:/path/")
    args = parser.parse_args()

    record_path = None
    if args.record:
        if args.out.strip():
            record_path = args.out.strip()
            os.makedirs(os.path.dirname(record_path) or ".", exist_ok=True)
        else:
            os.makedirs("videos", exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            record_path = os.path.join("videos", f"aruco_{ts}.mp4")

    worker = CameraWorker(
        camera_index=args.camera,
        dict_size=args.dict,
        thickness=args.thickness,
        record_path=record_path,
        fps=args.fps,
    )

    app = make_app(worker)

    # Start capture thread
    t = threading.Thread(target=worker.loop, daemon=True)
    t.start()

    # Graceful shutdown on Ctrl+C
    def handle_sigint(_sig, _frame):
        print("\n[shutdown] Stopping...")
        worker.stop()

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    try:
        print(f"[server] http://{args.host}:{args.port}  (page)")
        print(f"[server] http://{args.host}:{args.port}/video  (mjpeg stream)")
        app.run(host=args.host, port=args.port, threaded=True)
    finally:
        worker.stop()
        t.join(timeout=2.0)

        # Auto-scp if requested and we recorded
        if record_path and args.scp.strip():
            scp_to_host(record_path, args.scp.strip())
        print("[done]")


if __name__ == "__main__":
    main()