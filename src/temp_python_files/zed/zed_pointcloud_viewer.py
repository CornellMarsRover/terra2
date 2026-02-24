#!/usr/bin/env python3
"""Live ZED camera feed + depth + point cloud capture.

Controls:
  q / ESC  Quit
  p        Save current point cloud to PLY
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

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

    print("ZED stream started.")
    print("Controls: q or ESC = quit, p = save point cloud (.ply)")

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_image(depth_view, sl.VIEW.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            left_bgr = bgr_from_sl_mat(image)
            depth_bgr = bgr_from_sl_mat(depth_view)

            if left_bgr.shape[:2] != depth_bgr.shape[:2]:
                depth_bgr = cv2.resize(
                    depth_bgr, (left_bgr.shape[1], left_bgr.shape[0]), interpolation=cv2.INTER_LINEAR
                )

            display = np.hstack([left_bgr, depth_bgr])
            cv2.imshow("ZED Feed (Left | Depth)", display)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("p"):
                try:
                    ply_path = save_point_cloud(point_cloud, out_dir)
                    print(f"Saved point cloud: {ply_path}")
                except RuntimeError as err:
                    print(err)

    finally:
        cv2.destroyAllWindows()
        zed.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
