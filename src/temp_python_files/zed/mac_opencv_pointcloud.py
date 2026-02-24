#!/usr/bin/env python3
"""macOS OpenCV stereo fallback point cloud viewer.

ZED UVC stereo fallback: generate approximate point cloud without ZED SDK.

This script reads a side-by-side stereo stream from a UVC device, computes
stereo disparity with OpenCV, reprojects to 3D, and can save an ASCII PLY.

Controls:
  q / ESC  Quit
  p        Save current point cloud as PLY
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Approximate depth point cloud from ZED UVC stereo feed (no ZED SDK)."
    )
    parser.add_argument("--device", type=int, default=0, help="UVC camera index (default: 0).")
    parser.add_argument("--width", type=int, default=2560, help="Capture width (default: 2560).")
    parser.add_argument("--height", type=int, default=720, help="Capture height (default: 720).")
    parser.add_argument("--fps", type=int, default=30, help="Capture FPS (default: 30).")
    parser.add_argument(
        "--baseline-m",
        type=float,
        default=0.12,
        help="Stereo baseline in meters for Z scaling (default: 0.12).",
    )
    parser.add_argument(
        "--fx",
        type=float,
        default=700.0,
        help="Approximate focal length in pixels (default: 700).",
    )
    parser.add_argument(
        "--min-z",
        type=float,
        default=0.2,
        help="Minimum depth kept in cloud, meters (default: 0.2).",
    )
    parser.add_argument(
        "--max-z",
        type=float,
        default=20.0,
        help="Maximum depth kept in cloud, meters (default: 20.0).",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=200_000,
        help="Maximum points saved per snapshot (default: 200000).",
    )
    parser.add_argument(
        "--out-dir",
        default="captures_uvc",
        help="Output directory for PLY snapshots (default: captures_uvc).",
    )
    parser.add_argument(
        "--snapshot-interval",
        type=float,
        default=0.0,
        help="Auto-save point cloud every N seconds (0 disables, default: 0).",
    )
    parser.add_argument(
        "--lr-thresh",
        type=float,
        default=2.0,
        help="Left-right consistency threshold in disparity pixels (default: 2.0).",
    )
    parser.add_argument(
        "--temporal-alpha",
        type=float,
        default=0.7,
        help="Temporal smoothing weight for current frame in [0,1] (default: 0.7).",
    )
    return parser.parse_args()


def build_q_matrix(width: int, height: int, fx: float, baseline_m: float) -> np.ndarray:
    cx = width / 2.0
    cy = height / 2.0
    q = np.float32(
        [
            [1, 0, 0, -cx],
            [0, 1, 0, -cy],
            [0, 0, 0, fx],
            [0, 0, -1.0 / baseline_m, 0],
        ]
    )
    return q


def split_stereo(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    h, w = frame.shape[:2]
    if w % 2 != 0:
        frame = frame[:, : w - 1]
        h, w = frame.shape[:2]
    mid = w // 2
    left = frame[:, :mid]
    right = frame[:, mid:]
    return left, right


def make_stereo_matcher() -> cv2.StereoSGBM:
    num_disparities = 16 * 8
    block_size = 5
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disparities,
        blockSize=block_size,
        P1=8 * 3 * block_size * block_size,
        P2=32 * 3 * block_size * block_size,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    return matcher


def left_right_consistency(disp_primary: np.ndarray, disp_secondary: np.ndarray, thresh: float) -> np.ndarray:
    h, w = disp_primary.shape
    xs = np.broadcast_to(np.arange(w, dtype=np.float32), (h, w))
    disp_safe = np.where(np.isfinite(disp_primary), disp_primary, 0.0)
    xr = np.rint(xs - disp_safe).astype(np.int32)
    valid = np.isfinite(disp_primary) & (disp_primary > 0.0)
    inside = valid & (xr >= 0) & (xr < w)

    sec_sample = np.full_like(disp_primary, np.nan, dtype=np.float32)
    ys, xs_idx = np.where(inside)
    sec_sample[ys, xs_idx] = disp_secondary[ys, xr[ys, xs_idx]]

    consistent = inside & np.isfinite(sec_sample) & (sec_sample > 0.0) & (np.abs(disp_primary - sec_sample) <= thresh)
    return consistent


def compute_best_disparity(
    matcher: cv2.StereoSGBM, gray_l: np.ndarray, gray_r: np.ndarray, lr_thresh: float
) -> tuple[np.ndarray, np.ndarray, bool]:
    # Evaluate both stereo orders and keep the one with stronger L-R consistency.
    d_lr = matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
    d_rl = matcher.compute(gray_r, gray_l).astype(np.float32) / 16.0
    c_lr = left_right_consistency(d_lr, d_rl, lr_thresh)
    score_lr = int(np.count_nonzero(c_lr))

    d_lr_swapped = matcher.compute(gray_r, gray_l).astype(np.float32) / 16.0
    d_rl_swapped = matcher.compute(gray_l, gray_r).astype(np.float32) / 16.0
    c_swapped = left_right_consistency(d_lr_swapped, d_rl_swapped, lr_thresh)
    score_swapped = int(np.count_nonzero(c_swapped))

    if score_swapped > score_lr:
        disparity = np.where(c_swapped, d_lr_swapped, np.nan)
        return disparity, c_swapped, True

    disparity = np.where(c_lr, d_lr, np.nan)
    return disparity, c_lr, False


def smooth_disparity(disparity: np.ndarray, consistency_mask: np.ndarray) -> np.ndarray:
    disp_fill = np.nan_to_num(disparity, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
    disp_med = cv2.medianBlur(disp_fill, 5)
    disp_bilat = cv2.bilateralFilter(disp_med, d=7, sigmaColor=1.5, sigmaSpace=7)
    smoothed = np.where(consistency_mask, disp_bilat, np.nan)
    return smoothed


def fuse_left_right_color(left: np.ndarray, right: np.ndarray, disparity: np.ndarray) -> np.ndarray:
    h, w = disparity.shape
    fused = left.copy()
    xs = np.broadcast_to(np.arange(w, dtype=np.float32), (h, w))
    xr = np.rint(xs - disparity).astype(np.int32)
    valid = np.isfinite(disparity) & (disparity > 0.0) & (xr >= 0) & (xr < w)
    ys, xs_idx = np.where(valid)
    right_samples = right[ys, xr[ys, xs_idx]].astype(np.float32)
    left_samples = left[ys, xs_idx].astype(np.float32)
    fused_vals = ((left_samples + right_samples) * 0.5).astype(np.uint8)
    fused[ys, xs_idx] = fused_vals
    return fused


def write_ply(path: Path, vertices: np.ndarray, colors: np.ndarray) -> None:
    with path.open("w", encoding="ascii") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(vertices)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for (x, y, z), (r, g, b) in zip(vertices, colors):
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")


def downsample_points(vertices: np.ndarray, colors: np.ndarray, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    n = len(vertices)
    if n <= max_points:
        return vertices, colors
    idx = np.random.choice(n, size=max_points, replace=False)
    return vertices[idx], colors[idx]


def stylize_depth_bw(depth_gray: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    # Build a high-contrast black/white depth map with strong edge contours.
    depth_eq = cv2.equalizeHist(depth_gray)
    bw = cv2.threshold(depth_eq, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    bw = cv2.morphologyEx(
        bw, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=1
    )
    edges = cv2.Canny(depth_eq, 60, 150)
    stylized = bw.copy()
    stylized[edges > 0] = 0
    stylized[~valid_mask] = 0
    return stylized


def render_pointcloud_preview(
    vertices: np.ndarray, colors_rgb: np.ndarray, width: int = 960, height: int = 540, max_points: int = 60_000
) -> np.ndarray:
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    n = len(vertices)
    if n == 0:
        return canvas

    if n > max_points:
        idx = np.random.choice(n, size=max_points, replace=False)
        pts = vertices[idx]
        cols = colors_rgb[idx]
    else:
        pts = vertices
        cols = colors_rgb

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

    canvas[v, u] = cols[:, ::-1]
    canvas = cv2.dilate(canvas, np.ones((2, 2), dtype=np.uint8), iterations=1)
    return canvas


def main() -> int:
    args = parse_args()
    args.temporal_alpha = float(np.clip(args.temporal_alpha, 0.0, 1.0))

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    if not cap.isOpened():
        print(f"Unable to open camera index {args.device}")
        return 1

    matcher = make_stereo_matcher()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    last_stats_ts = 0.0
    last_snapshot_ts = time.time()
    prev_disparity: np.ndarray | None = None

    print("UVC stereo stream started.")
    print("Controls: q or ESC = quit, p = save point cloud (.ply)")
    if args.snapshot_interval > 0:
        print(f"Auto snapshot interval: {args.snapshot_interval:.1f}s")

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            left, right = split_stereo(frame)
            gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

            disparity, consistency_mask, used_swapped = compute_best_disparity(matcher, gray_l, gray_r, args.lr_thresh)
            disparity = smooth_disparity(disparity, consistency_mask)
            if prev_disparity is not None and prev_disparity.shape == disparity.shape:
                curr_ok = np.isfinite(disparity)
                prev_ok = np.isfinite(prev_disparity)
                both = curr_ok & prev_ok
                disparity[both] = args.temporal_alpha * disparity[both] + (1.0 - args.temporal_alpha) * prev_disparity[both]
                # Keep recent depth where current frame is weak/noisy.
                carry = ~curr_ok & prev_ok
                disparity[carry] = prev_disparity[carry]
            prev_disparity = disparity.copy()

            h, w = gray_l.shape
            q = build_q_matrix(w, h, args.fx, args.baseline_m)
            points_3d = cv2.reprojectImageTo3D(disparity, q)

            disparity_valid = np.isfinite(disparity)
            z = points_3d[:, :, 2]
            z_valid = disparity_valid & np.isfinite(z)

            flipped_depth_sign = False
            if np.any(z_valid):
                # UVC stereo feeds can yield an inverted Z sign depending on stereo ordering/convention.
                z_median = float(np.median(z[z_valid]))
                if z_median < 0.0:
                    points_3d *= -1.0
                    z = points_3d[:, :, 2]
                    z_valid = disparity_valid & np.isfinite(z)
                    flipped_depth_sign = True

            valid = z_valid & (z >= args.min_z) & (z <= args.max_z)

            # Grayscale depth view: closer = brighter, farther = darker.
            depth_vis = np.zeros_like(z, dtype=np.uint8)
            if np.any(z_valid):
                # Dynamic normalization ensures values are visible even when absolute range is off.
                z_vis = z[z_valid]
                z_min = float(np.percentile(z_vis, 5))
                z_max = float(np.percentile(z_vis, 95))
                if z_max - z_min < 1e-6:
                    z_max = z_min + 1e-6
                depth_norm = (z - z_min) / (z_max - z_min)
                depth_norm = np.clip(depth_norm, 0.0, 1.0)
                depth_norm = np.nan_to_num(depth_norm, nan=1.0, posinf=1.0, neginf=1.0)
                depth_gray = ((1.0 - depth_norm) * 255.0).astype(np.uint8)
                depth_vis[z_valid] = depth_gray[z_valid]
            else:
                # Fallback: show disparity as grayscale so collected values are still visible.
                disp_vis = np.nan_to_num(disparity, nan=0.0, posinf=0.0, neginf=0.0)
                disp_norm = cv2.normalize(disp_vis, None, 0, 255, cv2.NORM_MINMAX)
                depth_vis = disp_norm.astype(np.uint8)

            valid_count = int(np.count_nonzero(valid))
            z_valid_count = int(np.count_nonzero(z_valid))
            total_px = z.size
            now = time.time()
            if now - last_stats_ts >= 1.0:
                if z_valid_count > 0:
                    z_vals = z[z_valid]
                    zmin = float(np.min(z_vals))
                    zmax = float(np.max(z_vals))
                    print(
                        f"stats: valid_z={z_valid_count}/{total_px} in_range={valid_count} "
                        f"z=[{zmin:.2f},{zmax:.2f}]m swapped={used_swapped} sign_fix={flipped_depth_sign}"
                    )
                else:
                    print(
                        f"stats: valid_z=0/{total_px} in_range=0 "
                        f"swapped={used_swapped} sign_fix={flipped_depth_sign}"
                    )
                last_stats_ts = now

            if args.snapshot_interval > 0 and now - last_snapshot_ts >= args.snapshot_interval and valid_count > 0:
                verts = points_3d[valid]
                color_base = fuse_left_right_color(left, right, disparity)
                cols = color_base[valid][:, ::-1]
                verts, cols = downsample_points(verts, cols, args.max_points)
                ts = time.strftime("%Y%m%d_%H%M%S")
                out_path = out_dir / f"uvc_point_cloud_{ts}.ply"
                write_ply(out_path, verts, cols)
                print(f"Auto-saved {len(verts)} points to {out_path}")
                last_snapshot_ts = now

            depth_stylized = stylize_depth_bw(depth_vis, z_valid)
            disp_color = cv2.cvtColor(depth_stylized, cv2.COLOR_GRAY2BGR)

            display = np.hstack([left, right, disp_color])
            cv2.putText(
                display,
                f"in-range points: {valid_count}",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                display,
                "Depth Feed (B/W)",
                (10, display.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("ZED UVC (Left | Right | Depth Gray)", display)

            color_base = fuse_left_right_color(left, right, disparity)
            verts_preview = points_3d[valid]
            cols_preview = color_base[valid][:, ::-1]
            cloud_preview = render_pointcloud_preview(verts_preview, cols_preview)
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
            cv2.imshow("Point Cloud", cloud_preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("p"):
                verts = points_3d[valid]
                cols = color_base[valid][:, ::-1]
                if len(verts) == 0:
                    print("No valid points to save from current frame.")
                    continue

                verts, cols = downsample_points(verts, cols, args.max_points)

                ts = time.strftime("%Y%m%d_%H%M%S")
                out_path = out_dir / f"uvc_point_cloud_{ts}.ply"
                write_ply(out_path, verts, cols)
                print(f"Saved {len(verts)} points to {out_path}")

    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
