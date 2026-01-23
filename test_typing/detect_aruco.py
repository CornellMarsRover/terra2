#!/usr/bin/env python3
"""
AR Tag (ArUco) detection from a USB/USB‑C camera on macOS (or any OS with OpenCV).
- Auto-picks the first working camera, or use --camera-id to select one.
- Tries common ArUco dictionaries (4x4, 5x5, 6x6, 7x7) to detect markers.
- Draws marker borders and shows IDs in real time.
- Press 'q' to quit, 's' to save a frame, 'd' to cycle dictionaries, 'c' to cycle cameras.

Install deps:
    python3 -m pip install -r requirements.txt
Run:
    python3 detect_aruco.py
Optional:
    python3 detect_aruco.py --camera-id 0 --width 1280 --height 720
"""

import argparse
import time
import cv2
from datetime import datetime
import os

# --- Helper: build available aruco dictionaries in OpenCV ---
def get_aruco_dicts():
    # Common, widely printed sets
    names = [
        "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
        "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
        "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
        "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000"
    ]
    d = []
    for n in names:
        if hasattr(cv2.aruco, n):
            d.append((n, cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, n))))
    return d

def list_cameras(max_index=10, width=None, height=None):
    found = []
    for idx in range(max_index):
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)  # AVFoundation is good on macOS; falls back if not available
        if not cap.isOpened():
            cap.release()
            continue
        # Try to set resolution (some cameras reject silently)
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, _ = cap.read()
        if ok:
            found.append(idx)
        cap.release()
    return found

def open_camera(camera_id=None, width=None, height=None):
    if camera_id is not None:
        indices = [camera_id]
    else:
        indices = list_cameras(max_index=10, width=width, height=height)
        if not indices:
            # Try some defaults if enumeration failed
            indices = [0, 1, 2, 3]
    for idx in indices:
        cap = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            cap.release()
            continue
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, idx
        cap.release()
    return None, None

def draw_detected(frame, corners, ids, rejected=None, dict_name=""):
    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # Label
        cv2.putText(frame, f"Detected {len(ids)} marker(s) ({dict_name})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    else:
        cv2.putText(frame, f"No markers ({dict_name})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)
    # Optional: show count of rejected candidates (can be noisy)
    if rejected is not None:
        cv2.putText(frame, f"Rejected: {len(rejected)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

def main():
    parser = argparse.ArgumentParser(description="AR Tag (ArUco) detector")
    parser.add_argument("--camera-id", type=int, default=None, help="Camera index (default: auto-pick the first working)")
    parser.add_argument("--width", type=int, default=1280, help="Requested capture width")
    parser.add_argument("--height", type=int, default=720, help="Requested capture height")
    parser.add_argument("--save-dir", type=str, default="captures", help="Directory to save snapshots")
    args = parser.parse_args()

    dicts = get_aruco_dicts()
    if not dicts:
        raise RuntimeError("Your OpenCV build lacks ArUco. Install opencv-contrib-python.")

    dict_idx = 2  # start at a common one (e.g., DICT_4X4_250 if present)
    if dict_idx >= len(dicts):
        dict_idx = 0

    detector_params = cv2.aruco.DetectorParameters()
    # Slightly relax detection to help beginners
    detector_params.adaptiveThreshWinSizeMin = 3
    detector_params.adaptiveThreshWinSizeMax = 23
    detector_params.adaptiveThreshWinSizeStep = 10

    dictionary_name, dictionary = dicts[dict_idx]
    detector = cv2.aruco.ArucoDetector(dictionary, detector_params)

    cap, current_cam = open_camera(args.camera_id, args.width, args.height)
    if cap is None:
        raise RuntimeError("Could not open any camera. Try a different --camera-id or grant camera permissions to Terminal.")

    print(f"[INFO] Using camera index {current_cam}")
    os.makedirs(args.save_dir, exist_ok=True)

    last_fps_t = time.time()
    frames = 0
    fps = 0.0

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            # Try to reopen current camera once
            cap.release()
            cap, current_cam = open_camera(current_cam, args.width, args.height)
            if cap is None:
                print("[WARN] Camera dropped; attempting to open another camera...")
                cap, current_cam = open_camera(None, args.width, args.height)
                if cap is None:
                    print("[ERROR] No camera available. Exiting.")
                    break
            continue

        corners, ids, rejected = detector.detectMarkers(frame)
        draw_detected(frame, corners, ids, rejected, dict_name=dictionary_name)

        # FPS display
        frames += 1
        now = time.time()
        if now - last_fps_t >= 1.0:
            fps = frames / (now - last_fps_t)
            frames = 0
            last_fps_t = now
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("ArUco Detector", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = f"{args.save_dir}/aruco_{ts}.png"
            cv2.imwrite(path, frame)
            print(f"[INFO] Saved snapshot to {path}")
        elif key == ord('d'):
            # Cycle dictionary
            dict_idx = (dict_idx + 1) % len(dicts)
            dictionary_name, dictionary = dicts[dict_idx]
            detector = cv2.aruco.ArucoDetector(dictionary, detector_params)
            print(f"[INFO] Switched dictionary to {dictionary_name}")
        elif key == ord('c'):
            # Cycle camera
            next_candidates = list(range(current_cam + 1, current_cam + 5))
            cap.release()
            cap = None
            for cid in next_candidates:
                cap_try, idx = open_camera(cid, args.width, args.height)
                if cap_try is not None:
                    cap = cap_try
                    current_cam = idx
                    print(f"[INFO] Switched to camera index {current_cam}")
                    break
            if cap is None:
                # fallback to auto-pick
                cap, current_cam = open_camera(None, args.width, args.height)
                if cap is not None:
                    print(f"[INFO] Switched to camera index {current_cam}")
                else:
                    print("[WARN] Could not switch cameras; sticking with previous.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
