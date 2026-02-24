# aruco_laptop.py
import argparse
import time

import cv2


ID_LABELS = {
    0: "start post",
    1: "post 1",
    2: "post 2",
}


def get_aruco_dict(dict_size: int) -> "cv2.aruco_Dictionary":
    """
    dict_size: one of 50, 100, 250, 1000 for DICT_4X4_*
    """
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco not found. Install opencv-contrib-python:\n"
            "  python3 -m pip install opencv-contrib-python"
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


def draw_marker(frame, corners, marker_id: int, thickness: int = 8):
    # corners is shape (1, 4, 2) typically
    pts = corners.reshape((4, 2)).astype(int)
    color = (0, 255, 0)  # bright green in BGR

    # Draw polygon
    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness)

    # Place text near top-left corner
    x, y = pts[0]
    label = ID_LABELS.get(marker_id, f"detected {{id={marker_id}}}")
    cv2.putText(
        frame,
        label,
        (x + 8, y - 10 if y - 10 > 10 else y + 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        2,
        color,
        2,
        cv2.LINE_AA,
    )


def main():
    parser = argparse.ArgumentParser(description="Laptop ArUco detector (IDs 0/1/2 only).")
    parser.add_argument("--camera", type=int, default=0, help="Camera index (default: 0)")
    parser.add_argument("--dict", type=int, default=50, choices=[50, 100, 250, 1000],
                        help="ArUco dict size for DICT_4X4_* (default: 50)")
    parser.add_argument("--thickness", type=int, default=4, help="Box line thickness (default: 4)")
    parser.add_argument("--show-fps", action="store_true", help="Overlay FPS")
    args = parser.parse_args()

    aruco_dict = get_aruco_dict(args.dict)
    aruco_params = cv2.aruco.DetectorParameters()

    # Open camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {args.camera}")

    last_t = time.time()
    fps = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            # Detect
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                frame, aruco_dict, parameters=aruco_params
            )

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten()
                for c, mid in zip(corners, ids_flat):
                    if int(mid) in ID_LABELS:
                        draw_marker(frame, c, int(mid), thickness=args.thickness)

            # FPS (optional)
            now = time.time()
            dt = now - last_t
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else (1.0 / dt)
            last_t = now
            if args.show_fps:
                cv2.putText(
                    frame,
                    f"FPS: {fps:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow("ArUco (IDs 0/1/2)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()