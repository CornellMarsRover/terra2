"""
Local webcam test for the aruco detection + pose estimation pipeline.
Exercises the same logic as aruco_detection_node.py without requiring ROS.

Press 'q' to quit.
"""
import cv2
import numpy as np
import math
import time

camera_matrix = np.array([[657.70933821, 0.0, 605.53598505],
                           [0.0, 657.27652417, 343.9524918],
                           [0.0, 0.0, 1.0]])
dist_coeffs = np.array([0.34688736, 0.07662388, 0.14965771, 0.01600403])

marker_length = 0.1
half = marker_length / 2.0
marker_obj_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0],
], dtype=np.float32)

EXPECTED_IDS = {0, 1, 2, 3}

def compute_roll_pitch_yaw_from_normal(normal):
    normal = normal / np.linalg.norm(normal)
    nx, ny, nz = normal
    pitch = math.degrees(np.arcsin(np.clip(-nx, -1.0, 1.0)))
    yaw = math.degrees(np.arctan2(ny, nz))
    roll = 0.0
    return roll, pitch, yaw

def detect_and_annotate(image):
    if image is None or image.size == 0:
        return image, {}

    if image.shape[-1] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = detector.detectMarkers(gray)

    detections = {}
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_corners = corners[i][0]
            cx = int(np.mean(marker_corners[:, 0]))
            cy = int(np.mean(marker_corners[:, 1]))
            detections[marker_id] = (cx, cy)

            success, rvec, tvec = cv2.solvePnP(
                marker_obj_points, corners[i][0],
                camera_matrix, dist_coeffs)
            if success:
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                rot_matrix, _ = cv2.Rodrigues(rvec)
                normal = rot_matrix[:, 2]
                roll, pitch, yaw = compute_roll_pitch_yaw_from_normal(normal)

                label = f"ID:{marker_id} Y:{yaw:.1f} P:{pitch:.1f}"
                cv2.putText(image, label, (cx - 40, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        found = set(int(i) for i in ids.flatten())
        missing = EXPECTED_IDS - found
        if missing:
            status = f"Missing markers: {sorted(missing)}"
            color = (0, 0, 255)
        else:
            status = "All 4 markers detected!"
            color = (0, 255, 0)
    else:
        status = "No markers detected"
        color = (0, 0, 255)

    cv2.putText(image, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    return image, detections

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Could not open webcam index 0. Trying index 1...")
        cap = cv2.VideoCapture(1)
        if not cap.isOpened():
            print("ERROR: No webcam found.")
            return

    # macOS AVFoundation needs time; first few reads often fail
    print("Waiting for camera to warm up...")
    for _ in range(30):
        ret, _ = cap.read()
        if ret:
            break
        time.sleep(0.2)

    print("Webcam opened. Showing aruco detection. Press 'q' to quit.")
    print(f"Looking for DICT_6X6_50 markers with IDs: {sorted(EXPECTED_IDS)}")
    print(f"Camera matrix (hardcoded for ZED -- angles may differ on laptop cam):")
    print(f"  fx={camera_matrix[0,0]:.1f}  fy={camera_matrix[1,1]:.1f}")
    print(f"  cx={camera_matrix[0,2]:.1f}  cy={camera_matrix[1,2]:.1f}")
    print()

    consecutive_failures = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            consecutive_failures += 1
            if consecutive_failures > 30:
                print("Too many consecutive frame failures. Exiting.")
                break
            time.sleep(0.1)
            continue
        consecutive_failures = 0

        annotated, detections = detect_and_annotate(frame)

        if detections:
            for mid, (cx, cy) in sorted(detections.items()):
                print(f"  Marker {mid}: center=({cx}, {cy})", end="")
            print()

        cv2.imshow("Aruco Detection Test", annotated)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Done.")

if __name__ == '__main__':
    main()
