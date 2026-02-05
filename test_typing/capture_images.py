import cv2
import os

# Folder to save calibration images
save_dir = "calib_images"
os.makedirs(save_dir, exist_ok=True)

# Determine next image index
existing = [f for f in os.listdir(save_dir) if f.startswith("calib_") and f.endswith(".png")]
if existing:
    nums = [int(f.split("_")[1].split(".")[0]) for f in existing]
    next_idx = max(nums) + 1
else:
    next_idx = 0

cap = cv2.VideoCapture(1)

print("Press SPACE to save an image, ESC to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Capture Calibration Images", frame)
    key = cv2.waitKey(1)

    if key == 27:  # ESC
        break

    if key == 32:  # SPACE
        filename = os.path.join(save_dir, f"calib_{next_idx}.png")
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")
        next_idx += 1

cap.release()
cv2.destroyAllWindows()
