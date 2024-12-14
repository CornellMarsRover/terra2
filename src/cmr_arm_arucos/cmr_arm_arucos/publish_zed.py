import pyzed.sl as sl
import cv2
import numpy as np

'''
************
NOT ACTUAL ZED PUBLISHER NODE
************
use "zed_camera_publisher" in "cmr_zed" package
'''

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create an InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO  # Use automatic resolution
    init_params.camera_fps = 30  # Set fps at 30

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Camera Open Error: {err}. Exiting program.")
        exit()

    # Create Mat objects to hold the images
    image_left = sl.Mat()
    image_right = sl.Mat()

    runtime_parameters = sl.RuntimeParameters()

    print("Press 'q' to quit.")
    while True:
        # Grab an image
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left and right images
            zed.retrieve_image(image_left, sl.VIEW.LEFT)
            zed.retrieve_image(image_right, sl.VIEW.RIGHT)

            # Convert sl.Mat to numpy arrays (RGBA images)
            left_image = image_left.get_data()
            right_image = image_right.get_data()

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Failed to grab images.")
            break

    # Close the camera and windows
    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
