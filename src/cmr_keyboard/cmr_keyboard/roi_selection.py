#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ROISelectionNode(Node):
    def __init__(self):
        super().__init__('roi_selection')
        self.subscription = self.create_subscription(
            Image,
            '/zed/image_left',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_image = None

        # ROI variables
        self.roi = None          # Last selected ROI: (x, y, w, h)
        self.temp_roi = None     # ROI currently being drawn
        self.selecting = False   # True when waiting for a new ROI selection
        self.dragging = False    # True while the user is dragging the mouse
        self.roi_start = None    # Starting point of the ROI when dragging begins

        # Set up the display window and mouse callback.
        cv2.namedWindow("ROI Selection")
        cv2.setMouseCallback("ROI Selection", self.mouse_callback)

        # Timer callback to update the display.
        self.timer = self.create_timer(0.03, self.timer_callback)

    def image_callback(self, msg):
        """Receive an Image message and convert it to an OpenCV image."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        self.latest_image = cv_image

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events for ROI selection."""
        # Only process mouse events when in selection mode.
        if not self.selecting:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.roi_start = (x, y)
            self.temp_roi = (x, y, 0, 0)

        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            # Update the temporary ROI while dragging.
            x0, y0 = self.roi_start
            self.temp_roi = (x0, y0, x - x0, y - y0)

        elif event == cv2.EVENT_LBUTTONUP and self.dragging:
            self.dragging = False
            # Finalize the ROI and normalize it to have positive width and height.
            x0, y0, w, h = self.temp_roi
            if w < 0:
                x0 += w
                w = -w
            if h < 0:
                y0 += h
                h = -h
            self.roi = (x0, y0, w, h)
            self.temp_roi = None
            self.selecting = False
            self.get_logger().info(f"ROI selected: {self.roi}")

    def timer_callback(self):
        """Update the display with the latest image and ROI overlay, and handle key events."""
        if self.latest_image is not None:
            display_image = self.latest_image.copy()

            # Draw the finalized ROI in green if available.
            if self.roi is not None:
                x, y, w, h = self.roi
                cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # While selecting, if the user is dragging, draw the temporary ROI in blue.
            if self.selecting and self.dragging and self.temp_roi is not None:
                x, y, w, h = self.temp_roi
                cv2.rectangle(display_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

            cv2.imshow("ROI Selection", display_image)

            # Check for key events.
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                # Only allow a new ROI selection if not already selecting.
                if not self.selecting:
                    self.selecting = True
                    self.temp_roi = None
                    self.roi_start = None
                    self.get_logger().info("ROI selection enabled: click and drag to select a region.")
            elif key == ord('q'):
                self.get_logger().info("Quitting node.")
                rclpy.shutdown()
                cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ROISelectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
