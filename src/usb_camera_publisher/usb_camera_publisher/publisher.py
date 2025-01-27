#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

import threading

class X264MultiCameraPublisher(Node):
    def __init__(self, camera_ids):
        """
        camera_ids: list of integers corresponding to /dev/videoX devices.
                    e.g. [0, 1] for /dev/video0 and /dev/video1
        """
        super().__init__("x264_multi_camera_publisher")
        self.get_logger().info("Initializing Multi-Camera Publisher with x264enc (software H.264)...")

        self.camera_ids = camera_ids
        self.camera_pipelines = {}
        self.camera_mainloops = {}
        self.camera_threads = {}

        # Create a separate publisher for each camera
        self.cam_publishers = {}
        for cam_id in camera_ids:
            topic_name = f"camera_{cam_id}/h264"
            pub = self.create_publisher(CompressedVideo, topic_name, 10)
            self.cam_publishers[cam_id] = pub

        # Initialize GStreamer
        Gst.init(None)

        # Start each camera pipeline in its own thread/main loop
        for cam_id in camera_ids:
            self._start_camera_pipeline(cam_id)

        self.get_logger().info("All camera pipelines initialized.")

    def _start_camera_pipeline(self, cam_id):
        """
        Create a GStreamer pipeline that captures from /dev/video{cam_id},
        encodes with x264enc (CPU-based), and outputs H.264 data to appsink.
        We'll connect a 'new-sample' callback to publish frames over ROS.
        """
        appsink_name = f"appsink_cam{cam_id}"

        # Example pipeline:
        # v4l2src device=/dev/video{cam_id} !
        #   video/x-raw, width=640, height=480, framerate=30/1 !
        #   videoconvert !
        #   x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 !
        #   h264parse !
        #   appsink name={appsink_name} emit-signals=true sync=false max-buffers=1 drop=true
        gst_pipeline_str = (
            f"v4l2src device=/dev/video{cam_id} ! "
            "image/jpeg, width=640, height=480, framerate=30/1 ! "
            "jpegdec ! "
            "videoconvert ! "
            "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! "
            # Use h264parse without "stream-format"
            "h264parse config-interval=1 ! "
            # Force Annex B via capsfilter
            "capsfilter caps=\"video/x-h264,stream-format=(string)byte-stream\" ! "
            f"appsink name=appsink_cam{cam_id} emit-signals=true sync=false max-buffers=1 drop=true"
        )

        self.get_logger().info(f"Launching pipeline for camera {cam_id}: {gst_pipeline_str}")
        pipeline = Gst.parse_launch(gst_pipeline_str)
        if not pipeline:
            self.get_logger().error(f"Failed to create pipeline for camera {cam_id}")
            return

        appsink = pipeline.get_by_name(appsink_name)
        if not appsink:
            self.get_logger().error(f"Failed to retrieve appsink for camera {cam_id}")
            return

        # Connect "new-sample" callback
        appsink.connect("new-sample", self._on_new_sample, cam_id)

        self.camera_pipelines[cam_id] = pipeline

        # Start the pipeline
        ret = pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error(f"Failed to set pipeline to PLAYING for camera {cam_id}")
            return

        # Each pipeline needs its own GLib MainLoop
        loop = GLib.MainLoop()
        self.camera_mainloops[cam_id] = loop

        thread = threading.Thread(target=loop.run, daemon=True)
        self.camera_threads[cam_id] = thread
        thread.start()

    def _on_new_sample(self, appsink, cam_id):
        """
        Callback whenever the appsink gets a new H.264-encoded frame.
        We'll extract the raw H.264 bytes and publish them via foxglove_msgs/CompressedVideo.
        """
        sample = appsink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR

        buffer = sample.get_buffer()
        if not buffer:
            return Gst.FlowReturn.ERROR

        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            self.get_logger().error(f"Failed to map buffer for camera {cam_id}")
            return Gst.FlowReturn.ERROR

        # These are the raw H.264 bytes
        h264_data = map_info.data

        msg = CompressedVideo()
        msg.timestamp.sec = self.get_clock().now().to_msg().sec
        msg.timestamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg.format = "h264"
        msg.data = h264_data
        self.cam_publishers[cam_id].publish(msg)

        buffer.unmap(map_info)
        return Gst.FlowReturn.OK

    def stop_all(self):
        """
        Gracefully stop all pipelines and threads.
        """
        self.get_logger().info("Stopping all camera pipelines...")
        for _, pipeline in self.camera_pipelines.items():
            pipeline.set_state(Gst.State.NULL)
        for _, loop in self.camera_mainloops.items():
            loop.quit()
        for _, thread in self.camera_threads.items():
            if thread.is_alive():
                thread.join()
        self.get_logger().info("All camera pipelines stopped.")

def main(args=None):
    rclpy.init(args=args)

    # Update this list to match the cameras available on your system
    # e.g., camera_ids = [0, 1] if you have /dev/video0 and /dev/video1
    camera_ids = [0, 2, 4, 6, 8, 10]

    node = X264MultiCameraPublisher(camera_ids)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
