#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from foxglove_msgs.msg import CompressedVideo

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

import threading

class X264MultiCameraPublisher(Node):
    def __init__(self, camera_ids, calibration_xml_map):
        """
        camera_ids: list of integers corresponding to /dev/videoX devices.
        calibration_xml_map: dict mapping cam_id -> XML calibration string
          e.g. {0: "<?xml version=\"1.0\"?>...<\/opencv_storage>", 1: "..."}
        """
        super().__init__("x264_multi_camera_publisher")
        self.get_logger().info("Initializing Multi-Camera Publisher with x264enc (software H.264)...")

        self.camera_ids = camera_ids
        self.calib_map = calibration_xml_map

        # Prepare publishers: one raw and one undistorted per camera
        self.cam_publishers = {}
        for cam_id in camera_ids:
            raw_topic = f"camera_{cam_id}/h264/raw"
            undist_topic = f"camera_{cam_id}/h264/undistorted"
            self.cam_publishers[(cam_id, "raw")] = self.create_publisher(CompressedVideo, raw_topic, 10)
            self.cam_publishers[(cam_id, "undistorted")] = self.create_publisher(CompressedVideo, undist_topic, 10)

        # Initialize GStreamer once
        Gst.init(None)

        # Launch pipelines
        self.camera_pipelines = {}
        self.camera_mainloops = {}
        self.camera_threads = {}
        for cam_id in camera_ids:
            self._start_camera_pipeline(cam_id)

        self.get_logger().info("All camera pipelines initialized.")

    def _start_camera_pipeline(self, cam_id):
        """
        Build a single pipeline per camera with two branches:
        1) Raw H.264
        2) OpenCV undistorted H.264 via cameraundistort
        """
        xml = self.calib_map.get(cam_id)
        if xml is None:
            self.get_logger().error(f"No calibration XML for camera {cam_id}, skipping undistort branch")
            return

        raw_sink = f"appsink_raw_cam{cam_id}"
        undist_sink = f"appsink_undist_cam{cam_id}"

        pipeline_str = (
            f"v4l2src device=/dev/video{cam_id} ! "
            "image/jpeg, width=640, height=480, framerate=30/1 ! "
            "jpegdec ! videoconvert ! tee name=t "

            # Branch 1: raw
            f"t. ! queue ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! "
            "h264parse config-interval=1 ! "
            "capsfilter caps=\"video/x-h264,stream-format=(string)byte-stream\" ! "
            f"appsink name={raw_sink} emit-signals=true sync=false max-buffers=1 drop=true "

            # Branch 2: undistorted via OpenCV plugin
            f"t. ! queue ! "
            f"cameraundistort settings='{xml}' ! videoconvert ! "
            "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! "
            "h264parse config-interval=1 ! "
            "capsfilter caps=\"video/x-h264,stream-format=(string)byte-stream\" ! "
            f"appsink name={undist_sink} emit-signals=true sync=false max-buffers=1 drop=true"
        )

        self.get_logger().info(f"[Camera {cam_id}] pipeline: {pipeline_str}")
        pipeline = Gst.parse_launch(pipeline_str)
        if not pipeline:
            self.get_logger().error(f"Failed to parse pipeline for camera {cam_id}")
            return

        # Connect appsinks
        raw = pipeline.get_by_name(raw_sink)
        undist = pipeline.get_by_name(undist_sink)
        if not raw or not undist:
            self.get_logger().error(f"Could not get appsink elements for camera {cam_id}")
            return

        raw.connect("new-sample", self._on_new_sample, (cam_id, "raw"))
        undist.connect("new-sample", self._on_new_sample, (cam_id, "undistorted"))

        # Store and start
        self.camera_pipelines[cam_id] = pipeline
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error(f"Failed to PLAY pipeline for camera {cam_id}")
            return

        loop = GLib.MainLoop()
        self.camera_mainloops[cam_id] = loop
        thread = threading.Thread(target=loop.run, daemon=True)
        self.camera_threads[cam_id] = thread
        thread.start()

    def _on_new_sample(self, appsink, user_data):
        """
        Called for each new H.264 sample from either branch.
        user_data is (cam_id, mode) where mode is 'raw' or 'undistorted'.
        """
        cam_id, mode = user_data
        sample = appsink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        success, info = buf.map(Gst.MapFlags.READ)
        if not success:
            self.get_logger().error(f"Camera {cam_id} [{mode}]: buffer.map failed")
            return Gst.FlowReturn.ERROR

        msg = CompressedVideo()
        now = self.get_clock().now().to_msg()
        msg.timestamp.sec = now.sec
        msg.timestamp.nanosec = now.nanosec
        msg.format = "h264"
        msg.data = info.data
        self.cam_publishers[(cam_id, mode)].publish(msg)

        buf.unmap(info)
        return Gst.FlowReturn.OK

    def stop_all(self):
        self.get_logger().info("Stopping all camera pipelines...")
        for p in self.camera_pipelines.values():
            p.set_state(Gst.State.NULL)
        for loop in self.camera_mainloops.values():
            loop.quit()
        for t in self.camera_threads.values():
            if t.is_alive():
                t.join()
        self.get_logger().info("All camera pipelines stopped.")

def main(args=None):
    rclpy.init(args=args)

    # List your camera IDs here
    camera_ids = [0, 2, 4, 6, 8, 10]

    # Replace these with your real OpenCV XML calibration strings
    with open("/home/cmr/cmr/terra/src/usb_camera_publisher/usb_camera_publisher/cam.xml","r") as f:
        CALIBRATION_XML = { 0: f.read() }

    node = X264MultiCameraPublisher(camera_ids, CALIBRATION_XML)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, shutting down...")
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
