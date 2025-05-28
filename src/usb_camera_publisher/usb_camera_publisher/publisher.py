#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

from foxglove_msgs.msg import CompressedVideo
from sensor_msgs.msg import CameraInfo

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib

import threading

class X264MultiCameraPublisher(Node):
    def __init__(self, camera_ids, calib_dir):
        super().__init__("x264_multi_camera_publisher")
        self.get_logger().info("Initializing Multi-Camera Publisher with manual calibration…")

        self.camera_ids = camera_ids
        self.calib_dir  = calib_dir

        # publishers
        self.cam_publishers  = {}
        self.info_publishers = {}

        # per‐camera calibration + crop
        # expect each YAML to optionally contain a "crop" section:
        #   crop:
        #     left:   <px>
        #     right:  <px>
        #     top:    <px>
        #     bottom: <px>
        self.calib_data  = {}
        self.crop_params = {}

        for cam_id in camera_ids:
            # set up publishers
            self.cam_publishers[cam_id]  = self.create_publisher(
                CompressedVideo, f"camera_{cam_id}/h264", 10)
            self.info_publishers[cam_id] = self.create_publisher(
                CameraInfo,     f"camera_{cam_id}/camera_info", 10)

            # load YAML
            yaml_path = os.path.join(calib_dir, f"camera{cam_id}_calib.yaml")
            if not os.path.isfile(yaml_path):
                self.get_logger().error(f"Calibration file not found: {yaml_path}")
                continue

            with open(yaml_path, 'r') as f:
                calib = yaml.safe_load(f)

            # verify required keys
            for key in ('image_width','image_height',
                        'camera_matrix','distortion_coefficients',
                        'rectification_matrix','projection_matrix'):
                if key not in calib:
                    self.get_logger().error(f"{key} missing in {yaml_path}")
                    break

            # default no crop
            crop = calib.get('crop', {})
            self.crop_params[cam_id] = {
                'left':   crop.get('left',   0),
                'right':  crop.get('right',  0),
                'top':    crop.get('top',    0),
                'bottom': crop.get('bottom', 0)
            }

            self.calib_data[cam_id] = calib

        # initialize GStreamer
        Gst.init(None)
        self.camera_pipelines  = {}
        self.camera_mainloops  = {}
        self.camera_threads    = {}

        for cam_id in camera_ids:
            if cam_id in self.calib_data:
                self._start_camera_pipeline(cam_id)

        self.get_logger().info("All camera pipelines initialized.")

    def _start_camera_pipeline(self, cam_id):
        c = self.crop_params[cam_id]
        appsink_name = f"appsink_cam{cam_id}"

        # build videocrop element args
        crop_args = []
        for side in ('left','right','top','bottom'):
            val = c[side]
            if val:
                crop_args.append(f"{side}={val}")
        videocrop_str = "videocrop " + " ".join(crop_args) if crop_args else ""

        pipeline_str = (
            f"v4l2src device=/dev/video{cam_id} ! "
            "image/jpeg, width=640, height=480 ! "
            "jpegdec ! videoconvert ! "
            f"{videocrop_str} ! "
            "videoconvert ! "
            "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! "
            "h264parse config-interval=1 ! "
            "capsfilter caps=\"video/x-h264,stream-format=(string)byte-stream\" ! "
            f"appsink name={appsink_name} emit-signals=true sync=false max-buffers=1 drop=true"
        )

        self.get_logger().info(f"[cam{cam_id}] {pipeline_str}")
        pipe = Gst.parse_launch(pipeline_str)
        if not pipe:
            self.get_logger().error(f"[cam{cam_id}] parse_launch failed")
            return

        appsink = pipe.get_by_name(appsink_name)
        appsink.connect("new-sample", self._on_new_sample, cam_id)
        pipe.set_state(Gst.State.PLAYING)

        loop = GLib.MainLoop()
        self.camera_pipelines[cam_id] = pipe
        self.camera_mainloops[cam_id] = loop

        t = threading.Thread(target=loop.run, daemon=True)
        self.camera_threads[cam_id] = t
        t.start()

    def _on_new_sample(self, appsink, cam_id):
        sample = appsink.emit("pull-sample")
        buf    = sample.get_buffer()
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        # publish the (cropped) H.264 frame
        now = self.get_clock().now().to_msg()
        vid = CompressedVideo()
        vid.timestamp.sec     = now.sec
        vid.timestamp.nanosec = now.nanosec
        vid.format            = "h264"
        vid.frame_id          = f"camera_{cam_id}"
        vid.data              = mi.data
        self.cam_publishers[cam_id].publish(vid)
        buf.unmap(mi)

        # publish adjusted CameraInfo
        calib = self.calib_data[cam_id]
        crop  = self.crop_params[cam_id]

        info = CameraInfo()
        info.header.stamp    = now
        info.header.frame_id = f"camera_{cam_id}"

        # adjust width/height
        orig_w = calib['image_width']
        orig_h = calib['image_height']
        info.width  = orig_w - crop['left'] - crop['right']
        info.height = orig_h - crop['top'] - crop['bottom']

        # copy distortion & matrices
        info.distortion_model = calib.get('distortion_model','plumb_bob')
        info.d = calib['distortion_coefficients']['data']
        info.k = calib['camera_matrix']['data']
        info.r = calib['rectification_matrix']['data']
        info.p = calib['projection_matrix']['data']

        # explicitly declare no binning…
        info.binning_x = 0
        info.binning_y = 0

        # …and no ROI so Foxglove sees a “full‐frame” model
        info.roi.x_offset   = 0
        info.roi.y_offset   = 0
        info.roi.width      = 0
        info.roi.height     = 0
        info.roi.do_rectify = False
        
        self.info_publishers[cam_id].publish(info)
        return Gst.FlowReturn.OK

    def stop_all(self):
        self.get_logger().info("Stopping pipelines…")
        for p in self.camera_pipelines.values():
            p.set_state(Gst.State.NULL)
        for l in self.camera_mainloops.values():
            l.quit()
        for t in self.camera_threads.values():
            if t.is_alive():
                t.join()
        self.get_logger().info("Stopped.")

def main(args=None):
    rclpy.init(args=args)
    camera_ids = [0,2,4,6,8,10]
    calib_dir  = "/home/cmr/cmr/terra/calib_dir"
    node = X264MultiCameraPublisher(camera_ids, calib_dir)
    try:
        rclpy.spin(node)
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
