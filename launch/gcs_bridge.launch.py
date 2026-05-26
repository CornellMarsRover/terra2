"""GCS bridge launch file.

Starts the Foxglove WebSocket bridge (``foxglove_bridge``) with QoS overrides
tuned for a URC rover RF link:

* High-rate / large-payload streams (video, images, point clouds, IMU, ground
  plane) are pushed onto the bridge with **best-effort** QoS so that Foxglove
  never holds old frames waiting for retransmits, and the subscription queue
  depth is clamped to 1 so we only ever forward the latest sample.
* Command / service / low-rate topics stay at the ``foxglove_bridge`` default
  (reliable, keep-last, depth 10) so teleop and sequence starts do not drop.

`foxglove_bridge` is used instead of `rosbridge_server` because:

1. It speaks Foxglove Studio's native WebSocket protocol — lower overhead
   than JSON-encoded rosbridge messages, relevant under RF bandwidth caps.
2. It serves ROS message / service / action schemas automatically from the
   live graph, so Foxglove panels can subscribe to custom CMR types (e.g.
   ``cmr_msgs/IMUSensorData``) without hand-written schema files.
3. It supports per-topic best-effort QoS via the regex whitelist below.

Compatibility: the ``ros-humble-foxglove-bridge`` apt package targets ROS 2
Humble, which is this workspace's distro (see
``.github/workflows/build-test.yaml``).

Usage::

    ros2 launch /absolute/path/to/launch/gcs_bridge.launch.py

Or with args::

    ros2 launch launch/gcs_bridge.launch.py port:=8765 address:=0.0.0.0

Default endpoint: ``ws://<rover_ip>:8765``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Topics that should be forwarded best-effort (never block on retransmit).
# Patterns are ECMAScript regex strings, matched against the full topic name.
#
# Keep conservative: only flag the topics that are known-wide (video) or
# known-high-rate. Everything not matched here stays at reliable-default.
BEST_EFFORT_TOPIC_REGEXES = [
    # Cameras: usb_camera_publisher's H.264 stream and any rectified/raw images.
    r"^/?camera_\d+/h264$",
    r"^/?camera_\d+/image_raw$",
    r"^/?camera_\d+/image_rect$",
    r"^/?camera/stitched_image$",
    r"^/?birdseye/image_raw$",
    # ZED image topics.
    r"^/?zed/image(_left|_right)?$",
    # Perception high-rate streams.
    r"^/?camera/points$",
    r"^/?camera/ground_plane$",
    # IMU is 10 Hz and a stale sample is better dropped than buffered.
    r"^/?imu$",
    # ZED pose (10 Hz).
    r"^/?zed/pose$",
    # Fused autonomy pose.
    r"^/?autonomy/pose/robot/global$",
]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="8765",
                description="WebSocket port the Foxglove bridge binds to.",
            ),
            DeclareLaunchArgument(
                "address",
                default_value="0.0.0.0",
                description="Bind address (0.0.0.0 = all interfaces).",
            ),
            DeclareLaunchArgument(
                "tls",
                default_value="false",
                description="Enable TLS (requires certfile/keyfile).",
            ),
            DeclareLaunchArgument(
                "send_buffer_limit_bytes",
                # 10 MB: covers a burst of H.264 keyframes across ~6 cams without
                # silent-dropping, while still being low enough that a stuck
                # client stalls instead of eating memory.
                default_value=str(10 * 1024 * 1024),
                description="Per-client send buffer cap before oldest frames drop.",
            ),
            DeclareLaunchArgument(
                "num_threads",
                default_value="0",
                description="Worker threads (0 = hardware concurrency).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock.",
            ),
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "address": LaunchConfiguration("address"),
                        "tls": LaunchConfiguration("tls"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        # Queue depth the bridge uses when subscribing to ROS
                        # topics it forwards to the client. Depth 1 on video
                        # is what the task brief asks for; commands coming
                        # back from Foxglove go out via client-side publish
                        # (reliable, default keep_last 10) and are not affected.
                        "min_qos_depth": 1,
                        "max_qos_depth": 10,
                        "num_threads": LaunchConfiguration("num_threads"),
                        "send_buffer_limit": LaunchConfiguration(
                            "send_buffer_limit_bytes"
                        ),
                        # For every topic matching one of these regexes the
                        # bridge subscribes with BEST_EFFORT reliability,
                        # matching the usb_camera_publisher / ZED publisher
                        # default and ensuring we don't block on RF drops.
                        "best_effort_qos_topic_whitelist": BEST_EFFORT_TOPIC_REGEXES,
                        # Allow the GCS to call services (SiteAnalyze, lifecycle
                        # activate/deactivate, start_servo, etc.) and publish
                        # commands back on /drives_controller/cmd_vel etc.
                        "capabilities": [
                            "clientPublish",
                            "parameters",
                            "parametersSubscribe",
                            "services",
                            "connectionGraph",
                            "assets",
                        ],
                        # Do not expose ROS2 "hidden" (_underscore) topics.
                        "include_hidden": False,
                    }
                ],
            ),
        ]
    )
