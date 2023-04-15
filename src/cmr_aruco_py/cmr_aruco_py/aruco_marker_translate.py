import rclpy
import rclpy.node
from rclpy.duration import Duration
from ros2_aruco_interfaces.msg import ArucoMarkers
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from rclpy.qos import qos_profile_sensor_data


class ArucoMarkerTranslate(rclpy.node.Node):
    """
    This node is responsible for translating markers from the /aruco_markers topic
    (configurable via the "aruco_marker_topic" parameter) into a MarkerArray that can
    be displayed in RViz or Foxglove Studio. The marker size can be configured to
    match the actual value by setting the "marker_size" parameter; the default is 0.2m.
    """

    def __init__(self):
        super().__init__("marker_translate")

        self.declare_parameter("aruco_marker_topic", "/aruco_markers")
        self.declare_parameter("marker_size", 0.2)

        self.aruco_marker_topic = (
            self.get_parameter("aruco_marker_topic").get_parameter_value().string_value
        )
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )

        self.create_subscription(
            ArucoMarkers,
            self.aruco_marker_topic,
            self.marker_callback,
            qos_profile_sensor_data,
        )

        self.markers_pub = self.create_publisher(MarkerArray, "~/markers", 10)
        self.get_logger().info("Activated aruco_marker_translate")

    def marker_callback(self, msg: ArucoMarkers):
        res = MarkerArray()

        markers = []
        header = msg.header
        header.frame_id = "markers"
        n = len(msg.marker_ids)
        for i in range(n):
            id = i
            pose = msg.poses[i]

            scale = Vector3()
            scale.x = 1.0 * self.marker_size
            scale.y = 1.0 * self.marker_size
            scale.z = 0.2 * self.marker_size

            color = ColorRGBA()
            color.r = 0.0
            color.g = 1.0
            color.b = 0.0
            color.a = 0.7

            marker = Marker()
            marker.header = header
            marker.id = id
            marker.type = marker.CUBE
            marker.lifetime = Duration(seconds=1).to_msg()
            marker.pose = pose
            marker.scale = scale
            marker.color = color
            markers.append(marker)

        res.markers = markers
        self.markers_pub.publish(res)


def main():
    rclpy.init()
    node = ArucoMarkerTranslate()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
