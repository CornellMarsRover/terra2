import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from cmr_msgs.msg import IMUSensorData
from pyubx2 import llh2ecef  # assumes this returns (x, y, z) in meters

class RTKLocalization(Node):
    def __init__(self):
        super().__init__('rtk_localization_node')

        # IMU heading-correction parameters.
        #
        # The autonomy controller works in an N/W frame where bearings come
        # from atan2(west_err, north_err) (CCW-positive looking down:
        # 0=N, +pi/2=W, -pi/2=E). The HWT905-RS232 we currently fly outputs
        # anglez such that, on this rover (with its current mounting), the
        # raw value is already in the same CCW-positive sense -- so flipping
        # the sign with invert_imu_yaw=True produces a CW-positive published
        # yaw and the rover sees E/W swapped (and rotation reversed). Default
        # is therefore False; flip back to True only if a future IMU swap or
        # remount produces an empirical CW-positive raw stream. Static
        # confirmation: face physical W, expect published_yaw ~ +78.5 deg
        # (with declination=-11.5); face physical E, expect ~ -101.5 deg.
        self.declare_parameter('yaw_offset_degrees', 0.0)
        self.declare_parameter('magnetic_declination_degrees', -11.5)
        self.declare_parameter('invert_imu_yaw', False)
        self.declare_parameter('use_gyro_fusion', False)
        self.declare_parameter('gyro_fusion_alpha', 0.98)
        self.declare_parameter('diagnostic_logging', True)
        self.declare_parameter('imu_topic', '/imu')

        self.yaw_offset_degrees = self.get_parameter('yaw_offset_degrees').get_parameter_value().double_value
        self.magnetic_declination_degrees = self.get_parameter('magnetic_declination_degrees').get_parameter_value().double_value
        self.invert_imu_yaw = self.get_parameter('invert_imu_yaw').get_parameter_value().bool_value
        self.use_gyro_fusion = self.get_parameter('use_gyro_fusion').get_parameter_value().bool_value
        self.gyro_fusion_alpha = self.get_parameter('gyro_fusion_alpha').get_parameter_value().double_value
        self.diagnostic_logging = self.get_parameter('diagnostic_logging').get_parameter_value().bool_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # Subscribers for GPS and IMU data
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/rtk/navsatfix_data',
            self.gps_callback,
            10
        )
        self.sub_imu = self.create_subscription(
            IMUSensorData,
            self.imu_topic,
            self.imu_callback,
            10
        )
        # Publisher for the filtered pose estimate
        self.pub = self.create_publisher(TwistStamped, '/autonomy/pose/robot/global', 10)
        
        # Timer for 10Hz updates
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # For GPS offset initialization (using ECEF conversion)
        self.initial_x = None
        self.initial_y = None
        self.initial_lat = None
        self.initial_lon = None
        # Storage for latest sensor measurements
        self.gps_n = 0.0  # north displacement from GPS (meters)
        self.gps_w = 0.0  # west displacement from GPS (meters)
        self.gps_noise = 1.0  # horizontal std. deviation from GPS

        # Heading state. raw_yaw_degrees is the unmodified IMU output (handy for
        # log diagnostics); mag_yaw is the absolute magnetic-frame yaw expressed
        # in the controller's N/W convention; yaw is the value we publish.
        self.yaw = 0.0
        self.mag_yaw = 0.0
        self.raw_yaw_degrees = 0.0
        self.last_imu_time = None
        self.have_first_imu = False

        self.get_logger().info(
            f"RTK localization initialized with imu_topic={self.imu_topic} "
            f"yaw_offset_degrees={self.yaw_offset_degrees:+.2f} "
            f"magnetic_declination_degrees={self.magnetic_declination_degrees:+.2f} "
            f"invert_imu_yaw={self.invert_imu_yaw} "
            f"use_gyro_fusion={self.use_gyro_fusion} "
            f"gyro_fusion_alpha={self.gyro_fusion_alpha:.3f}"
        )

    def gps_callback(self, msg: NavSatFix):
        """
        Process GPS data. Note:
          - The altitude field is used as the horizontal standard deviation (in m)
          - llh2ecef converts (lat, lon, height) to ECEF x,y,z (in meters)
        """

        lat = msg.latitude
        lon = msg.longitude

        if self.initial_lat is None:
            self.initial_lat = lat
            self.initial_lon = lon
            self.get_logger().info(
                f"GPS origin captured lat={self.initial_lat:.8f} lon={self.initial_lon:.8f}"
            )
            return
        
        n, w = self.get_north_west_meters(lat, lon)
        self.gps_n = n
        self.gps_w = w

    def imu_callback(self, msg: IMUSensorData):
        """
        Process IMU data.

        IMUSensorData fields used here:
          - anglez: yaw in degrees, magnetic-north reference, NEU/NED CW-positive
          - gyroz : angular velocity around z in rad/s, body frame

        Pipeline:
          1. Flip sign if invert_imu_yaw is True (NED CW -> NWU CCW).
          2. Add magnetic_declination_degrees (true north correction).
          3. Subtract yaw_offset_degrees (rover-body mounting offset).
          4. Optionally complementary-filter with integrated gyroz.
        """
        self.raw_yaw_degrees = msg.anglez

        sign = -1.0 if self.invert_imu_yaw else 1.0
        mag_yaw_degrees = (
            sign * msg.anglez
            + self.magnetic_declination_degrees
            - self.yaw_offset_degrees
        )
        self.mag_yaw = math.radians(self._wrap_180_degrees(mag_yaw_degrees))

        now = self.get_clock().now()

        if not self.use_gyro_fusion or not self.have_first_imu:
            # Either fusion is disabled, or this is the first sample so we have
            # nothing to integrate from yet -- just snap to the absolute heading.
            self.yaw = self.mag_yaw
        else:
            dt_seconds = (now - self.last_imu_time).nanoseconds * 1e-9
            if dt_seconds <= 0.0 or dt_seconds > 0.5:
                # Bad dt (clock jump, paused, etc) -> fall back to the absolute
                # heading rather than integrating a giant step.
                self.yaw = self.mag_yaw
            else:
                gyro_step = sign * msg.gyroz * dt_seconds
                integrated = self._wrap_pi(self.yaw + gyro_step)
                # Complementary filter: trust gyro short-term, slowly correct
                # toward the magnetic-frame absolute heading.
                self.yaw = self._blend_angles(
                    integrated,
                    self.mag_yaw,
                    self.gyro_fusion_alpha,
                )

        self.last_imu_time = now
        self.have_first_imu = True

    def timer_callback(self):
        """
        Publish pose data
        """
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = self.gps_n
        twist_msg.twist.linear.y = self.gps_w
        twist_msg.twist.angular.z = self.yaw
        self.pub.publish(twist_msg)
        if self.diagnostic_logging:
            self.get_logger().info(
                "rtk_pose "
                f"north={self.gps_n:+.2f}m west={self.gps_w:+.2f}m "
                f"raw_imu_yaw={self.raw_yaw_degrees:+.1f}deg "
                f"mag_yaw={math.degrees(self.mag_yaw):+.1f}deg "
                f"published_yaw={math.degrees(self.yaw):+.1f}deg "
                f"declination={self.magnetic_declination_degrees:+.1f}deg "
                f"offset={self.yaw_offset_degrees:+.1f}deg "
                f"fusion={self.use_gyro_fusion}",
                throttle_duration_sec=1.0,
            )

    #--------------------------------------------------------------------------
    # Helpers
    #--------------------------------------------------------------------------
    @staticmethod
    def _wrap_180_degrees(angle_degrees: float) -> float:
        """Wrap an angle in degrees into [-180, 180)."""
        return (angle_degrees + 180.0) % 360.0 - 180.0

    @staticmethod
    def _wrap_pi(angle_radians: float) -> float:
        """Wrap an angle in radians into [-pi, pi)."""
        return math.atan2(math.sin(angle_radians), math.cos(angle_radians))

    @classmethod
    def _blend_angles(cls, a_radians: float, b_radians: float, alpha: float) -> float:
        """Circular blend of two angles: alpha*a + (1-alpha)*b, wrap-safe."""
        # Convert through unit-vector sum so we don't get bogus averages around
        # the +/- pi wrap point (e.g. averaging 179 and -179 should give 180).
        sin_blend = alpha * math.sin(a_radians) + (1.0 - alpha) * math.sin(b_radians)
        cos_blend = alpha * math.cos(a_radians) + (1.0 - alpha) * math.cos(b_radians)
        return math.atan2(sin_blend, cos_blend)

    def get_north_west_meters(self, lat, lon):
        """
        Converts GPS coordinates (latitude, longitude) to north/west displacements (meters)
        relative to the initial GPS coordinate.
        """
        R = 6378137.0  # Earth radius in meters
        lat1_rad = math.radians(lat)
        lat0_rad = math.radians(self.initial_lat)
        lon1_rad = math.radians(lon)
        lon0_rad = math.radians(self.initial_lon)

        d_lat  = lat0_rad - lat1_rad
        d_lon  = lon0_rad - lon1_rad
        mean_lat = (lat1_rad + lat0_rad) / 2.0

        north = -1.0 * d_lat * R
        west  = d_lon * R * math.cos(mean_lat)
        return north, west

def main(args=None):
    rclpy.init(args=args)
    node = RTKLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
