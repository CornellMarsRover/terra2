#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from cmr_msgs.msg import AutonomyDrive


class TestRealtimePlotterPublisher(Node):
    def __init__(self):
        super().__init__('test_realtime_plotter_publisher')

        self.pose_pub = self.create_publisher(
            TwistStamped,
            '/autonomy/pose/robot/global',
            10
        )
        self.target_pub = self.create_publisher(
            Twist,
            '/autonomy/target_object/position',
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.gps_pub = self.create_publisher(
            NavSatFix,
            '/rtk/navsatfix_data',
            10
        )
        self.state_pub = self.create_publisher(
            String,
            '/autonomy/state',
            10
        )
        self.move_type_pub = self.create_publisher(
            String,
            '/autonomy/move/move_type',
            10
        )
        self.ackermann_pub = self.create_publisher(
            AutonomyDrive,
            '/autonomy/move/ackerman',
            10
        )
        self.point_turn_pub = self.create_publisher(
            Twist,
            '/autonomy/move/point_turn',
            10
        )

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('lat_radius', 0.000120)
        self.declare_parameter('lon_radius', 0.000120)
        self.declare_parameter('angular_speed', 0.45)
        self.declare_parameter('center_lat', 42.444180)
        self.declare_parameter('center_lon', -76.483450)
        self.declare_parameter('target_lat', 42.444061)
        self.declare_parameter('target_lon', -76.483483)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('state_text', 'NAVIGATING')

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.lat_radius = float(self.get_parameter('lat_radius').value)
        self.lon_radius = float(self.get_parameter('lon_radius').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.center_lat = float(self.get_parameter('center_lat').value)
        self.center_lon = float(self.get_parameter('center_lon').value)
        self.target_lat = float(self.get_parameter('target_lat').value)
        self.target_lon = float(self.get_parameter('target_lon').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.state_text = self.get_parameter('state_text').value

        self.t = 0.0
        self.dt = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('Test publisher started.')
        self.get_logger().info(
            'Publishing fake TwistStamped pose, GPS, target twist, cmd_vel, autonomy state, and drive topics.'
        )

    def timer_callback(self):
        phase = self.angular_speed * self.t

        lon = self.center_lon + self.lon_radius * math.cos(phase)
        lat = self.center_lat + self.lat_radius * math.sin(phase)

        dlon_dt = -self.lon_radius * self.angular_speed * math.sin(phase)
        dlat_dt = self.lat_radius * self.angular_speed * math.cos(phase)
        yaw = math.atan2(dlat_dt, dlon_dt)

        pseudo_speed = math.sqrt(dlon_dt * dlon_dt + dlat_dt * dlat_dt)
        steer_angle_deg = 18.0 * math.sin(phase)
        point_turn_wz = 0.35 * math.cos(phase)
        move_type = 'ackerman' if math.cos(phase) >= 0.0 else 'point_turn'

        pose_msg = TwistStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.twist.linear.x = lon
        pose_msg.twist.linear.y = lat
        pose_msg.twist.linear.z = 0.0
        pose_msg.twist.angular.x = 0.0
        pose_msg.twist.angular.y = 0.0
        pose_msg.twist.angular.z = yaw
        self.pose_pub.publish(pose_msg)

        gps_msg = NavSatFix()
        gps_msg.header.stamp = pose_msg.header.stamp
        gps_msg.header.frame_id = self.frame_id
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = 488.0
        self.gps_pub.publish(gps_msg)

        target_msg = Twist()
        target_msg.linear.x = self.target_lon
        target_msg.linear.y = self.target_lat
        target_msg.linear.z = 0.0
        self.target_pub.publish(target_msg)

        cmd_msg = Twist()
        cmd_msg.linear.x = dlon_dt
        cmd_msg.linear.y = dlat_dt
        cmd_msg.linear.z = 0.0
        cmd_msg.angular.x = 0.0
        cmd_msg.angular.y = 0.0
        cmd_msg.angular.z = self.angular_speed
        self.cmd_pub.publish(cmd_msg)

        ack_msg = AutonomyDrive()
        ack_msg.vel = pseudo_speed
        ack_msg.fl_angle = steer_angle_deg
        ack_msg.fr_angle = steer_angle_deg
        ack_msg.bl_angle = 0.0
        ack_msg.br_angle = 0.0
        self.ackermann_pub.publish(ack_msg)

        point_turn_msg = Twist()
        point_turn_msg.angular.z = point_turn_wz
        self.point_turn_pub.publish(point_turn_msg)

        state_msg = String()
        state_msg.data = self.state_text
        self.state_pub.publish(state_msg)

        move_type_msg = String()
        move_type_msg.data = move_type
        self.move_type_pub.publish(move_type_msg)

        self.get_logger().info(
            f'Robot -> lon={lon:.6f}, lat={lat:.6f}, yaw={yaw:.2f} rad | '
            f'Target -> lon={self.target_lon:.6f}, lat={self.target_lat:.6f} | '
            f'Move={move_type} | cmd_vel -> vx={dlon_dt:.6f}, vy={dlat_dt:.6f}, wz={self.angular_speed:.2f}'
        )

        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = TestRealtimePlotterPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
