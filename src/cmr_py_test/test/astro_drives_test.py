from test_utils import *
from cmr_msgs.msg import Float64ArrayStamped
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
from rclpy.executors import SingleThreadedExecutor
import rclpy
import math


def check_base_link_to_odom_transform(node, buffer: Buffer, test_fn, timeout_sec=10.0):
    """
    Calls `test_fn` with the base_link pose in the odom frame and returns the a tuple of the
    result and the last transform.

    Will keep waiting if `tes_fn` returns false
    """

    exc = SingleThreadedExecutor()
    exc.add_node(node)
    start_time = time.time()
    res = False
    last_transform = None
    exc.spin_until_future_complete(buffer.wait_for_transform_async("odom", "base_link", rclpy.time.Time()), 
        timeout_sec=timeout_sec)
    while time.time() - start_time < timeout_sec:
        exc.spin_once(timeout_sec=timeout_sec)
        transform = buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        last_transform = transform.transform
        if test_fn:
            res = test_fn(transform.transform)
            if res:
                break
        else:
            break

    return (res, last_transform)

def euler_from_quaternion(quat):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class TestAstroDrives(CMRTestFixture):
    CMRTestFixture.nodes = [make_launch_file("cmr_control", "demo_drives.launch.py", 
        launch_arguments={'use_rviz': 'False'}.items())]

    def test_astro_sensor(self):
        time.sleep(5)
        sub = TopicSubscriber(Float64ArrayStamped, "/astro_controller/sensor")
        msg = sub.wait_for_msg()
        start_time = time.time()
        while msg is None and time.time() - start_time < 10.0:
            sub.clear_msgs()
            msg = sub.wait_for_msg()
        assert msg is not None
        assert len(msg.data) == 3
        # Assumes using dummy sensor values
        assert abs(msg.data[0] - 10.0) <= sys.float_info.epsilon
        assert abs(msg.data[1] - 20.0) <= sys.float_info.epsilon
        assert abs(msg.data[2] - 30.0) <= sys.float_info.epsilon

    def test_drives_linear(self):
        tf_node = Node('tf_node')
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, tf_node)

        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = 1.0

        publish_to_topic(TwistStamped, "/drives_controller/cmd_vel", msg)
        publish_to_topic(TwistStamped, "/drives_controller/cmd_vel", msg)
        res, last_transform = check_base_link_to_odom_transform(tf_node, tf_buffer, 
            lambda transform: transform.translation.x > sys.float_info.epsilon and 
            abs(transform.translation.y) < sys.float_info.epsilon and 
            abs(transform.translation.z) < sys.float_info.epsilon)
        assert last_transform.translation.x > sys.float_info.epsilon
        assert abs(last_transform.translation.y) < sys.float_info.epsilon
        assert abs(last_transform.translation.z) < sys.float_info.epsilon

        msg.twist.linear.x = -5.0
        publish_to_topic(TwistStamped, "/drives_controller/cmd_vel", msg)
        _, new_transform = check_base_link_to_odom_transform(tf_node, tf_buffer,
            lambda transform: transform.translation.x < last_transform.translation.x and
            abs(transform.translation.y) < sys.float_info.epsilon and 
            abs(transform.translation.z) < sys.float_info.epsilon)
        assert new_transform.translation.x < last_transform.translation.x
        assert abs(last_transform.translation.y) < sys.float_info.epsilon
        assert abs(last_transform.translation.z) < sys.float_info.epsilon

    def test_drives_angular(self):
        tf_node = Node('tf_node')
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, tf_node)

        msg = TwistStamped()
        msg.twist.angular.z = 1.0
        msg.header.frame_id = "base_link"

        def get_rpy_fn(lam):
            return lambda transform: lam(euler_from_quaternion(transform.rotation))

        publish_to_topic(TwistStamped, "/drives_controller/cmd_vel", msg)
        publish_to_topic(TwistStamped, "/drives_controller/cmd_vel", msg)
        res, last_transform = check_base_link_to_odom_transform(tf_node, tf_buffer, 
            get_rpy_fn(lambda rpy: rpy[2] > sys.float_info.epsilon and
                abs(rpy[0]) < sys.float_info.epsilon and
                abs(rpy[1]) < sys.float_info.epsilon))
        last_rpy = euler_from_quaternion(last_transform.rotation)
        assert last_rpy[2] > sys.float_info.epsilon
        assert abs(last_rpy[0]) < sys.float_info.epsilon
        assert abs(last_rpy[1]) < sys.float_info.epsilon

        msg.twist.angular.z = -5.0
        publish_to_topic(TwistStamped, "/drives_controller/cmd_vel", msg)
        _, new_transform = check_base_link_to_odom_transform(tf_node, tf_buffer,
            get_rpy_fn(lambda rpy: rpy[2] < last_rpy[2] and
                abs(rpy[0]) < sys.float_info.epsilon and
                abs(rpy[1]) < sys.float_info.epsilon))
        new_rpy = euler_from_quaternion(new_transform.rotation)
        assert new_rpy[2] < last_rpy[2]
        assert abs(last_rpy[0]) < sys.float_info.epsilon
        assert abs(last_rpy[1]) < sys.float_info.epsilon

        


