from test_utils import *
import time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.executors import SingleThreadedExecutor
import rclpy

def check_end_effector_to_base_transform(node, buffer: Buffer, test_fn, timeout_sec=10.0):
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
    exc.spin_until_future_complete(buffer.wait_for_transform_async("third_rotate", "base_link", rclpy.time.Time()), 
        timeout_sec=timeout_sec)
    while time.time() - start_time < timeout_sec:
        exc.spin_once(timeout_sec=timeout_sec)
        transform = buffer.lookup_transform("third_rotate", "base_link", rclpy.time.Time())
        last_transform = transform.transform
        if test_fn:
            res = test_fn(transform.transform)
            if res:
                break
        else:
            break

    print("Got transform")
    return last_transform


@cmr_node_test([
    make_launch_file("cmr_arm_config", "demo.launch.py", 
        launch_arguments={'use_rviz': 'False'}.items()),
    make_fabric_node("cmr_arm", config_path="ik_config.toml"),
])
def test_simple_ik_control(namespace):
    time.sleep(5)
    assert activate_fabric_node("ik_config", namespace)
    tf_node = Node('tf_node')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, tf_node)

    eps = 0.01 # 1 cm


    target_msg = PoseStamped()
    target_msg.header.frame_id= 'base'
    target_msg.pose.orientation.w = 1.0
    target_msg.pose.position.x = 0.6685640811920166
    target_msg.pose.position.y = 0.19633984565734863
    target_msg.pose.position.z = 0.5014975070953369


    tr_transform = check_end_effector_to_base_transform(tf_node, tf_buffer, None)
    # Assert initial point is not target point
    assert abs(-tr_transform.translation.x - target_msg.pose.position.x) > eps
    assert abs(-tr_transform.translation.y - target_msg.pose.position.y) > eps
    assert abs(-tr_transform.translation.z - target_msg.pose.position.z) > eps


    publish_to_topic(PoseStamped, "/arm_pose_topic", target_msg)
    time.sleep(8)
    tr_transform = check_end_effector_to_base_transform(tf_node, tf_buffer, 
        lambda transform: abs(transform.translation.x - target_msg.pose.position.x) < eps and 
                          abs(transform.translation.y - target_msg.pose.position.y) < eps and 
                          abs(transform.translation.z - target_msg.pose.position.z) < eps)

    print(f"Got transform: {tr_transform}")
    assert abs(-tr_transform.translation.x - target_msg.pose.position.x) < eps
    assert abs(-tr_transform.translation.y - target_msg.pose.position.y) < eps
    assert abs(-tr_transform.translation.z - target_msg.pose.position.z) < eps

    target_msg.pose.position.x = 0.4860771894454956
    target_msg.pose.position.y = 0.22480428218841553
    target_msg.pose.position.z = 0.720863401889801
    lastz=0.720863401889801
    publish_to_topic(PoseStamped, "/arm_pose_topic", target_msg)
    time.sleep(8)
    tr_transform = check_end_effector_to_base_transform(tf_node, tf_buffer, 
        lambda transform: abs(transform.translation.x - target_msg.pose.position.x) < eps and 
                          abs(transform.translation.y - target_msg.pose.position.y) < eps and 
                          abs(transform.translation.z - target_msg.pose.position.z) < eps)
    assert abs(-tr_transform.translation.x - target_msg.pose.position.x) < eps
    lastx_transform = tr_transform.translation.x
    assert abs(-tr_transform.translation.y - target_msg.pose.position.y) < eps
    lasty_transform = tr_transform.translation.y
    assert abs(-tr_transform.translation.z - target_msg.pose.position.z) < eps

   


    #Test with third_tilt reference frame

    target_msg.header.frame_id= 'third_rotate'
    target_msg.pose.orientation.w = 1.0
    target_msg.pose.position.x = 0.0
    target_msg.pose.position.y = 0.0
    target_msg.pose.position.z = 0.05

    # in the base frame, the arm should be
    # x should be ~0.5251580476760864
    # y should be ~0.2128298282623291
    # z should be ~0.7393946647644043
    target_z = lastz + target_msg.pose.position.z


    publish_to_topic(PoseStamped, "/arm_pose_topic", target_msg)
    time.sleep(8)
    tr_transform = check_end_effector_to_base_transform(tf_node, tf_buffer, 
        lambda transform: abs(transform.translation.x - lastx_transform) < eps and 
                          abs(transform.translation.y - lasty_transform) < eps and 
                          abs(-transform.translation.z - target_z) < eps)

    assert abs(tr_transform.translation.x - lastx_transform) < eps
    assert abs(tr_transform.translation.y - lasty_transform) < eps
    assert abs(-tr_transform.translation.z - target_z) < eps

    
    
    # Test not reachable

    target_msg.pose.position.x = 0.0
    target_msg.pose.position.y = 0.0
    target_msg.pose.position.z = 0.0

    publish_to_topic(PoseStamped, "/arm_pose_topic", target_msg)
    time.sleep(5)
    tr_transform = check_end_effector_to_base_transform(tf_node, tf_buffer, 
        lambda transform: abs(transform.translation.x - target_msg.pose.position.x) < eps and 
                          abs(transform.translation.y - target_msg.pose.position.y) < eps and 
                          abs(transform.translation.z - target_msg.pose.position.z) < eps)

    assert abs(-tr_transform.translation.x - target_msg.pose.position.x) > eps
    assert abs(-tr_transform.translation.y - target_msg.pose.position.y) > eps
    assert abs(-tr_transform.translation.z - target_msg.pose.position.z) > eps