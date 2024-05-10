from test_utils import *

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def check_joint_states(sub: TopicSubscriber, joint_name, test_fn, timeout_sec=10.0):
    """
    Calls `test_fn` with the position and velocity of `joint_name` state and returns the a tuple of the
    position and velocity.

    Will keep waiting if `tes_fn` returns false
    """
    start_time = time.time()
    last_pos = None
    last_vel = None
    while time.time() - start_time < timeout_sec:
        sub.clear_msgs()
        msg = sub.wait_for_msg()
        if msg is None:
            continue
        idx = None
        try:
            idx = msg.name.index(joint_name)
        except:
            continue
        last_pos = msg.position[idx]
        last_vel = msg.velocity[idx]
        if test_fn:
            res = test_fn(last_pos, last_vel)
            if res:
                break
        else:
            break

    return (last_pos, last_vel)

@cmr_node_test([
    make_launch_file("cmr_control", "demo_drives.launch.py", 
        launch_arguments={'use_rviz': 'False', 
                          'astro_motor_mode': '"velocity"'}.items())
])
def test_astro_motor_vel(namespace):
    time.sleep(5)
    sub = TopicSubscriber(JointState, "/joint_states")

    out_msg = Float64MultiArray()
    out_msg.data = [2.0]
    publish_to_topic(Float64MultiArray, "/astro_motor_vel_controller/commands", out_msg)
    publish_to_topic(Float64MultiArray, "/astro_motor_vel_controller/commands", out_msg)
    time.sleep(5)

    _, vel = check_joint_states(sub, "test_astro_joint", 
                lambda _, vel: abs(vel - out_msg.data[0]) <= sys.float_info.epsilon)

    assert abs(vel - out_msg.data[0]) <= sys.float_info.epsilon

    out_msg.data = [-0.3]
    publish_to_topic(Float64MultiArray, "/astro_motor_vel_controller/commands", out_msg)
    publish_to_topic(Float64MultiArray, "/astro_motor_vel_controller/commands", out_msg)
    time.sleep(5)

    _, vel = check_joint_states(sub, "test_astro_joint", 
                lambda _, vel: abs(vel - out_msg.data[0]) <= sys.float_info.epsilon)

    assert abs(vel - out_msg.data[0]) <= sys.float_info.epsilon