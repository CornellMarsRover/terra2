from test_utils import *
from cmr_msgs.msg import JoystickReading
from sensor_msgs.msg import JointState
import time


@cmr_node_test([
    make_launch_file("cmr_control", "demo_arm.launch.py"),
    make_fabric_node("cmr_arm", config_path="joystick_direct_control.toml"),
])
def test_basic_direct_control(namespace: str):
    assert activate_fabric_node("joystick_direct_control", namespace)

    joy_msg = JoystickReading()
    joy_msg.axis_id = JoystickReading.HORIZONTAL_AXIS_ID
    joy_msg.control_id = JoystickReading.MAIN_JOYSTICK_ID
    joy_msg.magnitude = 10.0
    print("About to send")
    time.sleep(10)
    print("Sending")
    publish_to_topic(JoystickReading, f"/js_input", joy_msg)
    print("Sent, waiting for reply")
    time.sleep(0.5)
    sub = TopicSubscriber(JointState, "/joint_states")
    msg = sub.wait_for_msg()
    print(f"Got reply {msg}")
    assert not (msg is None)
    assert msg.position[0] > 0
    assert all(abs(x) <= sys.float_info.epsilon for x in msg.position[1:])