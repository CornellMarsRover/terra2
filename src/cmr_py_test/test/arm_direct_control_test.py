from test_utils import *
from cmr_msgs.msg import JoystickReading
from sensor_msgs.msg import JointState
import time

# NOTE: Due to timing, this test can be a tad flaky. It should pass most of the time
# TODO(sev47): Look into this

BASE_ROTATE_IDX = 0
SHOULDER_IDX = 1
ELBOW_IDX = 2
SECOND_ROTATE_IDX = 3
THIRD_TILT_IDX = 4
THIRD_ROTATE_IDX = 5
END_EFFECTOR_IDX = 6

def send_unchecked_joystick_reading(control_id: int, axis_id: int, magnitude: float):
    joy_msg = JoystickReading()
    joy_msg.axis_id = axis_id
    joy_msg.control_id = control_id
    joy_msg.magnitude = magnitude
    publish_to_topic(JoystickReading, "/js_input", joy_msg)
    time.sleep(0.3)

def send_joystick_reading(control_id: int, axis_id: int, magnitude: float, idx: int, pred) -> float:
    """
    Sends a joystick reading and checks that the joint state has changed
    Returns the new joint position
    """
    joy_msg = JoystickReading()
    joy_msg.axis_id = axis_id
    joy_msg.control_id = control_id
    joy_msg.magnitude = magnitude
    publish_to_topic(JoystickReading, "/js_input", joy_msg)
    time.sleep(0.5)
    sub = TopicSubscriber(JointState, "/joint_states")
    msg = sub.wait_for_msg()
    print(f"Got reply {msg}")
    assert pred(msg.position[idx], msg.velocity[idx])
    last_pos = msg.position[idx]
    assert all(abs(x) <= sys.float_info.epsilon or i == idx
                for x, i in zip(msg.velocity, range(len(msg.velocity))))
    time.sleep(0.5)
    msg = sub.wait_for_msg()
    assert abs(msg.velocity[idx]) <= sys.float_info.epsilon
    return last_pos

# TODO(sev47): The test fixture class doesn't work here?

@cmr_node_test([
    make_launch_file("cmr_control", "forward_k_arm.launch.py"),
    make_fabric_node("cmr_arm", config_path="joystick_direct_control.toml"),
])
def test_basic_direct_control(namespace: str):
    assert activate_fabric_node("joystick_direct_control", namespace)

    time.sleep(4) # Give time for all nodes to start
    # TODO(sev47): Add waiting support for launch files

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Y_AXIS_ID, 0.04, SHOULDER_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Y_AXIS_ID, -0.04, SHOULDER_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Z_AXIS_ID, 0.04, BASE_ROTATE_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Z_AXIS_ID, -0.04, BASE_ROTATE_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    # Test elbow
    send_unchecked_joystick_reading(JoystickReading.THIRD_BUTTON_ID, 0, 0.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Y_AXIS_ID, 0.04, ELBOW_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Y_AXIS_ID, -0.04, ELBOW_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Z_AXIS_ID, 0.04, SECOND_ROTATE_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Z_AXIS_ID, -0.04, SECOND_ROTATE_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    # Test ignore
    send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.X_AXIS_ID, -0.04, SECOND_ROTATE_IDX,
                        lambda pos, vel: pos == last_pos and vel <= 0.0)

    # Test wrist
    send_unchecked_joystick_reading(JoystickReading.THIRD_BUTTON_ID, 0, 0.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Y_AXIS_ID, 0.04, THIRD_TILT_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Y_AXIS_ID, -0.04, THIRD_TILT_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Z_AXIS_ID, 0.04, THIRD_ROTATE_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Z_AXIS_ID, -0.04, THIRD_ROTATE_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    # Test ignore
    send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.X_AXIS_ID, -0.04, THIRD_ROTATE_IDX,
                        lambda pos, vel: pos == last_pos and vel <= 0.0)

    # Test loop back
    send_unchecked_joystick_reading(JoystickReading.THIRD_BUTTON_ID, 0, 0.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Y_AXIS_ID, 0.04, SHOULDER_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Y_AXIS_ID, -0.04, SHOULDER_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Z_AXIS_ID, 0.04, BASE_ROTATE_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    pos2 = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Z_AXIS_ID, -0.04, BASE_ROTATE_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)
    orig_diff = abs(pos2 - last_pos)

    # Test end effector

    last_pos = send_joystick_reading(JoystickReading.SECOND_JOYSTICK_ID, 
                        JoystickReading.Y_AXIS_ID, 0.04, END_EFFECTOR_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    last_pos = send_joystick_reading(JoystickReading.SECOND_JOYSTICK_ID,
                        JoystickReading.Y_AXIS_ID, -0.04, END_EFFECTOR_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    # Test sensitivity
    send_unchecked_joystick_reading(JoystickReading.SENS_SLIDER_ID, 0, 200.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Z_AXIS_ID, 0.04, BASE_ROTATE_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    pos2 = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Z_AXIS_ID, -0.04, BASE_ROTATE_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    assert abs(pos2 - last_pos) > orig_diff

    # Test reset sensitivity
    send_unchecked_joystick_reading(JoystickReading.SENS_SLIDER_ID, 0, 100.0)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, 
                        JoystickReading.Z_AXIS_ID, 0.04, BASE_ROTATE_IDX, 
                        lambda pos, vel: pos > 0.0 and vel >= 0.0)
    pos2 = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                        JoystickReading.Z_AXIS_ID, -0.04, BASE_ROTATE_IDX,
                        lambda pos, vel: pos < last_pos and vel <= 0.0)

    assert abs(abs(pos2 - last_pos) - orig_diff) < 0.1


    # Test multi messages
    for _ in range(5):
        send_unchecked_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID, JoystickReading.Y_AXIS_ID, 0.05)

    last_pos = send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                                    JoystickReading.Y_AXIS_ID, 0.04, SHOULDER_IDX,
                                    lambda pos, vel: pos > max(last_pos * 3, 1.) and vel >= 0.0)

    # Test ignore
    send_joystick_reading(JoystickReading.MAIN_JOYSTICK_ID,
                          JoystickReading.X_AXIS_ID, 0.04, SHOULDER_IDX,
                          lambda pos, vel: pos == last_pos and vel >= 0.0)
    
    