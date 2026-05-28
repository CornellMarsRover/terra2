from cmr_servo_control.end_effector_servo_mapper import (
    DPAD_DOWN,
    DPAD_LEFT,
    DPAD_NEUTRAL,
    DPAD_RIGHT,
    DPAD_UP,
    DRILL_FORWARD_DEGREES,
    DRILL_REVERSE_DEGREES,
    DRILL_STOP_DEGREES,
    EndEffectorServoMapper,
    SERVO_CAN_IDS,
    SERVO_EE0,
    SERVO_EE11,
    SERVO_EE15,
    SERVO_IDS,
)


def test_dpad_edges_step_once_until_released():
    mapper = EndEffectorServoMapper()

    assert mapper.update([0] * 8, DPAD_UP) == [("ee11", 105)]
    assert mapper.update([0] * 8, DPAD_UP) == []
    assert mapper.update([0] * 8, DPAD_NEUTRAL) == []
    assert mapper.update([0] * 8, DPAD_UP) == [("ee11", 110)]
    assert mapper.update([0] * 8, DPAD_DOWN) == [("ee11", 105)]


def test_dpad_left_right_command_drill_spin_servo_positions():
    mapper = EndEffectorServoMapper()

    assert mapper.update([0] * 8, DPAD_LEFT) == [
        ("ee0", DRILL_REVERSE_DEGREES)
    ]
    assert mapper.update([0] * 8, DPAD_RIGHT) == [
        ("ee0", DRILL_FORWARD_DEGREES)
    ]


def test_dpad_left_right_toggle_drill_to_stop():
    mapper = EndEffectorServoMapper()

    assert mapper.update([0] * 8, DPAD_RIGHT) == [
        ("ee0", DRILL_FORWARD_DEGREES)
    ]
    assert mapper.update([0] * 8, DPAD_NEUTRAL) == []
    assert mapper.update([0] * 8, DPAD_RIGHT) == [("ee0", DRILL_STOP_DEGREES)]

    assert mapper.update([0] * 8, DPAD_LEFT) == [
        ("ee0", DRILL_REVERSE_DEGREES)
    ]
    assert mapper.update([0] * 8, DPAD_NEUTRAL) == []
    assert mapper.update([0] * 8, DPAD_LEFT) == [("ee0", DRILL_STOP_DEGREES)]


def test_triangle_x_edges_step_ee15_once_until_released():
    mapper = EndEffectorServoMapper()
    buttons = [0] * 8

    buttons[7] = 1
    assert mapper.update(buttons, DPAD_NEUTRAL) == [("ee15", 100)]
    assert mapper.update(buttons, DPAD_NEUTRAL) == []

    buttons[7] = 0
    assert mapper.update(buttons, DPAD_NEUTRAL) == []

    buttons[5] = 1
    assert mapper.update(buttons, DPAD_NEUTRAL) == [("ee15", 95)]


def test_incremental_servos_are_clamped():
    mapper = EndEffectorServoMapper()

    mapper.state["ee11"] = 150
    mapper.state["ee15"] = 60

    assert mapper.update([0] * 8, DPAD_UP) == []
    assert mapper.update([0] * 8, DPAD_NEUTRAL) == []

    buttons = [0] * 8
    buttons[5] = 1
    assert mapper.update(buttons, DPAD_NEUTRAL) == []


def test_discovered_servo_board_mapping():
    assert SERVO_IDS == {
        SERVO_EE0: 0,
        SERVO_EE11: 11,
        SERVO_EE15: 15,
    }
    assert SERVO_CAN_IDS == {
        SERVO_EE0: 25,
        SERVO_EE11: 25,
        SERVO_EE15: 26,
    }
