from cmr_servo_control.end_effector_servo_mapper import (
    DPAD_DOWN,
    DPAD_LEFT,
    DPAD_NEUTRAL,
    DPAD_RIGHT,
    DPAD_UP,
    EndEffectorServoMapper,
)


def test_dpad_edges_step_once_until_released():
    mapper = EndEffectorServoMapper()

    assert mapper.update([0] * 8, DPAD_UP) == [("ee11", 105)]
    assert mapper.update([0] * 8, DPAD_UP) == []
    assert mapper.update([0] * 8, DPAD_NEUTRAL) == []
    assert mapper.update([0] * 8, DPAD_UP) == [("ee11", 110)]
    assert mapper.update([0] * 8, DPAD_DOWN) == [("ee11", 105)]


def test_dpad_left_right_command_absolute_spin_servo_positions():
    mapper = EndEffectorServoMapper()

    assert mapper.update([0] * 8, DPAD_LEFT) == [("ee0", 0)]
    assert mapper.update([0] * 8, DPAD_RIGHT) == [("ee0", 180)]


def test_square_circle_edges_step_ee15_once_until_released():
    mapper = EndEffectorServoMapper()
    buttons = [0] * 8

    buttons[4] = 1
    assert mapper.update(buttons, DPAD_NEUTRAL) == [("ee15", 100)]
    assert mapper.update(buttons, DPAD_NEUTRAL) == []

    buttons[4] = 0
    assert mapper.update(buttons, DPAD_NEUTRAL) == []

    buttons[6] = 1
    assert mapper.update(buttons, DPAD_NEUTRAL) == [("ee15", 95)]


def test_incremental_servos_are_clamped():
    mapper = EndEffectorServoMapper()

    mapper.state["ee11"] = 150
    mapper.state["ee15"] = 40

    assert mapper.update([0] * 8, DPAD_UP) == []
    assert mapper.update([0] * 8, DPAD_NEUTRAL) == []

    buttons = [0] * 8
    buttons[6] = 1
    assert mapper.update(buttons, DPAD_NEUTRAL) == []
