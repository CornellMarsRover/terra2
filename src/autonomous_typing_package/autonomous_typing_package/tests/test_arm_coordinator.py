import rclpy
import pytest

from autonomous_typing_package.arm_coordinator import ArmCoordinator


def setup_module():
    if not rclpy.ok():
        rclpy.init()


def teardown_module():
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def coordinator():
    node = ArmCoordinator()
    yield node
    node.destroy_node()


def test_sequence_is_c_m_r_with_press_and_retract(coordinator):
    assert [segment["name"] for segment in coordinator.segments] == [
        "MOVE_A_TO_C",
        "PRESS_C",
        "RETRACT_C",
        "MOVE_C_TO_M",
        "PRESS_M",
        "RETRACT_M",
        "MOVE_M_TO_R",
        "PRESS_R",
        "RETRACT_R",
    ]


def test_hardcoded_key_offsets_are_sequential(coordinator):
    offsets = {segment["name"]: segment["offset"] for segment in coordinator.segments}

    assert offsets["MOVE_A_TO_C"] == pytest.approx((0.1146, 0.0570, 0.0))
    assert offsets["MOVE_C_TO_M"] == pytest.approx((0.2292, 0.0, 0.0))
    assert offsets["MOVE_M_TO_R"] == pytest.approx((-0.1719, -0.1170, 0.0))


def test_each_press_and_retract_is_three_inches(coordinator):
    for key in "CMR":
        press = coordinator.segments[[s["name"] for s in coordinator.segments].index(f"PRESS_{key}")]
        retract = coordinator.segments[[s["name"] for s in coordinator.segments].index(f"RETRACT_{key}")]
        assert press["offset"] == pytest.approx((0.0, 0.0, -0.0762))
        assert retract["offset"] == pytest.approx((0.0, 0.0, 0.0762))


def test_segment_duration_matches_unitless_servo_scale(coordinator):
    segment = coordinator.segments[0]
    distance = (0.1146**2 + 0.0570**2) ** 0.5
    expected_speed = 0.05 * 0.8
    assert segment["duration"] == pytest.approx(distance / expected_speed)


def test_segment_command_has_configured_magnitude(coordinator):
    for segment in coordinator.segments:
        magnitude = sum(value * value for value in segment["command"]) ** 0.5
        assert magnitude == pytest.approx(coordinator.command_magnitude)
