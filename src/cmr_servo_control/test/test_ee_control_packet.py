import importlib.util
import sys
import types
from pathlib import Path


class FakeDualSense:
    pass


def load_ee_control_module():
    sys.modules.setdefault(
        "pydualsense",
        types.SimpleNamespace(pydualsense=FakeDualSense, TriggerModes=object),
    )
    sys.modules.setdefault(
        "hidapi",
        types.SimpleNamespace(Device=lambda path: object()),
    )

    repo_root = Path(__file__).resolve().parents[3]
    script_path = repo_root / "local_(no ROS)base_station_scripts" / "ee_control.py"
    spec = importlib.util.spec_from_file_location("ee_control_for_test", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeState:
    LX = 0
    LY = 0
    RX = 0
    RY = 0
    square = False
    cross = False
    circle = False
    triangle = False
    L1 = False
    R1 = False
    L2 = 0
    R2 = 0
    dpad = 8


def test_packet_preserves_dpad_byte():
    module = load_ee_control_module()
    state = FakeState()

    state.dpad = module.DPAD_NEUTRAL
    assert module.format_data_for_udp(state)[12] == 8

    state.dpad = module.DPAD_UP
    assert module.format_data_for_udp(state)[12] == 0

    state.dpad = module.DPAD_RIGHT
    assert module.format_data_for_udp(state)[12] == 2

    state.dpad = module.DPAD_DOWN
    assert module.format_data_for_udp(state)[12] == 4

    state.dpad = module.DPAD_LEFT
    assert module.format_data_for_udp(state)[12] == 6


def test_square_and_circle_stay_in_existing_button_slots():
    module = load_ee_control_module()
    state = FakeState()
    state.square = True
    state.circle = True

    packet = module.format_data_for_udp(state)

    assert packet[8] == 1
    assert packet[10] == 1
