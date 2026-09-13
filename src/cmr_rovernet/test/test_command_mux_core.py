import pytest
from cmr_rovernet.command_mux_core import CommandMux


def test_selected_zero_does_not_fall_through():
    mux = CommandMux()
    mux.receive("teleop", (0, 0, 0), 1)
    mux.receive("autonomy", (1, 0, 0), 1)
    assert mux.output(1.1) == (0, 0, 0)
    assert mux.output(1.5) is None


def test_switch_requires_fresh_selected_input():
    mux = CommandMux()
    mux.receive("autonomy", (1,), 1)
    mux.select("autonomy")
    assert mux.output(1.1) is None
    mux.receive("autonomy", (2,), 1.2)
    assert mux.output(1.3) == (2,)


def test_estop_discards_commands_until_reset_and_fresh_input():
    mux = CommandMux()
    mux.receive("teleop", (1,), 1)
    mux.emergency_stop()
    mux.receive("teleop", (2,), 1.1)
    assert mux.output(1.2) is None
    mux.reset()
    assert mux.output(1.2) is None
    mux.receive("teleop", (0,), 1.3)
    assert mux.output(1.4) == (0,)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_invalid_input_clears_previous_motion(bad):
    mux = CommandMux()
    mux.receive("teleop", (1,), 1)
    assert not mux.receive("teleop", (bad,), 1.1)
    assert mux.output(1.2) is None
    with pytest.raises(ValueError):
        CommandMux(bad)
