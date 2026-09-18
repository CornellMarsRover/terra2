"""Pure drive-mode selection and command shaping helpers."""

from __future__ import annotations

from enum import Enum


class DriveMode(str, Enum):
    """Operator-selectable ways to use the rover's swerve modules."""

    TRANSLATION_ROTATION = "translation_rotation"
    ACKERMANN = "ackermann"
    STEADY_HEADING = "steady_heading"
    POINT_TURN = "point_turn"


# D-pad values are defined by cmr_controller_remote/connect.py.
DPAD_TO_DRIVE_MODE = {
    0: DriveMode.TRANSLATION_ROTATION,  # up
    2: DriveMode.ACKERMANN,             # right
    4: DriveMode.POINT_TURN,            # down
    6: DriveMode.STEADY_HEADING,        # left
}


def parse_drive_mode(value: str | DriveMode) -> DriveMode:
    """Return a validated drive mode from a config or runtime value."""

    if isinstance(value, DriveMode):
        return value
    return DriveMode(str(value).strip().lower())


def drive_mode_for_dpad(dpad: int) -> DriveMode | None:
    """Map a cardinal D-pad press to a mode; diagonals and neutral do nothing."""

    return DPAD_TO_DRIVE_MODE.get(int(dpad))


def shape_motion_for_mode(
    mode: DriveMode | str,
    vx: float,
    vy: float,
    omega: float,
) -> tuple[float, float, float]:
    """Apply the selected motion constraints before swerve kinematics.

    ``translation_rotation`` preserves all three commands.
    ``steady_heading`` allows translation but prevents body yaw.
    ``point_turn`` allows body yaw but prevents translation.
    ``ackermann`` prevents lateral translation and makes steering proportional
    to forward speed, so it cannot turn in place.
    """

    selected = parse_drive_mode(mode)
    vx = float(vx)
    vy = float(vy)
    omega = float(omega)

    if selected is DriveMode.TRANSLATION_ROTATION:
        return vx, vy, omega
    if selected is DriveMode.STEADY_HEADING:
        return vx, vy, 0.0
    if selected is DriveMode.POINT_TURN:
        return 0.0, 0.0, omega

    # Ackermann-style driving has no sideways translation and no stationary
    # spin. Scaling omega by forward demand preserves a consistent curvature
    # as the operator changes speed.
    return vx, 0.0, omega * abs(vx)
