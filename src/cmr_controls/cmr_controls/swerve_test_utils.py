import math
from dataclasses import dataclass
from typing import Dict, List, Tuple


L = 0.83
W = 0.83
WHEEL_RADIUS = 0.127
WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * math.pi
SWERVE_RATIO = 50
DRIVE_RATIO = 26


DRIVE_MOTOR_IDS = {
    "FL_DRIVE": 1,
    "BL_DRIVE": 2,
    "FR_DRIVE": 3,
    "BR_DRIVE": 4,
}

SWERVE_MOTOR_IDS = {
    "FL_SWERVE": 5,
    "BL_SWERVE": 6,
    "FR_SWERVE": 7,
    "BR_SWERVE": 8,
}

ALL_MOTOR_IDS = {**DRIVE_MOTOR_IDS, **SWERVE_MOTOR_IDS}

# Mirror the live controller's remap exactly so test patterns match deployed behavior.
LOGICAL_DRIVE_TO_SERVO = {
    "ws1": 4,
    "ws2": 1,
    "ws3": 2,
    "ws4": 3,
}

LOGICAL_SWERVE_TO_SERVO = {
    "wa1": 7,
    "wa2": 5,
    "wa3": 6,
    "wa4": 8,
}

POINT_TURN_ANGLES = {
    "wa1": (-45.0 / 360.0) * SWERVE_RATIO,
    "wa2": (45.0 / 360.0) * SWERVE_RATIO,
    "wa3": (-45.0 / 360.0) * SWERVE_RATIO,
    "wa4": (45.0 / 360.0) * SWERVE_RATIO,
}


@dataclass(frozen=True)
class ServoPattern:
    key: str
    label: str
    description: str
    drive_targets: Tuple[float, float, float, float]
    swerve_targets: Tuple[float, float, float, float]
    topic_mode: str
    topic_payload: Dict[str, float]


def degrees_to_servo_turns(angle_deg: float) -> float:
    return (angle_deg / 360.0) * SWERVE_RATIO


def servo_targets_from_logical(
    ws1: float,
    ws2: float,
    ws3: float,
    ws4: float,
    wa1: float,
    wa2: float,
    wa3: float,
    wa4: float,
) -> Dict[int, Dict[str, float]]:
    targets = {}
    drive_values = {"ws1": ws1, "ws2": ws2, "ws3": ws3, "ws4": ws4}
    swerve_values = {"wa1": wa1, "wa2": wa2, "wa3": wa3, "wa4": wa4}

    for key, servo_id in LOGICAL_DRIVE_TO_SERVO.items():
        targets[servo_id] = {"velocity": drive_values[key]}
    for key, servo_id in LOGICAL_SWERVE_TO_SERVO.items():
        targets[servo_id] = {"position": swerve_values[key]}
    return targets


def point_turn_command(angular_z: float) -> Tuple[float, float, float, float, float, float, float, float]:
    r = math.sqrt(((L / 2.0) ** 2) + ((W / 2.0) ** 2))
    v = ((angular_z * r) / WHEEL_CIRCUMFERENCE) * DRIVE_RATIO
    return (
        v,
        v,
        v,
        v,
        POINT_TURN_ANGLES["wa1"],
        POINT_TURN_ANGLES["wa2"],
        POINT_TURN_ANGLES["wa3"],
        POINT_TURN_ANGLES["wa4"],
    )


def ackermann_command(
    velocity_mps: float,
    front_left_angle_deg: float,
) -> Tuple[float, float, float, float, float, float, float, float]:
    s = (velocity_mps / WHEEL_CIRCUMFERENCE) * DRIVE_RATIO

    if abs(front_left_angle_deg) > 1.0:
        tangent = math.tan(math.radians(front_left_angle_deg))
        radius = L / tangent
        left_radius = radius - (W / 2.0)
        right_radius = radius + (W / 2.0)
        left_velocity = s * (left_radius / radius)
        right_velocity = s * (right_radius / radius)
        theta_left = math.degrees(math.atan(L / left_radius))
        theta_right = math.degrees(math.atan(L / right_radius))
    else:
        theta_left = 0.0
        theta_right = 0.0
        left_velocity = s
        right_velocity = s

    wa3 = -1.0 * degrees_to_servo_turns(theta_left)
    wa4 = -1.0 * degrees_to_servo_turns(theta_right)
    wa2 = 0.0
    wa1 = 0.0

    return -1.0 * left_velocity, s, right_velocity, -1.0 * s, wa1, wa2, wa3, wa4


def movement_patterns(speed_scale: float) -> List[ServoPattern]:
    clamped_speed = max(0.05, min(speed_scale, 1.0))
    turn_rate = 0.20 * clamped_speed
    linear_speed = 0.10 * clamped_speed
    steer_angle = 12.0 * clamped_speed

    patterns = [
        ServoPattern(
            key="straight_fwd",
            label="Straight forward",
            description="All drive motors should spin for forward motion with swerve modules held straight.",
            drive_targets=ackermann_command(linear_speed, 0.0)[:4],
            swerve_targets=ackermann_command(linear_speed, 0.0)[4:],
            topic_mode="ackerman",
            topic_payload={"vel": linear_speed, "fl_angle": 0.0},
        ),
        ServoPattern(
            key="straight_rev",
            label="Straight reverse",
            description="All drive motors should reverse while the swerve modules remain straight.",
            drive_targets=ackermann_command(-linear_speed, 0.0)[:4],
            swerve_targets=ackermann_command(-linear_speed, 0.0)[4:],
            topic_mode="ackerman",
            topic_payload={"vel": -linear_speed, "fl_angle": 0.0},
        ),
        ServoPattern(
            key="point_turn_cw",
            label="Point turn CW",
            description="Modules should move to the point-turn angles and all drives should spin to rotate clockwise.",
            drive_targets=point_turn_command(-turn_rate)[:4],
            swerve_targets=point_turn_command(-turn_rate)[4:],
            topic_mode="point_turn",
            topic_payload={"angular_z": -turn_rate},
        ),
        ServoPattern(
            key="point_turn_ccw",
            label="Point turn CCW",
            description="Modules should move to the point-turn angles and all drives should spin to rotate counter-clockwise.",
            drive_targets=point_turn_command(turn_rate)[:4],
            swerve_targets=point_turn_command(turn_rate)[4:],
            topic_mode="point_turn",
            topic_payload={"angular_z": turn_rate},
        ),
        ServoPattern(
            key="ackermann_left",
            label="Gentle left ackermann",
            description="Rear modules stay near zero, front modules steer left, and left/right drive speeds should differ.",
            drive_targets=ackermann_command(linear_speed, steer_angle)[:4],
            swerve_targets=ackermann_command(linear_speed, steer_angle)[4:],
            topic_mode="ackerman",
            topic_payload={"vel": linear_speed, "fl_angle": steer_angle},
        ),
        ServoPattern(
            key="ackermann_right",
            label="Gentle right ackermann",
            description="Rear modules stay near zero, front modules steer right, and left/right drive speeds should differ.",
            drive_targets=ackermann_command(linear_speed, -steer_angle)[:4],
            swerve_targets=ackermann_command(linear_speed, -steer_angle)[4:],
            topic_mode="ackerman",
            topic_payload={"vel": linear_speed, "fl_angle": -steer_angle},
        ),
    ]
    return patterns


def format_expected_servo_targets(pattern: ServoPattern) -> str:
    targets = servo_targets_from_logical(
        *pattern.drive_targets,
        *pattern.swerve_targets,
    )
    lines = [f"Pattern: {pattern.label}", pattern.description, "Expected servo targets:"]
    for servo_name, servo_id in ALL_MOTOR_IDS.items():
        target = targets.get(servo_id, {})
        if "velocity" in target:
            lines.append(f"  - {servo_name} (id={servo_id}): velocity={target['velocity']:.3f}")
        elif "position" in target:
            lines.append(f"  - {servo_name} (id={servo_id}): position={target['position']:.3f}")
    return "\n".join(lines)
