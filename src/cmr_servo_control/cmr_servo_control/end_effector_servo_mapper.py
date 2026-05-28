DPAD_UP = 0
DPAD_RIGHT = 2
DPAD_DOWN = 4
DPAD_LEFT = 6
DPAD_NEUTRAL = 8

BUTTON_X_INDEX = 5
BUTTON_TRIANGLE_INDEX = 7

STEP_DEGREES = 5
DRILL_STOP_DEGREES = 90
DRILL_REVERSE_DEGREES = 0
DRILL_FORWARD_DEGREES = 180

SERVO_EE0 = "ee0"
SERVO_EE11 = "ee11"
SERVO_EE15 = "ee15"

INITIAL_STATE = {
    SERVO_EE0: DRILL_STOP_DEGREES,
    SERVO_EE11: 100,
    SERVO_EE15: 95,
}

SERVO_IDS = {
    SERVO_EE0: 0,
    SERVO_EE11: 11,
    SERVO_EE15: 15,
}

SERVO_CAN_IDS = {
    SERVO_EE0: 25,
    SERVO_EE11: 25,
    SERVO_EE15: 26,
}

SERVO_MIN = {
    SERVO_EE11: 40,
    SERVO_EE15: 75,
}

SERVO_MAX = {
    SERVO_EE11: 150,
    SERVO_EE15: 120,
}


def clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


class EndEffectorServoMapper:
    def __init__(self):
        self.state = dict(INITIAL_STATE)
        self.previous_dpad = DPAD_NEUTRAL
        self.previous_buttons = []

    def update(self, button_array, dpad: int):
        commands = []

        if dpad != self.previous_dpad:
            if dpad == DPAD_UP:
                commands.append(
                    self._step_servo(SERVO_EE11, STEP_DEGREES)
                )
            elif dpad == DPAD_DOWN:
                commands.append(
                    self._step_servo(SERVO_EE11, -STEP_DEGREES)
                )
            elif dpad == DPAD_LEFT:
                commands.append(
                    self._toggle_servo(
                        SERVO_EE0,
                        DRILL_REVERSE_DEGREES,
                        DRILL_STOP_DEGREES,
                    )
                )
            elif dpad == DPAD_RIGHT:
                commands.append(
                    self._toggle_servo(
                        SERVO_EE0,
                        DRILL_FORWARD_DEGREES,
                        DRILL_STOP_DEGREES,
                    )
                )

        if self._rising_edge(button_array, BUTTON_TRIANGLE_INDEX):
            commands.append(self._step_servo(SERVO_EE15, STEP_DEGREES))
        if self._rising_edge(button_array, BUTTON_X_INDEX):
            commands.append(self._step_servo(SERVO_EE15, -STEP_DEGREES))

        self.previous_dpad = dpad
        self.previous_buttons = list(button_array)

        return [command for command in commands if command is not None]

    def _step_servo(self, name: str, delta: int):
        return self._set_servo(name, self.state[name] + delta)

    def _toggle_servo(self, name: str, active_angle: int, inactive_angle: int):
        if self.state[name] == active_angle:
            return self._set_servo(name, inactive_angle)
        return self._set_servo(name, active_angle)

    def _set_servo(self, name: str, angle: int):
        if name in SERVO_MIN:
            angle = clamp(angle, SERVO_MIN[name], SERVO_MAX[name])
        angle = int(angle)
        if self.state[name] == angle:
            return None
        self.state[name] = angle
        return name, angle

    def _rising_edge(self, button_array, index: int) -> bool:
        current = self._button_pressed(button_array, index)
        previous = self._button_pressed(self.previous_buttons, index)
        return current and not previous

    @staticmethod
    def _button_pressed(button_array, index: int) -> bool:
        return index < len(button_array) and bool(button_array[index])
