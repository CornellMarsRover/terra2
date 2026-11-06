# Drive modes

The active `usama_control_testing_node` supports four operator-selectable drive
modes. A cardinal D-pad press selects a mode, and neutral or diagonal D-pad
values leave the current mode unchanged.

| D-pad | Mode | Left stick | Right stick X |
| --- | --- | --- | --- |
| Up | Translation + rotation | Translate in any direction | Rotate while translating |
| Right | Ackermann | Forward/reverse only | Set turn curvature |
| Down | Point turn | Ignored | Rotate in place |
| Left | Steady heading | Translate in any direction | Ignored |

Changing mode sends a stop command before movement is accepted in the new mode.
The initial mode is configured with `drive_mode` in `config/drivesnet.toml`.

Ackermann mode uses the existing four-module swerve kinematics, constrains
lateral motion to zero, and scales yaw demand with forward demand. This prevents
an Ackermann command from turning the rover in place. Point-turn mode is the
dedicated way to spin around the rover center.
