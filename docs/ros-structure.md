# ROS structure

This graph shows the canonical runtime path. Labels are ROS topics; boxes are
nodes or external systems. Dashed edges are simulation boundary adapters.

```mermaid
flowchart LR
  PAD[Controller] -->|UDP 5010| RX[connect_node]
  RX -->|/controller/drives/axes + buttons| DRIVE[drivesnet adapter/backend]
  DRIVE -->|/cmd_vel/teleop: DriveCommand| MUX[drive_command_mux]
  GPS[RTK GPS] --> LOC[new_kalman]
  IMU[IMU] --> LOC
  ZED[ZED camera] -->|/camera/points| CM[costmap]
  ZED -->|/zed/image| OD[object_detection]
  LOC -->|/autonomy/pose/robot/global| SM[state_machine] & CM & LP & CTRL
  SM -->|/autonomy/target/local| LP[local_planner]
  OD -->|/autonomy/target_object/position| SM
  OD --> CM
  CM -->|/autonomy/costmap| LP
  LP -->|/autonomy/path/next_waypoint| CTRL[controller]
  CTRL -->|/cmd_vel/autonomy: DriveCommand| MUX
  SAFE[/cmd_vel/source + /cmd_vel/estop] --> MUX
  MUX -->|/cmd_vel: DriveCommand| DRIVE
  DRIVE -->|swerve targets| MOT[Moteus drive + steer motors]
  GZ[Gazebo] -. /drives/odom .-> POSE[pose adapter]
  POSE -. /autonomy/pose/robot/global .-> SM & CM & LP & CTRL
  MUX -. selected command .-> GB[Gazebo drive adapter]
  GB -. /drives/cmd_vel .-> GZ
```

## Drive contract

| Topic | Type | Owner |
| --- | --- | --- |
| `/cmd_vel/teleop` | `cmr_msgs/DriveCommand` | manual adapter |
| `/cmd_vel/autonomy` | `cmr_msgs/DriveCommand` | autonomy controller |
| `/cmd_vel/source` | `std_msgs/String` | operator/launch mode |
| `/cmd_vel/estop` | `std_msgs/Bool` | safety inputs |
| `/cmd_vel` | `cmr_msgs/DriveCommand` | mux only; backend input |

`DriveCommand` carries normalized `vx`, `vy`, and `omega`, plus signed
`speed_rps`. The mux invalidates buffered commands on source changes, enforces a
0.5-second timeout, latches estop, and publishes zeros when motion is unsafe.
