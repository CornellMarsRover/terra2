# ROS structure

Read this graph from top to bottom. Operator and sensor inputs are furthest from
the rover; the physical wheels and steering are the lowest layer.

```mermaid
flowchart TD
  PAD["Controller"] -->|"UDP 5010"| RX["connect_node"]
  RX -->|"controller drive topics"| TELEOP["tele-op command adapter"]
  TELEOP -->|"/cmd_vel/teleop"| MUX["drive_command_mux"]
  GPS["RTK GPS"] --> LOC["new_kalman"]
  IMU["IMU"] --> LOC
  ZED["ZED camera"] -->|"/camera/points"| CM["costmap"]
  ZED -->|"/zed/image"| OD["object_detection"]
  LOC -->|"global autonomy pose"| SM["state_machine"]
  LOC --> CM
  LOC --> LP["local_planner"]
  LOC --> CTRL["controller"]
  SM -->|"/autonomy/target/local"| LP
  OD -->|"target object position"| SM
  OD --> CM
  CM -->|"/autonomy/costmap"| LP
  LP -->|"/autonomy/path/next_waypoint"| CTRL
  CTRL -->|"/cmd_vel/autonomy"| MUX
  SAFE["source selection and estop"] --> MUX
  MUX -->|"/cmd_vel"| BACKEND["shared RoverNet drive backend"]
  BACKEND -->|"swerve targets"| MOT["Moteus motor controllers"]
  MOT --> ROBOT["physical rover wheels and steering"]
```

Gazebo replaces the three lowest hardware layers during simulation:
`/cmd_vel -> Gazebo drive adapter -> Gazebo rover -> odometry pose adapter`.

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

## Other domains

- Arm: controller Joy -> MoveIt Servo -> joint trajectory -> RoverNet arm backend.
- Cameras: USB/ZED nodes publish raw, rectified, stitched, and bird's-eye images.
- Science: `astrotech_node` owns auger, Raman, environment, and mixing interfaces.
- Fabric: lifecycle manager and fault handler supervise TOML-composed nodes.

Legacy arm IK utilities also use `/cmd_vel` as `Twist`; remap them before
co-launching because the canonical drive topic uses `DriveCommand`.
