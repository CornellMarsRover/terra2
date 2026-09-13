# ROS structure

Read this graph from top to bottom. Operator and sensor inputs are furthest from
the rover; the physical wheels and steering are the lowest layer. It shows the
primary driving topics in the committed real-hardware path.

```mermaid
flowchart TD
  PAD["Controller"] -->|"UDP 5010 or 5005"| RX["controller_connect<br/>connect_node executable"]
  RX -->|"/controller/drives/axes<br/>/controller/drives/buttons"| DRIVE["drivesnet<br/>usama_control_testing_node"]
  DRIVE -->|"/cmd_vel/teleop"| MUX["drive_command_mux"]
  DRIVE -->|"/cmd_vel/estop"| MUX
  GPS["gps_rover"] -->|"/rtk/navsatfix_data"| LOC["new_kalman"]
  ZED["zed_autonomy<br/>threaded executable"] -->|"/zed/pose"| LOC
  ZED -->|"/camera/points<br/>/camera/ground_plane"| CM["costmap"]
  ZED -->|"/camera/ground_plane"| LP["local_planner"]
  ZED -->|"/zed/image"| OD["object_detection"]
  LOC -->|"/autonomy/pose/robot/global"| SM["state_machine"]
  LOC -->|"/autonomy/pose/robot/global"| CM
  LOC -->|"/autonomy/pose/robot/global"| LP
  LOC -->|"/autonomy/pose/robot/global"| CTRL["controller"]
  SM -->|"/autonomy/target/local"| LP
  SM -->|"/autonomy/target_object/name"| OD
  OD -->|"/autonomy/target_object/position"| SM
  OD -->|"/autonomy/target_object/position"| CM
  CTRL -->|"/autonomy/move/move_type"| CM
  CM -->|"/autonomy/costmap"| LP
  LP -->|"/autonomy/path/next_waypoint"| CTRL
  CTRL -->|"/cmd_vel/autonomy"| MUX
  MODE["active_source parameter<br/>or /cmd_vel/source"] --> MUX
  MUX -->|"/cmd_vel"| DRIVE
  DRIVE -->|"fdcanusb Moteus commands"| MOT["Moteus controllers"]
  MOT --> ROBOT["physical drive and steer motors"]
```

`drivesnet` is intentionally shown on both sides of the mux because one node
both converts controller input and sends the selected command to Moteus.

## Simulation boundary

The committed `sim_autonomy.launch.py` starts the mux and autonomy nodes, not
Gazebo or a drive adapter. Its `localization_sim` input contract is `/gps_exact`,
`/navsatfix`, and `/imu`; an external simulator must provide those inputs and
consume `/cmd_vel`.

## Drive contract

| Topic | Type | Owner |
| --- | --- | --- |
| `/cmd_vel/teleop` | `cmr_msgs/DriveCommand` | `drivesnet` |
| `/cmd_vel/autonomy` | `cmr_msgs/DriveCommand` | `controller` |
| `/cmd_vel/source` | `std_msgs/String` | operator tooling |
| `/cmd_vel/estop` | `std_msgs/Bool` | `drivesnet` |
| `/cmd_vel` | `cmr_msgs/DriveCommand` | mux only; backend input |

`DriveCommand` carries normalized `vx`, `vy`, and `omega`, plus signed
`speed_rps`. The mux invalidates buffered commands on source changes, enforces a
0.5-second timeout, and publishes zeros while estop is asserted or a command is
stale.

## Other domains

- Arm: controller Joy -> MoveIt Servo -> joint trajectory -> RoverNet arm backend.
- Cameras: USB/ZED nodes publish raw, rectified, stitched, and bird's-eye images.
- Science: `astrotech_node` owns auger, Raman, environment, and mixing interfaces.
- Fabric: lifecycle manager and fault handler supervise TOML-composed nodes.

Legacy arm IK utilities also use `/cmd_vel` as `Twist`; remap them before
co-launching because the canonical drive topic uses `DriveCommand`.
