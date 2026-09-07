# Autonomy architecture

This map describes the current `autonomy-fall2026` data path. Solid edges are
shared runtime behavior; dashed edges are Gazebo adapters used only at system
boundaries. Arm packages are intentionally outside this document.

```mermaid
flowchart LR
  Mission[Waypoint YAML] --> SM[State machine]
  Camera[ZED point cloud] --> CM[Costmap]
  Camera --> OD[Object detection]
  GPS[RTK GPS + IMU] --> LOC[Localization]
  LOC -->|/autonomy/pose/robot/global| SM
  LOC -->|pose| GP[Global planner]
  LOC -->|pose| CM
  LOC -->|pose| LP[Local planner]
  LOC -->|pose| CTRL[Controller]
  OD -->|/autonomy/target_object/position| SM
  OD -->|target object| CM
  SM -->|/autonomy/target/global| GP
  SM -->|/autonomy/target/local| LP
  GP -->|/autonomy/target/local| LP
  CM -->|/autonomy/costmap| LP
  LP -->|/autonomy/path/next_waypoint| CTRL
  CTRL -->|/cmd_vel_drives| DRIVE[Shared RoverNet drive node]
  UDP[Controller UDP] --> RX[cmr_controller_remote]
  RX -->|manual TwistStamped + buttons| DRIVE
  DRIVE --> KIN[Swerve kinematics + Moteus worker]
  KIN --> HW[Drive and steer motors]

  GZ[Gazebo rover + obstacles] -. /drives/odom .-> OA[Odometry pose adapter]
  OA -. pose .-> SM
  OA -. pose .-> GP
  OA -. pose .-> CM
  OA -. pose .-> LP
  OA -. pose .-> CTRL
  PC[Simulated camera adapter] -. /camera/points .-> CM
  CTRL -. /cmd_vel_drives .-> GB[Gazebo drive bridge]
  GB -. /drives/cmd_vel .-> GZ
```

## Convergence boundary

Tele-op and autonomy differ only before the shared drive node. Tele-op produces
manual controller topics; autonomy produces normalized `Twist` commands on
`/cmd_vel_drives`. The RoverNet node arbitrates those inputs and owns the same
swerve geometry, Moteus transport, watchdog, and motor command path for both.
Gazebo replaces that final hardware backend, not perception or planning logic.
