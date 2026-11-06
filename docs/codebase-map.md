# Terra2 codebase map

Source-verified on `autonomy_fall2026`. The workspace contains 20 ROS 2
packages plus the generated `cmr_arm_simulator_moveit_config` package.

```mermaid
flowchart TD
  ROOT[terra2] --> RUN[run + scripts: setup, launch, test, sim checks]
  ROOT --> DOCS[docs + codebase_index: architecture and operations]
  ROOT --> SRC[src: ROS 2 workspace]
  SRC --> CORE[Core: cmr_msgs, cmr_utils, cmr_fabric, cmr_fabric_wrappers]
  SRC --> AUTO[Autonomy: autonomous_navigation]
  SRC --> DRIVE[Drive: cmr_controller_remote, cmr_rovernet, cmr_controls]
  SRC --> SENSE[Sensing: cmr_zed, cmr_cams, usb_camera_publisher, cmr_imu, cmr_rtkgps, cmr_cv]
  SRC --> ARM[Arm: cmr_arm_sim, cmr_arm_simulator, moveit_servo, cmr_servo_control]
  SRC --> SCI[Science: astrotech_rover]
  SRC --> TOOLS[Tools: cmr_param_gui, autonomous_typing_package]
  AUTO --> PURE[Pure logic: planner_core, costmap_core, state_machine_core, drive_command]
  AUTO --> ADAPTERS[ROS adapters: state machine, planner, costmap, controller, localization]
  DRIVE --> MUX[DriveCommand mux + shared Moteus backend]
  CORE --> AUTO
  CORE --> DRIVE
```

| Area | Primary ownership |
| --- | --- |
| Interfaces | `src/cmr_msgs/msg`, `src/cmr_msgs/srv` |
| Runtime orchestration | `cmr_fabric`, TOML configs, package `launch/` folders |
| Autonomy decisions | `src/autonomous_navigation/autonomous_navigation` |
| Tele-op ingress | `src/cmr_controller_remote`, controller-side `zenchang_controller` |
| Drive actuation | `src/cmr_rovernet`; `cmr_controls` retains compatibility/tools |
| Perception/localization | `cmr_zed`, `cmr_cams`, `cmr_imu`, `cmr_rtkgps` |
| Arm stack | `cmr_arm_sim*`, `moveit_servo`, `cmr_servo_control` |
| Science stack | `src/astrotech_rover` |
| Tests | package `test/` folders and `scripts/test_autonomy.sh` |

## Main entry points

- `./run teleop`: controller receiver, mux in tele-op mode, shared drive backend.
- `./run auto`: sensors, autonomy pipeline, mux in autonomy mode, shared backend.
- `./run sim`: simulated-input autonomy nodes; Gazebo remains an external harness.
- `./run test`: focused autonomy and drive safety tests.

See [ROS structure](ros-structure.md) for nodes/topics and
[autonomy architecture](autonomy-architecture.md) for planner internals.
