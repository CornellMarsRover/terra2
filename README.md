# Terra
[![Build and test](https://github.com/CornellMarsRover/terra/actions/workflows/build-test.yaml/badge.svg?branch=main)](https://github.com/CornellMarsRover/terra/actions/workflows/build-test.yaml)

Terra contains all the code that is part of a ROS codebase and implements the brains of our devices. This main branch contains the current fully integrated and tested functionality for the rover.

## Current Functionality

### RoverNet: [cmr_rovernet](./src/cmr_rovernet/)
- **Description:** CMR controls stack (Jetson ↔ ECE) for arm control, drives control, and ECE feedback.

### Controller Input: [cmr_controller_remote](./src/cmr_controller_remote/)
- **Description:** PS5 controller input reception for data sent by team laptop over UDP.
- **Flow:** Controller → Laptop → Jetson → `cmr_controller_remote` → `cmr_rovernet`.

### Arm URDF Data: [cmr_arm_sim](./src/cmr_arm_sim/)
- **Description:** Hosts the Rover Arm's URDF for our Inverse Kinematics algorithms.
- **Additional:** Includes a launch file to visualize the arm: [display.launch.py](./src/cmr_arm_sim/launch).

### Arm Inverse Kinematics: [cmr_arm_simulator](./src/cmr_arm_simulator/)
- **Description:** Contains the ROS2 MoveIt configs and basic launch files to implement IK for the rover's arm.

### Arm PS5 Controller Control: [moveit_servo](./src/cmr_arm_simulator/)
- **Description:** Initializes the IK algorithm, takes controller data from `cmr_controller_remote`, and outputs arm joint positions/velocities to `cmr_rovernet`.

### Fabric Nodes: [cmr_fabric](./src/cmr_fabric/)
- **Description:** Custom nodes with better fault handling and configuration management.
- **Features:**
  - Launch ROS nodes using a config folder with TOML files.
  - Avoids manually launching each node.
  - Example: `rovernet.launch.py` and the config folder in `cmr_rovernet`.

### IMU Input: [cmr_imu](./src/cmr_imu)
- **Description:** Pulls data from WitMotion's HWT905 IMU and feeds it into an IMU topic for other packages.

### Custom Messages: [cmr_msgs](./src/cmr_msgs)
- **Description:** Contains and initializes custom message types for CMR-specific node-to-node communication.

### C++ Utility Functions: [cmr_utils](./src/cmr_utils)
- **Description:** Custom functions to simplify C++ ROS2 development (not used much anymore).

### Camera Integration: [cmr_cams](./src/cmr_cams)
- **Description:** Contains the functiaonlity for using the Rover's cameras
## TODO Functionality

- **Autonomous Navigation:** [cmr_navigation](./src/cmr_navigation/)
- **GPS Functionality** (inside of cmr_navigation in old main, gps.py)
- **Arm Autonomy**
- **ArUco Tag Detection**
- **Drives Simulation**
- **AruCo Tag Navigation (Spiral Algorithm)**
- **Bird's Eye View**
- **Obstacle Detection:** [cmr_obstacle_data](./src/cmr_obstacle_data/)
- **Drives GUI:** [cmr_param_gui](./src/cmr_param_gui/)
