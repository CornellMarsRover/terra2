# cmr_arm

@brief This package includes software to control the rover arm with both forward and inverse kinematics.

MoveIt 2 in ROS2 is used for arm functionality. Documentation and overview be found [here](https://moveit.picknik.ai/humble/index.html#).

Software to control the arm is handled by the IK and Joystick nodes:
* `cmr::InverseKinematics`
* `cmr::JoystickDirectControl`

## Inverse Kinematics
IK is given position, reference frame, and topic through `cmr::InverseKinematics::update_arm_position`. This method takes PoseStamped messages. These messages contain header information such as reference frames, which enables arm movement from any reference frame. 
- Reference frames on the arm include: 'base', 'base_rotate', 'shoulder', 'elbow', 'second_rotate' 'third_tilt', 'third_rotate', 'end_effector'. These are listed `cmr_control/arm_controllers.yaml`.
- MoveIt2 covers the IK functionality through planning and execution. It will invoke MoveIt2 to create a plan for where the arm should be and set Pose target.


## Joystick Control 
The joystick is used for direct control, by selecting a joint and the joystick will control it. Joints are one dimensional angles and have three degrees of freedom, with each freedom matched to different actions of the physical joystick. Joystick messages are passed into and handle by `cmr::JoystickDirectControl::update_arm_position`.
- Listens to events on joystick, and when it detects them it directly sends messages links are pivot points that connect joints
- And the motors passed using segments 

## Other Folders
The launch folder contains the launch files for ROS to launch the nodes, for each toml file in the config folder it will read paramaters and create a node based on the parameters. THe URDFs essentially contain the data on the arm, including names of different components used for the code, and links.
