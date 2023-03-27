# cmr_control

@brief Code related to hardware control of the rover.

An Overview of ROS 2 Control can be found [here](https://control.ros.org/master/doc/getting_started/getting_started.html#).

This package primarily provides implementations of Controllers and Hardware Interfaces.

A Hardware Interface is essentially what is used to implement communication with the
physical hardware. It provides a way to read and/or write data to sensors and actuators.
We can either have a Sensor, Actuator, or System Interface (the latter is essentially
a combination of the two formers). An example of this would be the 
`cmr_control::DrivesSystemHardware`.


A Controller (or Broadcaster) is essentially what is used to implement the topic interface we
use from other nodes to communicate with the hardware interface. It is configured
from YAML files. An example of this would be the `cmr_control::AstroSensorBroadcaster`.

We currently have:
* `cmr_control::AstroSensorHardware`
* `semantic_components::AstroSensor`
* `cmr_control::AstroSensorBroadcaster`
* `cmr_control::AstroMotor`
* `cmr_control::DrivesSystemHardware`
* `cmr_control::ArmSystemHardware`
* `cmr_control::MockArmSystemHardware` (unnecessary, see [ROS2 Control Mocks](https://control.ros.org/galactic/doc/api/classfake__components_1_1GenericSystem.html) for mocking)

The Controllers are configured via a YAML file which contains parameters that link up
a controller and hardware interface. A hardware interface must be "connected"
to a joint or other object within the URDF via the `<ros2_control>` tag

## Steps to adding a new Hardware Component:

1. Create a Hardware Interface that subtypes from the correct base class. 
Skip this step if you will reuse an existing one. Make sure the interface
uses `PLUGINLIB_EXPORT_CLASS` macro to register itself with
[pluginlib](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
2. Make sure the Hardware Interface is added to `hardware.xml` and the `CMakeLists.txt`
3. Create a Xacro macro to instantiate the ROS2 control components. This will go in the
`ros2_control` folder in the `<...>_description` folder along with the `urdf` folder and other 
description directories. Here's an example:
*Note*: The usage of a macro isn't strictly necessary

    ```XML

    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro">
        <xacro:macro name="astro_motor_ros2_control" params="name joint">
            <ros2_control name="${name}" type="actuator">
                <hardware>
                    <plugin>cmr_control/AstroMotor</plugin>
                </hardware>
                <joint name="${joint}">
                    <command_interface name="position" />
                    <command_interface name="velocity" />
                    <state_interface name="position"/>
                    <state_interface name="velocity"/>
                </joint>
            </ros2_control>
        </xacro:macro>
    </robot>

    ```

4. Call the XACRO macro from the main URDF file. Here's an example:

    ```XML

        <xacro:include filename="$(find cmr_drives_description)/ros2_control/astro_motor.ros2_control.xacro" />

        <xacro:astro_motor_ros2_control name="test_astro_motor" joint="test_astro_joint" />

    ```

5. Add the hardware interface to a controllers yaml config file to indicate which
controller will provide the topic interface for your hardware. Example:

    ```YAML
    controller_manager:
        ros__parameters:
            update_rate: 10 #Hz

            joint_state_broadcaster:
                # Doc -> https://github.com/ros-controls/ros2_controllers/blob/master/joint_state_broadcaster/doc/userdoc.rst
                type: joint_state_broadcaster/JointStateBroadcaster

            astro_motor_pos_controller:
                type: position_controllers/JointGroupPositionController

            astro_motor_vel_controller:
                type: velocity_controllers/JointGroupVelocityController

    astro_motor_pos_controller:
        ros__parameters:
            joints:
            - test_astro_joint

    astro_motor_vel_controller:
        ros__parameters:
            joints:
            - test_astro_joint
    ```

6. Launch your controller for the hardware interface from the launch file. Example: 

    ```Python

        astro_motor_controllers = PathJoinSubstitution(
            [FindPackageShare("cmr_control"), "config", "astro_motor_controllers.yaml"]
        )

        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers, astro_broadcasters, astro_motor_controllers],
            output="both",
        )

        # ...

        astro_motor_pos_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["astro_motor_pos_controller", "-c", "/controller_manager"],
            condition=IfCondition(PythonExpression(
                [astro_motor_mode, " == 'position'"]
            ))
        )

    ```

You can find further information [here](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html) for adding a HW interface
and [here](https://control.ros.org/foxy/doc/ros2_controllers/doc/writing_new_controller.html)
for adding a controller.

## Some notes about HW Interfaces

Each HW interface typically has state interfaces and command interfaces. The HW interface
typically has member variables for each state and command interface it provides.
The command interface value is set by the controller when it receives commands
on the topics is subscribes to. The state interfaces value is what the hardware interface
reads and writes. They should reflect the current state (ex. velocity, position) of the
motor or sensor the HW interface corresponds to.

The method `prepare_command_mode_switch` is called with a vector of interface names being passed
to a HW interface. Note that `prepare_command_mode_switch` may be called with a set of
command interfaces not relevant to the HW interface, and when that occurs the call should
be ignored. The vector names represents the new command modes being requested for each
joint under the HW interface's control. Each string in the vector
will be of the form `JOINT_NAME/NEW_COMMAND_MODE_NAME`.
