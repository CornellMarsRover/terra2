# cmr_bs

@brief The package for the baseStation subsystem containing joystick and drivingController nodes 

This package primarily provides USB controller usage, specifically for the arm joystick controller and the drives Logitech controller. It provides a way to read and output data to the arm and drives. 

The controllers are configured using [.toml] files. Currently, there are two toml files: one for the arm and one for the drives. 
* `armcontroller.toml`
* `drivescontroller.toml`

Each .toml file contains node specific configurations under the parameter [node]. Each files contains a device path [device], a controller type [type] (armcontroller or drives controller), and the maximum speed for the respective controller [arm_max_speed] or [max_speed]. 

Below is the code for the publishers for the arm and drives controllers respectively

    ```C++

    m_joystick_pub =
        this->create_lifecycle_publisher<cmr_msgs::msg::JoystickReading>(
            "js_input", buffer_size);

    m_drives_pub =
        this->create_lifecycle_publisher<geometry_msgs::msg::TwistStamped>(
            "/drives_controller/cmd_vel", buffer_size);

    ```
ARM: 
* Publisher listed as m_joystick_pub 
* Publishes to topic "js_input" 
* Arm nodes subscribe to "js_input" to receive input

DRIVES: 
* Publisher listed as m_drives_pub 
* Publishes to topic "/drives_controller/cmd_vel" 
* ROS automatically takes values in drives topic to     incorporate drives functionality

## Using the USB Device Controllers 

1. If the USB device has not been added to the Docker image, then follow the link below that explains how to add a USB device to Docker. 
https://cmrdocs.gitbook.io/software/phobos-cli/usage/device-management

2. After plugging in the USB device to a computer with Ubuntu, open up two Ubuntu shells. Run `phobos terra build` to build the latest version of phobos. Then, run `ros2 launch (packagename(cmr_bs)) (Launch file name(`launch file name`))` in one shell. In the other shell run, `ros2 service call /rover/lifecycle/activate cmr_msgs/ActivateNode "node_name: '`device name`'"`. The ros2 launch command will run a launch file that uses the node being activated. The ros2 service command will actually activate the node so it can be used. 

For example, if I wanted to activate the joystick with its launch file titles bs.launch.py, the command I would run are as follows:

`phobos terra build`

`ros2 launch (packagename(cmr_bs)) (Launch file name(bs.launch.py))`

`ros2 service call /rover/lifecycle/activate cmr_msgs/ActivateNode "node_name: 'jscontroller'"`

3. Any output expected to be printed out will be printed in the terminal with the launch file run. Use the controller, and you should be then able to see the output in the launch file shell!



