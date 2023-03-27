# cmr_description

@brief Contains the URDF files and other hardware configurations for the different
subsystems of the rover.

These packages also contain the `ros2_control` URDF parameters as described in 
[cmr_control](@ref cmr_control/README.md).

Within this package are 3 sub-packages for each subsytem of the rover. 
Each sub-package contains a `urdf` folder which stores the URDF files, a `ros2_control`
folder to store the files containing macros for initializing ROS2 Control components,
a `world` folder for storing `.sdf` (which are the same as `.world`) files which
contain the world used in gazebo simulations, and `config` folders which contain
the saved RViz config files to restore the view of RViz (panels, added objects, etc.)
when it was saved.