# cmr_cv

This package contains code related to computer vision tasks for the rover.

-How to run it/how to use it
-Basically everything you know about the system
-If someone asks you something, and your answer isn't "it's in the docs", then they need more details
-Links to external resources
-References to links to documentation in the code. Make sure the code documentation is sufficiently detailed to
-What is included in the package (files, classes, modules, etc.)
-Notes for maintainers
-Anything and everything relevant you can of. You may copy and paste these docs for your final reports.

The Aruco Action node assists with the Autonomous Navigaton mission that the
rover competes in during competition. The system overall detects AR tags in the
field and from there takes the tags detected and posts the average Vector3Stamped
position to the AR Tag port in the blackboard. The overall purpose of the class
<ArucoAction> can be read in <aruco_bt_action.hpp> along with the purposes of 
helper functions contained within the class. The specific details on each step
of the process and calculations can be read in <aruco_bt_action.cpp>.

The following are the files, clases, modules, etc. included in this package:
<behaviortree_cpp_v3/action_node.h>
<behaviortree_cpp_v3/bt_factory.h>
<behaviortree_cpp_v3/tree_node.h>
<tf2_ros/buffer.h>
<tf2_ros/transform_listener.h>
<builtin_interfaces/msg/time.hpp>
<geometry_msgs/msg/detail/vector3__struct.hpp>
<geometry_msgs/msg/pose_array.hpp>
<rclcpp/node.hpp>
<rclcpp/rclcpp.hpp>
"cmr_cv/aruco_bt_action.hpp"
"behaviortree_cpp_v3/basic_types.h"
"behaviortree_cpp_v3/utils/shared_library.h"
"nav2_behavior_tree/behavior_tree_engine.hpp"