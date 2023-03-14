# cmr_cv

@brief This package contains code related to computer vision tasks for the rover.

The Aruco Action node assists with the Autonomous Navigaton mission that the
rover competes in during competition. The system overall detects AR tags in the
field and from there takes the tags detected and posts the average Vector3Stamped
position to the AR Tag port in the blackboard. The overall purpose of the class
`ArucoAction` can be read in `aruco_bt_action.hpp` along with the purposes of 
helper functions contained within the class. The specific details on each step
of the process and calculations can be read in `aruco_bt_action.cpp`.
