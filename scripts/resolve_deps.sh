#!/bin/bash

# Run 'rosdep install' on the ROS workspace

pushd "$CMR_ROOT/terra/" &> /dev/null
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
popd &> /dev/null

