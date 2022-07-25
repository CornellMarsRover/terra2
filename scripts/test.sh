#!/bin/bash

# Runs all unit tests in the ROS workspace.

pushd "$CMR_ROOT/terra" &> /dev/null
source install/setup.bash
colcon test; colcon test-result
popd &>/dev/null
