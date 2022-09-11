#!/bin/bash

# Runs a build on the ROS workspace.

pushd "$CMR_ROOT/terra" &>/dev/null
source install/setup.bash
CC=clang CXX=clang++ colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"
retval=$?
if [[ $retval -ne 0 ]];
then
  echo "CMR_EXIT_STATUS=$retval"
  exit $retval
fi

popd &>/dev/null
