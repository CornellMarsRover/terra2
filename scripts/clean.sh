#!/bin/bash

# Cleans up the ROS workspace.

pushd "$CMR_ROOT/terra" &> /dev/null
rm -rf build install log
popd &> /dev/null
