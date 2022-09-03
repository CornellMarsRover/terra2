#!/bin/bash

# Runs all unit tests in the ROS workspace.

pushd "$CMR_ROOT/terra" &> /dev/null
export TERRA_DIR="$CMR_ROOT/terra"
bash scripts/test_wd.sh
genhtml build/cov.info -o build/cov-output
popd &>/dev/null
