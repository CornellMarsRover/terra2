#!/bin/bash

# Runs all unit tests in the ROS workspace.

pushd "$CMR_ROOT/terra2" &> /dev/null
bash scripts/test_wd.sh
chmod +x $CMR_ROOT/terra2/scripts/llvm-cov-wrapper.sh
bash scripts/code_coverage.sh --gcov-tool $CMR_ROOT/terra2/scripts/llvm-cov-wrapper.sh
popd &>/dev/null
