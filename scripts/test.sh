#!/bin/bash

# Runs all unit tests in the ROS workspace.

pushd "$CMR_ROOT/terra" &> /dev/null
source install/setup.bash
find . -name "*.gcda" -delete
colcon test --executor sequential
colcon test-result --verbose
test_result_code=$?
if [ $test_result_code != 0 ]; then 
    exit $test_result_code
fi
chmod +x $CMR_ROOT/terra/scripts/llvm-cov-wrapper.sh
bash scripts/code_coverage.sh --gcov-tool $CMR_ROOT/terra/scripts/llvm-cov-wrapper.sh
popd &>/dev/null
