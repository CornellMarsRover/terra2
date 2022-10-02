#!/bin/bash

# Runs all unit tests in the ROS workspace.

pushd "$CMR_ROOT/terra" &> /dev/null
source install/setup.bash
find . -name "*.gcda" -delete
colcon test --executor sequential
# I gave up (for now?) trying to figure out why the tests don't work in parallel
# So we just run them sequentially for now. This should be robust enough because
# I don't forsee us running two complete different software systems on the same
# hardware in competition.
# It seems that something is occurring that is invalidating the ROS context in
# the other test.
colcon test-result --verbose
test_result_code=$?
if [ $test_result_code != 0 ]; then 
    exit $test_result_code
fi
chmod +x $CMR_ROOT/terra/scripts/llvm-cov-wrapper.sh
bash scripts/code_coverage.sh --gcov-tool $CMR_ROOT/terra/scripts/llvm-cov-wrapper.sh
popd &>/dev/null
