#!/bin/bash

# Runs all unit tests in the ROS workspace.
# Assumes the working directory is the terra root directory
# Requires TERRA_DIR to be defined

source install/setup.bash
find . -name "*.gcda" -delete
colcon test; colcon test-result
chmod +x $TERRA_DIR/scripts/llvm-cov-wrapper.sh
lcov --directory . --base-directory . --gcov-tool \
    $TERRA_DIR/scripts/llvm-cov-wrapper.sh  --capture -o build/cov.info
lcov -r build/cov.info "/opt/*" -o build/cov.info
lcov -r build/cov.info "/usr/*" -o build/cov.info
