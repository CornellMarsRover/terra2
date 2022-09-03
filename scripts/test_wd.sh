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
lcov -r build/cov.info "*/cmr_msgs/*" -o build/cov.info
lcov -r build/cov.info "*/cmr_*/test/*" -o build/cov.info
# Remove external lib files
lcov -r build/cov.info "*.cxx" -o build/cov.info
lcov -r build/cov.info "*.hxx" -o build/cov.info
lcov -r build/cov.info "*.c" -o build/cov.info
lcov -r build/cov.info "*.h" -o build/cov.info
# Remove c++ standard library
lcov -r build/cov.info "/usr/*" -o build/cov.info
