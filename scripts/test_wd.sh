#!/bin/bash

# This script is meant to be run from the root of the repository
# runs all tests


source install/setup.bash
find . -name "*.gcda" -delete
colcon test --executor sequential --pytest-args " -k SKIP_ALL_TESTS" --packages-skip-regex ros2_aruco
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

pytest --ignore=src/cmr_aruco --ignore=src/cmr_py_test
test_result_code=$?
if [[ $test_result_code != 0 && $test_result_code != 5 ]]; then 
    exit $test_result_code
fi

# I have no clue why, but the tests in cmr_py_test don't always work when run
# with pytest together, but they do work when run individually.
find ./src/cmr_py_test -name "*_test.py" | while read file; do
    pytest $file -v --tb=long
    test_result_code=$?
    if [[ $test_result_code != 0 && $test_result_code != 5 ]]; then 
        exit $test_result_code
    fi
done