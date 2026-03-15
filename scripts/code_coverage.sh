#!/bin/bash

# Performs code coverage after running tests
# Builds html output in build/cov-output directory
# Assumes the working directory is the terra2 root directory

# pushd "$CMR_ROOT/terra2" &> /dev/null

lcov --directory . --base-directory . "$@"  --capture -o build/cov.info > /dev/null
lcov -r build/cov.info "/opt/*" -o build/cov.info > /dev/null
lcov -r build/cov.info "*/cmr_msgs/*" -o build/cov.info > /dev/null
lcov -r build/cov.info "*/cmr_*/test/*" -o build/cov.info > /dev/null
# Remove external lib files
lcov -r build/cov.info "*/external/*" -o build/cov.info > /dev/null
# Remove c++ standard library
lcov -r build/cov.info "/usr/*" -o build/cov.info > /dev/null
genhtml build/cov.info -o build/cov-output

# popd &>/dev/null