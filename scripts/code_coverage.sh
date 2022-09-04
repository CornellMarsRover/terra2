#!/bin/bash

# Performs code coverage after running tests
# Builds html output in build/cov-output directory
# Assumes the working directory is the terra root directory

lcov --directory . --base-directory . "$@"  --capture -o build/cov.info > /dev/null
lcov -r build/cov.info "/opt/*" -o build/cov.info > /dev/null
lcov -r build/cov.info "*/cmr_msgs/*" -o build/cov.info > /dev/null
lcov -r build/cov.info "*/cmr_*/test/*" -o build/cov.info > /dev/null
# Remove external lib files
lcov -r build/cov.info "*.cxx" -o build/cov.info > /dev/null
lcov -r build/cov.info "*.hxx" -o build/cov.info > /dev/null
lcov -r build/cov.info "*.c" -o build/cov.info > /dev/null
lcov -r build/cov.info "*.h" -o build/cov.info > /dev/null
# Remove c++ standard library
lcov -r build/cov.info "/usr/*" -o build/cov.info > /dev/null
genhtml build/cov.info -o build/cov-output