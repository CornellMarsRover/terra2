#!/bin/bash

# This is hacky to hard code the path to the wrapper for the CI
bash scripts/code_coverage.sh --gcov-tool /__w/terra/terra/scripts/llvm-cov-wrapper.sh