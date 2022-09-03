#!/bin/bash

# Manually performs static analysis 

pushd "$CMR_ROOT/terra" &> /dev/null
format_files=$(find ./src -type f -name "*.cpp" -or -name "*.hpp" -or -name "*.inl")
clang-format --dry-run --Werror $format_files
echo "Formatting done"
time find ./src -type f -name "*.cpp" -or -name "*.inl" \
 | xargs clang-tidy -p build/compile_commands.json --config-file=.clang-tidy \
  --header-filter="^cmr_.*pp$"
popd &> /dev/null