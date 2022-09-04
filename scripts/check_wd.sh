#!/bin/bash

# Manually perform static analysis
# assumes we are in terra directory

format_files=$(find ./src -type f \( -name "*.cpp" -or -name "*.hpp" -or -name "*.inl" \) -and -not -path "*/external/*")
clang-format --dry-run --Werror $format_files
echo "Formatting done"
time find ./src -type f \( -name "*.cpp" -or -name "*.inl" \) -and -not -path "*/external/*" \
 | xargs clang-tidy -p build/compile_commands.json --config-file=.clang-tidy \
  --header-filter="^cmr_.*pp$"