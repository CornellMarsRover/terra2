#!/bin/bash

# Manually perform static analysis
# assumes we are in terra directory

format_files=$(find ./src -type f \( -name "*.cpp" -or -name "*.hpp" -or -name "*.inl" \) -and -not -path "*/external/*")
clang-format --dry-run --Werror $format_files
if [ $? -ne 0 ]; then
    echo "clang-format contains errors"
    exit 1
fi
echo "Formatting done"
if [[ "$@" = "" ]]; then
    changed_files=$(git diff --name-only HEAD~1 | grep -e ".*cpp" -e ".*inl" | grep -v -e "*/external/*")
else
    changed_files=$(grep -e "*.cpp" -e "*.inl" -e "*.hpp" "$@" | grep -v -e "*/external/*")
fi
if [[ "$changed_files" = "" ]]; then
    echo "No files to check"
    exit 0
else
    echo "Checking files: $changed_files"
fi
time clang-tidy -p build/compile_commands.json --config-file=.clang-tidy \
  --header-filter="^cmr_.*pp$" $changed_files
if [ $? -ne 0 ]; then
    echo "clang-tidy contains errors"
    exit 1
fi