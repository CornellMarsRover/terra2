#!/bin/bash

# Manually perform static analysis
# assumes we are in terra2 directory

if [[ "$@" = "" ]]; then
    changed_files=$(git diff --diff-filter=ACMRT --name-only HEAD~1 | grep -E '\.(hpp|cpp|inl)$' | grep -v -e ".*/external/.*")
else
    changed_files=$(echo "$@" | tr ' ' '\n' | grep -e ".hpp$" -e ".cpp$" -e ".inl$" | grep -v -e ".*/external/.*")
fi
if [[ "$changed_files" = "" ]]; then
    echo "No changed C++ files to check"
    exit 0
else
    echo "Checking files: $changed_files"
fi

clang-format --dry-run --Werror $changed_files
if [ $? -ne 0 ]; then
    echo "clang-format contains errors"
    exit 1
fi
echo "Formatting done"

time clang-tidy -p build/compile_commands.json --config-file=.clang-tidy \
  --header-filter="^cmr_.*pp$" $changed_files
if [ $? -ne 0 ]; then
    echo "clang-tidy contains errors"
    exit 1
fi