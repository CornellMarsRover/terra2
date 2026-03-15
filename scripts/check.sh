#!/bin/bash

# Manually performs static analysis 

pushd "$CMR_ROOT/terra2" &> /dev/null
bash scripts/check_wd.sh "$@"
popd &> /dev/null