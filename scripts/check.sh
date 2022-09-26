#!/bin/bash

# Manually performs static analysis 

pushd "$CMR_ROOT/terra" &> /dev/null
bash scripts/check_wd.sh "$@"
popd &> /dev/null