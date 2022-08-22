#!/bin/bash

# Manually performs static analysis 

pushd "$CMR_ROOT/terra" &> /dev/null
pre-commit run --all-files --hook-stage commit
pre-commit run --all-files --hook-stage push
popd &> /dev/null