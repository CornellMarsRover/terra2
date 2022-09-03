#!/bin/bash

pushd "$CMR_ROOT/terra" &> /dev/null

bash scripts/clean_build_db_wd.sh

popd &> /dev/null