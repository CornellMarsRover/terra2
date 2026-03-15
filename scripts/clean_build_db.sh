#!/bin/bash

pushd "$CMR_ROOT/terra2" &> /dev/null

bash scripts/clean_build_db_wd.sh

popd &> /dev/null