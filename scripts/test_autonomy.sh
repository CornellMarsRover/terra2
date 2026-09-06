#!/bin/bash
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"
export PYTHONPATH="$ROOT/src/autonomous_navigation${PYTHONPATH:+:$PYTHONPATH}"

python3 -m coverage erase
PURE_MODULES="autonomous_navigation.drive_command,autonomous_navigation.planner_core,autonomous_navigation.costmap_core,autonomous_navigation.state_machine_core"
python3 -m coverage run --branch --source="$PURE_MODULES" \
  -m pytest -q -p no:cacheprovider src/autonomous_navigation/test
python3 -m coverage report --show-missing --fail-under=100
mkdir -p build
python3 -m coverage xml -o build/autonomy-coverage.xml
