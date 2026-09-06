#!/bin/bash
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"
export PYTHONPATH="$ROOT/src/autonomous_navigation${PYTHONPATH:+:$PYTHONPATH}"

python3 -m coverage erase
python3 -m coverage run --branch --source=autonomous_navigation.drive_command \
  -m pytest -q -p no:cacheprovider src/autonomous_navigation/test
python3 -m coverage report --show-missing --fail-under=100
mkdir -p build
python3 -m coverage xml -o build/autonomy-coverage.xml
