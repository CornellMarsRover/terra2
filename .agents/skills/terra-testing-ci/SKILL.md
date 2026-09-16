---
name: terra-testing-ci
description: Select, run, and diagnose Terra2 tests, builds, coverage, static analysis, and GitHub Actions. Use when validating changes, adding tests, reproducing CI, or determining the smallest sufficient verification set.
---
# Terra2 testing and CI

Match validation to the changed layer; do not begin with the slowest suite.

1. Run the nearest pytest or C++ test while iterating.
2. For autonomy pure logic, run `./run test`; its line and branch gate is 100%.
3. Build affected ROS packages with `./run build --packages-select PACKAGE`.
4. For C++, build first, then run `bash scripts/check_wd.sh CHANGED_FILES`.
5. Use `bash scripts/test_wd.sh` for full workspace integration in the prepared
   dev environment.
6. Use `./sim check` plus basic/stress demos only for runtime drive/autonomy work.

If `./run test` reports missing `coverage` or `pytest`, use the prepared dev
container or `./sim check`; do not treat an unprepared host as a code failure.

Tests should target contracts, invalid/stale inputs, thresholds, empty data,
timeouts, estop/stop behavior, and regression cases. Do not inflate coverage with
assertion-free execution or tests coupled to private implementation details.

Diagnose GitHub jobs by owner: `Static Analysis` runs changed-file clang format
and tidy; `Build and test` performs ROS build, tests, integration retries, C++
coverage, and autonomy coverage; `Run Gazebo demo` records optional system
evidence. Reproduce the owning command locally and report exact failures.

Never call an incomplete Gazebo run a pass because it produced video. Record all
commands run, outcomes, skipped checks, and environment limitations.
