---
name: terra-ros-python
description: Create or modify Python ROS 2 nodes and packages in Terra2. Use for rclpy callbacks, topics, parameters, timers, Python launch integration, package metadata, and extracting testable ROS-independent logic.
---
# Terra2 Python ROS code

Inspect the package's `package.xml`, `setup.py`, launch files, nearby node, and
tests before changing an interface. Confirm topic names and message types from
code; do not infer them from filenames or old docs.

Keep `Node` classes thin. Move deterministic state transitions, planning,
validation, conversions, and safety decisions into importable pure modules. Pure
modules must not require a running ROS context and should validate finite values,
units, ranges, stale timestamps, and empty inputs.

Declare runtime tunables as parameters. Use timers for periodic work and avoid
blocking callbacks. Preserve QoS choices for sensor streams. Log state changes or
throttled diagnostics rather than every callback. Motion publishers must fail
safe to zero/stop on stale, invalid, missing, estopped, or unselected input.

When adding an executable, update `setup.py` console scripts, `package.xml`
dependencies, installed configs/launch files, and callers together. Shutdown must
tolerate Ctrl-C and an already-shutdown context without obscuring the real error.

Add pure-module pytest coverage first, then a node/integration test only for ROS
wiring. Build the affected package with `colcon` or `./run build
--packages-select PACKAGE`; use `terra-testing-ci` for broader gates.
