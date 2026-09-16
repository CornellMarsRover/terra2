# Development workflow

This is the normal software-development path for Terra2. Use the specialized
Gazebo guide only when a change affects driving, autonomy, perception, planning,
or simulation behavior.

## 1. Start safely

Work in the ROS 2 Humble dev container or on the Jetson for hardware-only work.
From the repository root:

```bash
git status --short
git branch --show-current
./run setup       # first clone or changed dependencies
./run build       # first clone, branch switch, or ROS interface/package change
```

Never discard a dirty worktree. Separate pre-existing edits from the files you
intend to change. Do not develop directly on `main`; the pre-commit hooks reject
commits and pushes to it.

Before editing, identify:

- the package that owns the behavior;
- its inputs, outputs, message types, and launch entry point;
- the nearest existing tests;
- whether the behavior can be extracted into ROS-independent logic; and
- the smallest end-to-end check that proves the change.

Use `docs/codebase-map.md` and `docs/ros-structure.md` instead of guessing package
or topic ownership.

## 2. Implement in the owning layer

Prefer a small change in the existing architecture over a parallel path.

- Put deterministic decisions and transformations in pure modules.
- Keep ROS nodes thin: parameters, subscriptions, publishers, timers, logging,
  and translation to/from pure functions.
- Reuse existing messages and topics unless the contract truly changes.
- Update `package.xml`, `setup.py`, CMake, launch files, and configs together when
  adding a dependency or executable.
- Put tunable runtime values in ROS parameters or config, not scattered literals.
- Reject non-finite or stale motion inputs and fail to a zero/stop command.
- Preserve tele-op, estop, timeout, and watchdog behavior when changing autonomy.
- Keep hardware and simulation adapters at explicit boundaries.
- Do not modify arm code during a drive/autonomy task.

Match surrounding naming and layout. Use ASCII unless an existing file requires
