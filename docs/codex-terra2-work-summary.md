# Terra2 Codex Work Inventory

Last audited: 2026-09-14

## Scope and evidence

This document inventories the Terra2 work recoverable from local Git history,
branch refs, generated logs, test reports, videos, and the current checkout. It
covers the Codex collaboration from the first drive/Gazebo work through the
current `autonomy_fall2026` branch.

Git cannot prove whether a human or Codex typed a commit when both used the same
identity. The exhaustive ledger therefore uses the identities configured during
this work: `agupt0318 <anantg001@gmail.com>`, the earlier malformed
`Anant <anantg001@gmail>`, and the older local identity
`Anant Gupta <ag987@cornell.edu>`. One clearly associated GitHub-authored commit,
`ad3d169`, is also included. Ambiguous unrelated commits are not claimed.

Current audited state:

- Branch: `autonomy_fall2026`
- HEAD: `b37995e` (`Raise Gazebo obstacle course camera`)
- Tracking branch: `origin/autonomy_fall2026`
- Local status before creating this document: clean and 12 commits ahead
- Origin: `https://github.com/CornellMarsRover/terra2.git`
- Local Git identity: `agupt0318 <anantg001@gmail.com>`
- Primary verified commit ledger: 197 commits across the configured local identities
- Additional clearly associated GitHub commit: `ad3d169`
- Current branch delta for the September effort: 84 files, 2,468 insertions, and 1,049 deletions

## Executive summary

The work moved Terra2 from multiple partially independent drive paths toward one
explicit rover-drive contract. Tele-op and autonomy now publish the same custom
`cmr_msgs/DriveCommand` type into a safety-aware command multiplexer. The mux
selects exactly one source, enforces a timeout and emergency stop, and publishes
the only command accepted by the shared RoverNet drive backend.

The autonomy stack was made more testable by extracting planner, costmap,
mission-state, target-contract, and drive-command decisions into pure Python
modules. Safety checks were added for stale data, malformed messages, blocked
paths, deterministic replanning, target arrival, and costmap inflation escape.

Gazebo work progressed from a simple planar test robot to a mesh-backed rover,
a jointed swerve reference model, repeatable controller/topic validation, and a
physical three-obstacle autonomy harness. The newest harness uses the real depth
point cloud, production costmap, production planner/controller, shared command
mux, and simulation-only boundary adapters. It produced synchronized Gazebo,
RGB, depth, costmap, pose, waypoint, and trajectory evidence.

CI was repaired and expanded to build the ROS workspace, install missing ROS and
Python dependencies, run focused autonomy coverage, isolate flaky integration
tests, and run incremental C++ static analysis. The pure autonomy and mux cores
are held to 100% line and branch coverage by `scripts/test_autonomy.sh`.

## Git and branch management

- Preserved and committed existing work before branch switches.
- Created and used `anant-test-driving` for unified hardware-drive experiments.
- Renamed the drive-testing branch to the requested snake-style name.
- Created `feature/autonomy-obstacle-avoidance-cv` from autonomy work.
- Kept Gazebo reference work separate from the obstacle-avoidance branch rather than merging `gazebo_sim` into it.
- Created `anant/sim-bugfix-2026-07-07` for focused autonomy simulation fixes.
- Created `codex/document-stale-branches` and documented inactive branches for contributors and future agents.
- Created and developed `autonomy_fall2026` as the converged autonomy/drive branch.
- Resolved local branch divergence and non-fast-forward situations without discarding recoverable work.
- Kept commits intentionally small during the September work; the explicit constraint was no more than 50 changed lines per commit.
- Configured the local repository identity as `agupt0318 <anantg001@gmail.com>`.
- Pointed `origin` at the Cornell Mars Rover Terra2 repository.
- Diagnosed why GitHub can display a profile name independently of `git config user.name`: attribution follows the commit email linked to the GitHub account.
- Recovered/started Docker Desktop sufficiently to run repeated ROS 2 and Gazebo sessions in the `terra-sim` container.

Branch tips at this audit:

| Branch | Local tip | Tracking status |
| --- | --- | --- |
| `autonomy_fall2026` | `b37995e` | 12 commits ahead of `origin/autonomy_fall2026` |
| `gazebo_sim` | `ad8667a` | 2 commits behind `origin/gazebo_sim` |
| `feature/autonomy-obstacle-avoidance-cv` | `3ad00c3` | matches its origin ref |
| `anant-test-driving` | `f8f23de` | 5 commits behind its origin ref |
| `anant/sim-bugfix-2026-07-07` | `bd821e0` | matches its origin ref |
| `codex/document-stale-branches` | `c33d814` | matches its origin ref |
| `main` | `7ba1dac` | locally divergent: 6 ahead and 75 behind `origin/main` |

## Initial drive and controller debugging

- Traced UDP controller packets from the laptop to the Jetson-side ROS receiver.
- Verified that controller packets reached `connect_node` and produced ROS drive messages.
- Diagnosed Gazebo failures where commands arrived but joint effort calls failed with `Joint not found`.
- Investigated scoped versus unscoped Gazebo joint names.
- Added clearer drive-command and controller-packet logging.
- Added a simple planar URDF as a known-good Gazebo baseline.
- Added a Sony/DualSense UDP sender for direct end-to-end testing.
- Documented controller sender startup and controller selection.
- Explained ROS package discovery, workspace sourcing, and package listing.
- Documented hardware startup order: controller on laptop, UDP receiver/mux/drive backend on Jetson, then optional autonomy.
- Investigated unreachable Jetson SSH and network connectivity separately from ROS behavior.

## Unified hardware drive node

The `anant-test-driving` work created and reviewed a dedicated rover movement
node based on the existing Moteus GUI behavior.

- Added `zenny_drives_node.py` to consume controller-derived ROS topics and command drive/steer Moteus controllers.
- Preserved the controller mapping for triggers, steering, and emergency stop.
- Added steer-zero capture at startup.
- Added drive and steer watchdog refresh behavior.
- Added session logging for commands and motor state.
- Added documentation in `docs/driving_stack.md`.
- Added a documentation updater script for the driving guide.
- Ignored generated drive-session data in Git.
- Reviewed the node for safety, shutdown, queueing, and command-state bugs.
- Fixed the `asyncio` failure where locks/events were bound to a different event loop.
- Consolidated Moteus work onto a stable worker/event-loop model.
- Fixed emergency-stop release/reset behavior.
- Avoided modifying arm-control behavior during the drive-focused work.

## Explicit drive command contract

A dedicated message was selected instead of forcing the existing controls into a
standard `Twist`, because the rover requires normalized chassis axes and a
separate signed wheel-speed limit.

`cmr_msgs/DriveCommand` now carries:

| Field | Meaning |
| --- | --- |
| `vx` | Normalized rover-frame forward demand in `[-1, 1]` |
| `vy` | Normalized rover-frame lateral demand in `[-1, 1]` |
| `omega` | Normalized rover-frame rotation demand in `[-1, 1]` |
| `speed_rps` | Signed maximum wheel speed in revolutions per second |

Canonical drive topics:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/controller/drives/axes` | controller data | Raw controller drive axes |
| `/controller/drives/buttons` | controller data | Raw controller buttons |
| `/cmd_vel/teleop` | `cmr_msgs/DriveCommand` | Converted manual command |
| `/cmd_vel/autonomy` | `cmr_msgs/DriveCommand` | Autonomous command |
| `/cmd_vel/source` | `std_msgs/String` | Runtime source selection |
| `/cmd_vel/estop` | `std_msgs/Bool` | Emergency-stop state |
| `/cmd_vel` | `cmr_msgs/DriveCommand` | Only selected backend input |

Implemented convergence changes:

- Added the `DriveCommand` message to `cmr_msgs`.
- Added pure `CommandMux` safety logic.
- Added the ROS `drive_command_mux` node.
- Registered the mux as a RoverNet executable.
