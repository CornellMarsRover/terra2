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
- Published autonomy commands on `/cmd_vel/autonomy`.
- Converted tele-op commands to `/cmd_vel/teleop` without changing controller mappings.
- Made `/cmd_vel` the selected output only.
- Added launch-time source selection for tele-op and autonomy modes.
- Added runtime source switching via `/cmd_vel/source`.
- Added a 0.5-second command timeout.
- Added emergency-stop latching behavior at the mux boundary.
- Cleared buffered commands on source changes and estop transitions.
- Rejected non-finite commands.
- Published zeros on timeout or estop.
- Removed hidden drive-priority arbitration and bypass callbacks.
- Routed keyboard control and the compatibility swerve path through the mux.
- Made the shared RoverNet backend consume only the selected command.
- Preserved one shared Moteus/kinematics backend for both tele-op and autonomy.
- Renamed ambiguous raw and selected drive topics throughout code, launch files, summaries, and architecture maps.
- Documented that legacy arm utilities using `/cmd_vel` as `Twist` must be remapped before co-launching.

## Autonomy control convergence

- Added normalized autonomy drive conversion in `drive_command.py`.
- Routed the autonomy controller into the same `DriveCommand` and mux interface as tele-op.
- Launched the same RoverNet drive backend for autonomy hardware mode.
- Used the same drive scaling in simulation.
- Added mapping tests for forward, heading correction, point turn, stop, clamping, and invalid values.
- Verified tele-op and autonomy with identical semantic command sequences.
- Confirmed both sources produced the same selected commands at the shared lower-level interface.
- Measured a 0.0498 m endpoint difference and 0.00664 rad yaw difference in the accepted non-video convergence run.
- Measured a 0.0787 m endpoint difference and 0.0257 rad yaw difference in the recorded convergence run.
- Verified manual override, autonomy resume, lateral motion, and turn arbitration in Gazebo.

## Autonomy modularization

Pure, ROS-independent logic was extracted so decisions can be tested without a
running graph or hardware.

- Added `planner_core.py` for target parsing, path progression, density checks, simplification, neighbor costs, segment costs, traversability, deterministic goal selection, and invalidation confirmation.
- Added `costmap_core.py` for point projection, obstacle scoring, and cost decay.
- Added `state_machine_core.py` for waypoint selection, search behavior, mission-object transitions, coordinate conversion, and arrival decisions.
- Added `target_contract.py` for perception/mission target matching.
- Added `drive_command.py` for normalized autonomy command mapping and freshness checks.
- Rewired ROS nodes to call these pure modules.
- Removed duplicated node-bound helper implementations after extraction.
- Kept ROS subscriptions, publications, and launch behavior in adapter nodes.

## Autonomy safety and correctness fixes

- Added deterministic nearest-clear-goal selection.
- Added deterministic local replanning behavior.
- Required consecutive blocked observations before invalidating a path.
- Rejected unsafe above-threshold planner transitions.
- Held unsafe current paths while replanning.
- Added risk-reducing transitions so a rover already inside an inflated region can escape.
- Kept obstacle replanning mobile while preserving collision-safe smoothing.
- Distinguished actual occupied cells from soft costmap inflation when deciding whether to stop.
- Prevented the rover from deadlocking inside a newly observed safety-inflation band.
- Validated smoothed paths against inflated collision costs and retained dense paths when smoothing cut through obstacles.
- Used a wider cost sampling/inflation gap for physical rover clearance.
- Added target parsing checks for malformed, short, non-finite, and optional-yaw payloads.
- Added costmap payload validation.
- Added stale pose/waypoint fail-closed behavior.
- Required GPS initialization before publishing real autonomy pose.
- Published an initialized filtered simulation pose.
- Defined safe waypoint arrival and commanded a stop at reached waypoints.
- Added true target-bearing arrival diagnostics.
- Wrapped simulated yaw deltas correctly.
- Fixed localization topic, yaw-unit, and origin-capture behavior.
- Fixed asymmetric neighbor sampling in planner cost checks.
- Fixed costmap field-of-view failures.
- Fixed costmap game-object clearing dimensions.
- Stopped unbounded free-space growth in the costmap.
- Reset search state and detector targets when advancing waypoints.
- Suppressed stale detector positions briefly after waypoint transitions.
- Removed an unused `pyubx2` import that prevented state-machine startup.
- Made planner visualization optional when Rerun is unavailable.
- Used simulated time consistently for planner/controller nodes in simulation.

## Obstacle avoidance computer vision and planning

The initial feature branch created a modular obstacle-avoidance pipeline and was
iterated repeatedly against generated video and logs.

- Created `feature/autonomy-obstacle-avoidance-cv` from autonomy work.
- Inspected the existing state machine, costmap, planner, controller, messages, configs, and launches.
- Used Gazebo work as reference without merging the `gazebo_sim` branch into the feature branch.
- Added simulated vision obstacle detection and pure visibility logic.
- Added obstacle-guard logic and a pure obstacle-guard core.
- Added simulation point-cloud generation so obstacles enter the shared costmap path.
- Added odometry-to-autonomy-pose and simulation-drive adapters.
- Added repeatable single- and multi-obstacle launch scenarios.
- Added physical block models and deterministic spawning.
- Added goal publication and demo telemetry logging.
- Added map rendering that showed rover pose, goal, obstacles, visible obstacles, local target, next waypoint, and commands.
- Reused the production costmap, local planner, controller, and autonomy messages instead of retaining a parallel simulation-only avoidance controller.
- Tightened synthetic obstacle visibility at startup.
- Improved path following, replanning, arrival, respawn, and stall recovery over multiple iterations.
- Fixed the single-obstacle launch after the multi-obstacle work.
- Documented simulation execution, architecture, copied assets, limitations, and next steps.

## Gazebo rover models and simulation

