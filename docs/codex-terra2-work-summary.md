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

