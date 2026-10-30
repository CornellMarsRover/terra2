---
name: terra-development
description: Implement ordinary Terra2 repository changes from issue investigation through focused tests and review. Use for cross-package work, new features, bug fixes, refactors, launch/config changes, or when no more specialized Terra skill fully owns the task.
---
# Terra2 development

Read `docs/development-workflow.md` for the maintained workflow and
`docs/codebase-map.md` before changing an unfamiliar package.

Start with branch and status. Preserve all pre-existing edits and stop if files
change unexpectedly. Identify the owning package, interface, launch path, tests,
and smallest end-to-end check before editing.

Keep architecture converged: pure logic owns decisions; ROS nodes own transport;
adapters only translate boundaries. Reuse messages/topics and update package,
build, launch, and config metadata with code. Put tunables in parameters/config.
Do not modify unrelated domains, especially arm code during drive/autonomy work.

Test from narrow to broad: focused unit test, affected package build/test, then
system validation only when behavior crosses nodes. Use `terra-testing-ci` for
command selection, `terra-ros-python` or `terra-cpp` for language conventions,
and `terra-gazebo` for driving/autonomy runtime behavior.

Before handoff, run `git diff --check`, review every touched file, document exact
tests and limitations, and leave generated data ignored. Do not commit or push
without user authorization.
