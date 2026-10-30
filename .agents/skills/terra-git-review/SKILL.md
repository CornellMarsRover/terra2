---
name: terra-git-review
description: Prepare, review, commit, or hand off Terra2 changes safely. Use for dirty worktrees, branch work, commit planning, pull-request preparation, code review, or reconciling local changes without losing another developer's work.
---
# Terra2 Git and review workflow

Start with `git status --short`, current branch, remotes, and the relevant diff.
Treat every pre-existing edit as owned by someone else until proven otherwise.
Never reset, discard, overwrite, stash away, amend, rebase, merge, commit, or push
without the user's request and the operation's consequences being understood.
Stop if unexpected concurrent changes appear.

Do not commit on `main`; pre-commit blocks it. The working convention is at most
50 changed lines per commit. Split by coherent behavior: core logic, tests,
adapter/wiring, launch/config, and docs. Each commit should build on the previous
one and have an imperative message.

Review findings before summaries. Prioritize safety failures, behavioral
regressions, stale inputs, units/frames/signs, topic compatibility, missing
package metadata, untested branches, generated files, and secrets. Cite exact
files and lines.

Before an authorized commit, run the narrow relevant tests and `git diff
--check`, inspect staged content, and ensure unrelated files are excluded. Before
an authorized push, confirm the destination branch and report tests, known gaps,
and whether CI-only checks remain. Use the handoff template in
`docs/development-workflow.md` for unfinished work.
