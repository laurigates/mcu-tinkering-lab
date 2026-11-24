---
reviewed: 2026-11-14
---

# How a PR Lands: Write Each Commit for the Component It Touches

release-please reads the commits that reach `main`, each by its type and by
the paths it touches (`release-please-config.json`). A rebase merge keeps a
PR's commits; a squash replaces them with one commit under the PR title that
touches every path the PR touched. For a PR inside one component that makes no
difference. For one that spans components it decides what gets released.

## Write the commits for the paths they touch

- A commit under a released component's path carries the type that component
  should see. A host-only hook or test under the firmware's path is
  `test(robocar)`, which releases nothing.
- Keep a breaking change in one commit, with `!` and a `BREAKING CHANGE:`
  footer, touching only the component it breaks.
- List which commits touch which component before asking for the merge:
  `git show --format= --name-only <commit>` against the component's path and
  its `exclude-paths`.

The merge-method rules (what turns a PR to squash), the force-push procedure
and the check for a red check inherited from `main` are in the `pr-landing`
skill (`.claude/skills/pr-landing/`).
