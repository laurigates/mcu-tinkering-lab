---
root: false
targets: ["claudecode", "copilot", "antigravity-cli", "cursor"]
description: "GitHub issue and PR metadata standards — assignee, type label, label prefixes, milestone, reviewer, linked issue"
globs: ["**/*"]
---
# GitHub Metadata Hygiene

## Rule: Every issue and PR gets complete metadata

Apply at creation time, and backfill gaps when commenting on or reviewing existing issues/PRs. Tell the user what was backfilled; do not modify silently.

### Issues

| Field | Requirement |
|-------|-------------|
| Assignee | Always set one (default: the repo owner) |
| Type label | Always set at least one: `bug`, `enhancement`, `chore`, `docs`, `security`, `refactor` |
| Milestone | Set one if a relevant milestone exists in the repo |

### Pull Requests

| Field | Requirement |
|-------|-------------|
| Assignee | Always set one |
| Reviewer | Only when the reviewer is not the PR author |
| Type label | Match the linked issue's labels; add a type label if missing |
| Linked issue | Reference the issue being addressed with `Closes #N` (or `Fixes #N`) in the PR body |

**Self-review guard:** requesting a review from the PR's author fails with HTTP 422. Check the author first, and request reviewers in a `gh` call separate from the other metadata, so one rejection does not fail the whole update.

### Labels

Check the repo's labels with `gh label list --limit 500` before applying (the default page of 30 makes a later label read as missing). Do not invent labels.

A repo using the prefix taxonomy keeps its `build:` and `area:` names in `.github/issue-labels.json`; an issue there carries one type label and at least one routing label (`build:`, `area:` or `core:`).

| Label | Says |
|-------|------|
| A type label (listed under Issues) | The kind of change. One per issue or PR |
| `build:<name>` | The build it belongs to in a monorepo: one firmware, app or package among several |
| `area:<name>` | A toolchain or subsystem that cuts across builds |
| `core:<name>` | Repo machinery: CI, quality gates, agent config, build system, shared components, docs upkeep |
| `needs:hardware`, `needs:decision`, `needs:human`, `blocked` | What stops the issue being picked up |

### What stops an issue: who sets and clears it

| Label | Set by | Cleared by |
|-------|--------|------------|
| `needs:hardware` | An agent or the maintainer, when the next step needs physical hardware | The maintainer, or an agent recording the maintainer's result in a comment. Never automation |
| `needs:decision` | An agent stopping on a call that is the maintainer's, with a comment giving the options and a recommendation | Whoever records the decision in a comment |
| `needs:human` | An agent, when the work needs a live session, credentials, local data or a scheduled run | The maintainer, or the agent that lands the work |
| `blocked`, with a `blocked_by` dependency link when the blocker is an issue | Whoever adds the link; by hand when the blocker is a PR or an upstream change | Automation, once every linked blocker is closed; by hand when there is no link |

The dependency link is the record, and `blocked` shows it in the issue list.
