---
paths:
  - '**/*'
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

**Self-review guard:** GitHub rejects a review request for the PR's own author with HTTP 422. Check the author first, and request reviewers in a separate `gh` call from the other metadata so one rejection does not fail the whole update.

### Label conventions

| Label | When |
|-------|------|
| `bug` | Defect or broken behavior |
| `enhancement` | New feature or improvement to existing feature |
| `chore` | Maintenance, dependency updates, refactoring |
| `docs` | Documentation changes |
| `security` | Security-related fixes or improvements |
| `refactor` | Code restructure with no behavior change |

Check available labels with `gh label list` before applying — do not invent labels that don't exist in the repo.
