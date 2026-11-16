---
root: false
targets: ["claudecode", "copilot", "antigravity-cli", "cursor"]
description: "Conventional commits and release-please automation — never hand-edit versioning artifacts"
globs: ["**/*"]
---
# Commits and Releases

## Rule: Conventional commits

Every commit message follows [Conventional Commits](https://www.conventionalcommits.org/):

```
type(scope): description
```

Types: `feat`, `fix`, `perf`, `refactor`, `docs`, `test`, `style`, `build`, `ci`, `chore`, `revert`. The scope names the affected area (a workflow, package, or component). The description is a short imperative summary.

## Rule: release-please owns versioning

Applies to repos that have a `release-please-config.json` or `.release-please-manifest.json`. [release-please](https://github.com/googleapis/release-please) derives versions and changelogs from conventional commit types (`feat` → minor, `fix` → patch, `feat!`/`BREAKING CHANGE` → major).

- **Never hand-edit `CHANGELOG.md`** — release-please generates it.
- **Never hand-edit version fields** (`package.json` version, `pyproject.toml` version, `.release-please-manifest.json`) — release-please bumps them.
- Manual edits to these files conflict with the open release PR and break the automation.

## Rule: PR titles follow conventional commits

On squash-merge repos the PR title becomes the commit subject on the default branch, so PR titles must themselves be valid conventional commits. In release-please repos, that subject is what release-please parses.

## Rule: A fix under a hidden commit type is not released

release-please opens a release PR only for commit types that the repo's `release-please-config.json` does not mark `hidden` (`feat`, `fix` and `perf` in the usual config). A dependency or security fix merged as `chore(deps)`, `build` or `ci` therefore stays unreleased until the next releasable commit. Renovate's lock refreshes are `chore(deps)`.

When such a fix has to ship:

- Title the PR `fix(deps): ...` before merging it, or
- Land a commit whose body ends with a `Release-As: X.Y.Z` footer. On a squash merge the footer has to be in the squash commit message. A footer that exists only in a branch commit is lost when the squash message is rewritten.

Do not re-run a failed release job to publish the fix. The job checks out the tag, and the tag's tree predates the fix.
