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
