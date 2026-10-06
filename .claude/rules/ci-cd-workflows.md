---
paths:
  - '**/.github/workflows/**'
---
# CI/CD Workflow Conventions

## Rule: Call org reusable workflows — do not duplicate CI logic inline

CI/CD logic is centralized in [`laurigates/.github`](https://github.com/laurigates/.github). Application repos call the reusable workflows instead of defining build/release/review logic inline:

```yaml
uses: laurigates/.github/.github/workflows/reusable-<name>.yml@main
```

Key reusable workflows: container build/release (build-once/promote via GHCR), release-please, Claude PR review, auto-fix, conventional commit enforcement, sync-ai-rules.

## Rule: Open PRs and push with a GitHub App token, not `GITHUB_TOKEN`

A workflow that opens a pull request or pushes a commit uses a GitHub App token. `GITHUB_TOKEN` is refused outright when a repo does not allow Actions to create pull requests (GitHub's default), and any PR or push it does make triggers no further workflows, so CI never runs on it. Repos flagged `release_please = true` in `laurigates/gitops` receive the App credentials (`vars.RELEASE_PLEASE_CLIENT_ID`, `secrets.RELEASE_PLEASE_PRIVATE_KEY`); a repo without them needs that flag first. Pass them to reusable workflows that take `app-id` / `APP_PRIVATE_KEY`.

## Rule: Pin third-party actions to a full commit SHA

Every third-party action is pinned to its full 40-character commit SHA with a trailing version comment:

```yaml
uses: actions/checkout@11bd71901bbe5b1630ceea73d27597364c9af683 # v4.2.2
```

Never pin to a mutable tag (`@v4`) or branch (`@main`) for third-party actions. Reusable workflows from `laurigates/.github` are the one exception — they are called with `@main`.

## Rule: Declare minimal workflow-level permissions

Every workflow declares an explicit `permissions:` block at workflow level, granting only what the jobs need:

```yaml
permissions:
  contents: read
```

Elevate individual scopes (`contents: write`, `pull-requests: write`) only when a step requires it.

## Rule: Use concurrency groups

Workflows that can race against themselves (release, deploy, PR checks) declare a concurrency group so superseded runs are cancelled or queued:

```yaml
concurrency:
  group: <workflow-name>-${{ github.ref }}
  cancel-in-progress: true
```
