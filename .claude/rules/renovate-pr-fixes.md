# Fixing a Renovate PR: Merge It Before Anything That Touches Its Files

`auto-resolve-conflicts.yml` (the reusable `reusable-auto-resolve-conflicts.yml`)
runs on every push to `main`. It does not try to resolve a conflicting PR whose
branch matches `renovate/*`, `dependabot/*`, `release-please--branches--*`,
`chore/update-version-tracking-*` or `chore/claude-config-sync`: it closes the
PR and deletes the branch, so the bot can recreate it. Any fix commit pushed to
that branch is deleted with it, and the recreated PR fails the same way.

On 2026-10-31 this closed #870 (a torchvision bump pushed to pair with torch
2.13.0) and #888 (a sphinx hold and a Dockerfile revert), seconds after
unrelated merges made them conflict. Both fixes had to be republished, as #891
and #892.

- **Merge a fixed Renovate PR first.** Before merging any other PR, check
  whether it touches the Renovate PR's files (lockfiles and `pyproject.toml`
  most often). If it does, merge the Renovate PR first.
- **If it has to wait, move the fix off the bot branch.** Cherry-pick the
  bot's commit and the fix onto a branch outside those patterns (for example
  `chore/deps-<topic>`), open that as the PR, and let the bot PR close.
- **Hold the failing bump in `renovate.json`.** Otherwise Renovate proposes it
  again on the next run. Add a `packageRules` entry with `allowedVersions` or
  `enabled: false` and a `description` saying why, as #893 did for sphinx and
  the ESP-IDF dev container.
