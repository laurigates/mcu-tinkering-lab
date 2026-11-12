---
reviewed: 2026-11-01
---

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

## Redoing a lock-file-maintenance PR locally

Renovate's `lockFileMaintenance` PR is `uv lock --upgrade` in each uv project
whose `uv.lock` it lists. It is enabled once for every laurigates repo, in the
self-hosted runner's global config (`laurigates/gitops` `renovate-config.js`,
monthly on the 1st), not in this repo's `renovate.json`; do not enable it here
as well. When it fails, the Renovate CLI cannot redo it here:
`--platform=local` only reports lookups, and writing to the bot branch needs
its token. Instead, on a branch outside the patterns above (#993 for #920):

- **Run `uv lock --upgrade` with the uv version CI pins** (`version:` on
  `astral-sh/setup-uv` in `.github/actions/setup-tools/action.yml`, the one
  place CI installs uv; 0.12.23 at #993), via
  `uv tool install uv==<version>`. A local uv 0.11.32 could not resolve
  melody-train at all (`No interpreter found for Python 3.12.15`).
- **Compare per-file diff sizes with the bot PR.** At #993 they matched #920's
  within 2 lines; a larger gap means a different resolver or a moved base.
- **Run the checks a lockfile-only diff skips.** The `ty` pre-commit hook
  fires only on `packages/robotics/robocar/simulation/src/*.py`, so a ty bump in
  `pyproject.toml` or `uv.lock` alone is never type-checked in CI; run
  `uv run ty check src/` there. melody-train leaves `ty` unpinned, so a lock
  refresh can jump it many releases: #920 moved it 0.0.34 to 0.0.99, past
  ty 0.0.97's rename of the `unresolved-import` rule to `import-not-found`,
  and `ty check src` failed on the renamed-rule warning.
