---
reviewed: 2026-11-24
paths:
  - '.github/**'
  - 'tools/test_workflows.py'
  - 'tools/*python*suites*.py'
  - 'tools/verify-release-assets.sh'
---

# CI/CD Workflows

Most workflows delegate to reusable workflows from
[`laurigates/.github`](https://github.com/laurigates/.github). `CLAUDE.md`
§ CI/CD Workflows points here.

| Workflow | Trigger | Purpose | Reusable? |
|----------|---------|---------|-----------|
| `build.yml` | Push/PR (path-filtered) | Single CI build for every firmware project (ESP-IDF/ESPHome/Pico). Discovers changed projects from `.github/project-matrix.json` (an entry's `exclude_paths` drops the projects nested inside it from its triggers) and fans out via the `_ci-build-*.yml` reusables | Local reusable `_ci-build-*.yml` |
| `build-python-simulation.yml` | Push/PR to main/develop | Build the Python simulation package (import check) | No (local) |
| `test.yml` | Push/PR to main/develop | Pre-commit, simulation pytest, cppcheck, format check, and the Python Suites matrix: every uv project test suite in `.github/python-suites.json` (tools/enclosure, mechanism, cadstd, glitchtip bridge, the robocar behaviour twin per [ADR-030](../../docs/decisions/ADR-030-robocar-behaviour-twin.md) as two suites, `robocar-twin` for its scripted scenarios and `robocar-twin-physics` for the ones stepped against the simulation's plant per [ADR-036](../../docs/decisions/ADR-036-robocar-twin-plant-port.md), the live probe, the TTS generator, and the make-only component host suites, `just components::test`) on its own path changes, with `UV_LOCKED=1`. The CI Config Hooks job runs, on every event, the hooks that guard workflows, ESP-IDF version pins, the root justfile, `project-matrix.json`, the just pin, the release-please components the OTA topics use and the Claude post-edit hook, since a PR touching only those runs no Code Quality Checks; `tools/test_workflows.py` fails when a hook watching `.github/`, `.claude/hooks/` or a root file other than Markdown is left uncovered (#1089) | No (local) |
| `build-firmware.yml` | Release published / manual | Check the tag is `<component>-v<semver>`, then build + attach the flasher-enabled ESP-IDF binaries to the release and deploy the web flasher; a Pico SDK project's release (balancebot) also gets its `.uf2`, built by `_ci-build-pico.yml` at the tag. Release-asset check: `tools/verify-release-assets.sh` | No (local); Pico via local `_ci-build-pico.yml` |
| `release-please.yml` | Push to main | Auto-generate release PRs from conventional commits | `reusable-release-please.yml` |
| `claude.yml` | @claude mention | Interactive Claude assistance in issues/PRs | `reusable-claude.yml` |
| `auto-fix.yml` | CI failure / manual | Analyze and auto-fix CI failures with Claude | `reusable-auto-fix.yml` |
| `enforce-conventional-commits.yml` | PR opened/edited/sync | Auto-fix PR titles to conventional commits format | `reusable-enforce-conventional-commits.yml` |
| `security-secrets.yml` | PR to main/develop | gitleaks over the PR's commits, with a planted-key canary; no actor gate, so bot PRs are scanned too (the Claude scan was removed in #860) | No (local) |
| `auto-resolve-conflicts.yml` | Push to main / schedule | Auto-resolve merge conflicts in PRs with Claude | `reusable-auto-resolve-conflicts.yml` |
| `fix-release-conflicts.yml` | Push to main / schedule | Fix conflicts in release-please PRs | `reusable-fix-release-conflicts.yml` |
| `sync-ai-rules.yml` | Weekly / manual | Sync AI coding rules from `.github` repo | `reusable-sync-ai-rules.yml` |
| `carrier-check.yml` | Push/PR (path-filtered) | KiCad-free carrier check: `tools/kicad` tests, vendored-library hashes, schematic regenerates byte-identical, board pads match the join, the check stamp matches the committed files, every BOM part has an `lcsc` or `buy` source (`kicad.sourcing`) ([ADR-027](../../docs/decisions/ADR-027-kicad-carrier-generation.md)) | No (local) |
| `refresh-idf-locks.yml` | Monthly / manual | Regenerate every git-tracked ESP-IDF `dependencies.lock` and open one PR with the diff — see [`.claude/rules/esp-idf-dependency-locks.md`](esp-idf-dependency-locks.md) | No (local) |
| `issue-labels.yml` | Issue and PR events / push of the taxonomy / weekly / manual | Type and routing labels (`build:`, `area:`, `core:`) from titles, quoted paths and changed files; clears `blocked` when every linked blocker closes; weekly label audit. No model. Writes for every event but `pull_request`, with `pr-labels: false` (a manual run defaults to `dry-run: true`); a second call on `pull_request` is always `dry-run: true` and only previews a PR's labels in its job summary (#1066) | `reusable-issue-labels.yml` |
| `build-guide-check.yml` | Push/PR (path-filtered) / manual | Runs `tools/check-typst-docs.py`, then regenerates each `docs/auto/pin_defs.typ` and pinout SVG and recompiles every committed Typst document with the pinned CLI; fails on any drift — see [`.claude/rules/build-guide-drift-guard.md`](build-guide-drift-guard.md) | No (local) |
| `hardware-check.yml` | Push/PR (path-filtered) / manual | `just hardware::check` spelled out: the `tools/hardware` unit tests, then `hardware.docs --check` on every generated Markdown block | No (local) |
| `schematics-check.yml` | Push/PR (path-filtered) / manual | The `docs/schematics` test suites, then re-renders every circuit and fails when an SVG differs from the committed one; PNG drift is only a notice | No (local) |
| `test-melody-train.yml` | Push/PR to main/develop (path-filtered) / manual | melody-detector training package: ruff, ty, pytest, and the quantize-extra test, with `UV_LOCKED=1` | No (local) |
| `merge-method.yml` | PR opened/edited/sync/labeled | Advisory `merge-method` check and `merge:rebase` / `merge:squash` label; never required, and a human-applied `merge:*` label overrides | `reusable-merge-method.yml` |
| `pr-base-coverage-notice.yml` | PR opened/reopened/base edited | Comments on a PR whose base is not main/develop, since some workflows do not run for it (#487); builds and tests nothing | No (local) |
| `notebook-doc-reminder.yml` | PR (path-filtered) | Comments with the NotebookLM notebooks a PR's changed files map to — see `docs/reference/notebook-mapping.md` | No (local) |
| `notebook-refresh-reminder.yml` | Monthly / manual | Opens the month's NotebookLM refresh checklist issue (label `notebooks`) unless one is open | No (local) |
