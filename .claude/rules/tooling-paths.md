---
reviewed: 2026-11-24
paths:
  - 'tools/**'
  - 'scripts/**'
  - 'docker-compose.yml'
  - 'Dockerfile*'
  - '.github/**'
  - '.mcp.json'
  - '.pre-commit-config.yaml'
  - 'docs/reference/parts/**'
  - 'docs/reference/blocks/**'
  - 'packages/components/host-test/**'
  - 'packages/**/hardware/carrier/**'
  - '**/hardware.toml'
  - '**/sdkconfig.defaults'
  - '**/idf_component.yml'
  - '**/dependencies.lock'
---

# Important Paths: What Each Tool Checks

The full entries behind `CLAUDE.md` § Important Paths, which keeps one line
per path.

- `tools/glitchtip/` — self-hosted GlitchTip plus the MQTT bridge that forwards robocar resets, crash backtraces and planner notes into it ([ADR-033](../../docs/decisions/ADR-033-robocar-error-reporting.md)); the repo `.mcp.json` registers GlitchTip's built-in MCP server as `glitchtip` (exports from `just glitchtip::mcp-env`), and `just glitchtip::issues` / `::event` read issues as text without it. Triage workflow: the `glitchtip-triage` skill
- `tools/esp32.just` — Shared justfile config imported by all ESP-IDF projects (container_cmd, port detection, require-port, _serial-monitor)
- `tools/hardware/` — the board × header × parts join ([ADR-021](../../docs/decisions/ADR-021-hardware-source-of-truth.md)): reads a project's `hardware.toml`, its `pin_config.h` and the repo-level `docs/reference/boards/` page the sidecar names; `just hardware::gen` regenerates everything it feeds: `docs/auto/pin_defs.typ`; the tables between `<!-- BEGIN GENERATED -->` markers in any Markdown git would track under a project, at any depth, such as `WIRING.md`; the board pinout images; the schematic SVG/PNG (via `schematics::render-all`); and the build-guide PDFs; `just hardware::check` is the CI gate (`hardware-check.yml`)
- `tools/kicad/` — KiCad carrier PCBs generated from the hardware join ([ADR-027](../../docs/decisions/ADR-027-kicad-carrier-generation.md)): the stdlib schematic writer, the KiCad-free board and stamp checks CI runs (`just kicad::check`, `carrier-check.yml`), the pcbnew scripts under `pcb/`, and `lib/`, the vendored KiCad 10.0.6 and Seeed XIAO library subset pinned by `MANIFEST.tsv`. Workflow and traps: the `carrier-pcb` skill
- `tools/mechanism/` — printed mechanisms (servo pan-tilts, brackets, linkages) in build123d: servo keep-out envelopes, the clash sweep over a design's travel, tools/enclosure's wall gate, and VTK renders (`just mechanism::check|showcase|animate <design dir>`). Contract, checklist and the design tournament: the `printed-mechanism` skill. Worked example: `packages/robotics/robocar/hardware/pan-tilt/`
- `tools/cadstd/` — the shared dimensions of printed parts ([ADR-035](../../docs/decisions/ADR-035-printed-part-dimension-standard.md)): metric fasteners, fits by class per print process (`fdm`, `sla`, `mjf`), `MIN_WALL`, the 10 mm M3 mounting grid, build123d cutters, and the calibration coupon (`just cadstd::test|coupon <process>`). `tools/enclosure` and `tools/mechanism` build from it
- `tools/labels/` — parts-drawer stickers: TOML data files to A4 print-and-cut pages, printed on the Epson and kiss-cut on the Silhouette Portrait 2 through registration marks, headless (`just labels::build|print|cut`). `labels::candidates` drafts labels for unlabelled items in the private parts-stock repo, read-only. Workflow and cutter traps: the `component-labels` skill
- `tools/kicad.just` — Shared carrier recipes a project imports (`carrier-schematic`, `carrier-board`, `carrier-board-checked`, `carrier-check`, `carrier-review`, `carrier-fab`); a project's `hardware/carrier/board.toml` holds placement, fixed tracks, silkscreen, net classes, the pour and the ground-via fanout
- `docs/reference/parts/` — the parts registry: one TOML per physical part a carrier mounts (KiCad symbol, footprint, LCSC number, pin map and where the pin order came from), named from `hardware.toml` by `part = "<slug>"`
- `docs/reference/blocks/` — circuit blocks: a stage (buck, pack-input protection) designed once from registry parts, used from `hardware.toml` by `[blocks.<id>]` and expanded when the file is read (`tools/hardware/blocks.py`, [ADR-031](../../docs/decisions/ADR-031-circuit-blocks.md))
- `docker-compose.yml` — ESP-IDF container service definition, built from the `Dockerfile` (`espressif/idf:v5.5.6`). The ESP-IDF version is set in the `Dockerfile`, the compose image tag and `esp-idf-tools` volume name, and `IDF_VERSION` in `_ci-build-esp32.yml`, `build-firmware.yml` and `refresh-idf-locks.yml`; `tools/kconfig_symbols/test_kconfig_symbols.py` fails when they disagree
- `tools/kconfig_symbols/` — checks every tracked `sdkconfig.defaults` without a build: each symbol must be defined by ESP-IDF (`idf-v<version>.txt`, extracted from the `espressif/idf` image by `just kconfig-symbols::idf`), the project's own Kconfig files, or its managed components (`components/<ns>__<name>@<version>.txt`, recorded from a build's `managed_components/` by `just kconfig-symbols::component`). ESP-IDF ignores an unknown symbol with only a build-log warning (#707). It also checks the assignments in fenced code blocks of tracked Markdown, against ESP-IDF, every recorded component and every tracked Kconfig file together (#1092). Runs as the `kconfig-symbols` pre-commit hook; a test fails when the ESP-IDF list disagrees with any of the ESP-IDF pins the `docker-compose.yml` entry above lists
- `tools/check-md-links.py` — fails on a relative Markdown link whose target git does not track (#1147). Runs as the `check-md-links` pre-commit hook on every commit, since moving a target breaks a link without touching a `.md`, and in `test.yml`'s CI Config Hooks job, so a docs-only PR is checked. Each exclude in its `EXCLUDES` carries its reason
- `tools/refresh-idf-locks.py` — the one update path for git-tracked `dependencies.lock` files (`just refresh-idf-locks`, and the monthly `refresh-idf-locks.yml`). Nothing else re-resolves a tracked lock while its `idf_component.yml` is unchanged; editing the manifest re-resolves on the next build. Procedure: [`.claude/rules/esp-idf-dependency-locks.md`](esp-idf-dependency-locks.md)
- `packages/components/host-test/` — `unity_compat.h`, `esp_log.h` and `esp_err.h`, shared by every component host suite (`packages/<...>/components/<name>/test/Makefile`); `just components::test` builds and runs all of them (#1077)
- `.clang-format` — C/C++ formatting rules
- `.pre-commit-config.yaml` — Pre-commit hook definitions
- `.gitleaks.toml` — Secret scanning allowlist
- `.github/python-suites.json` — the test suites `test.yml`'s Python Suites job runs: each one's uv project `dir` (none for the make-only `components` suite; `robocar-twin-physics` names the twin harness, and its recipe installs the simulation's plant from that project's own lock), its `run` command, any `apt` packages and the `paths` that select it (`tools/python-suites.py` does the selection). Add a suite here, not a new workflow. `tools/test_python_suites.py` (pre-commit, and the Python Suites Drift Test job) fails when a tracked `uv.lock` is installed without `--locked` or by no workflow, when a `tools/*/justfile` `test` recipe runs nowhere in CI, when a component's `test/` directory is one `just components::test` would not run, when a workflow installs uv or just other than through `.github/actions/setup-tools` (the one place CI pins both), or when `scripts/install_pkgs.sh` installs another just than that pin
- `.github/issue-labels.json` — the label taxonomy `issue-labels.yml` reads from the default branch: each `build:` and `area:` label's colour and description, plus the path globs and title scopes that select every `build:`, `area:` and `core:` label. A new `build:` or `area:` label starts as an entry here. The workflow's provision job creates it, with the entry's colour and description, on the push to `main` that adds the entry. `tools/test_issue_labels_taxonomy.py` (pre-commit, and the Label Taxonomy Drift Test job in `test.yml`) fails on a field or value the script would reject, a glob that matches no tracked path, and a scope two entries share
- `tools/scaffold/new-esp32-project.sh` — New project scaffolding
- `docs/flasher/index.html` — ESP Web Tools browser-based firmware flasher page
