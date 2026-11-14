# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

MCU Tinkering Lab is a production-ready embedded systems monorepo for ESP32, STM32, and Arduino platforms. The primary project is an AI-powered robot car on a single XIAO ESP32-S3 Sense (`robocar`, in `packages/robotics/robocar/`).

## Repository Structure

Projects are grouped by domain under `packages/`. Add new projects to the
folder that matches the domain, or create a new domain folder. Drop the
`esp32-` prefix — the category already implies it.

```
mcu-tinkering-lab/
├── packages/
│   ├── robotics/                   # Robots
│   │   ├── robocar/                # AI robot car: firmware (XIAO ESP32-S3 Sense) and its socketed carrier PCB
│   │   │   ├── integrated/         # Carrier PCB variant with the breakout chips on board (hardware only)
│   │   │   ├── bringup/            # Hardware self-test firmware
│   │   │   ├── simulation/         # Python 3.12 physics simulation (Pymunk)
│   │   │   └── twin/               # Behaviour twin: planner C on a virtual clock + scenario harness (ADR-030)
│   │   └── balancebot/             # Self-balancing robot (XIAO RP2350 + MPU6050 + steppers, Pico SDK)
│   ├── thinkpack/                  # ESP-NOW mesh of modular companion "boxes"
│   │   ├── brainbox/               # LLM mesh coordinator (ESP32-S3)
│   │   ├── boombox/                # Audio playback box
│   │   ├── chatterbox/             # Conversational box
│   │   ├── finderbox/              # NFC/finder box
│   │   ├── glowbug/                # LED peer box
│   │   └── mesh-demo/              # ESP-NOW mesh demo
│   ├── camera-vision/              # Standalone camera / AI vision projects
│   │   ├── cam-webserver/          # MJPEG streaming server
│   │   ├── cam-i2s-audio/          # Camera + I2S audio
│   │   ├── llm-telegram/           # LLM vision with Telegram bot
│   │   └── gemini-vision/          # Gemini Robotics-ER object detection
│   ├── audio/                      # Audio / synth / toys
│   │   ├── gamepad-synth/          # BLE gamepad I2S synthesizer (Monotron-inspired)
│   │   ├── kids-audio-toy/         # Potentiometer-controlled audio toy
│   │   ├── melody-detector/        # Melody detection + ML training
│   │   └── audiobook-player/       # RFID audiobook player (ESPHome)
│   ├── input-gaming/               # Gamepads and controller bridges
│   │   ├── xbox-switch-bridge/     # Xbox BLE → Switch USB bridge
│   │   ├── switch-usb-proxy/       # Switch USB protocol proxy
│   │   ├── lego-boost-xbox/        # Drive LEGO Boost with an Xbox controller
│   │   └── lego-boost-xbox-fw/     # LEGO Boost direct Xbox-pairing hub firmware
│   ├── networking/                 # WiFi tests, VPN, network tools
│   │   ├── it-troubleshooter/      # IT troubleshooting assistant
│   │   ├── wifitest/               # WiFi AP test firmware
│   │   └── wireguard-ha/           # WireGuard + Home Assistant (ESPHome)
│   ├── home/                       # Home automation
│   │   └── ceiling-fan-ir/         # IR ceiling-fan controller (ESPHome, ESP8285)
│   ├── sensors/                    # Sensor firmware
│   │   └── presence-detector/      # 24 GHz mmWave presence sensor (ESPHome)
│   ├── usb-tools/                  # USB tools and protocol firmware
│   │   └── facedancer-espdancer-fw/ # ESP32-S3 raw USB relay for facedancer
│   ├── bench/                      # Workbench tools (3D-printed)
│   │   └── phone-microscope/       # Phone + M12 lens inspection microscope stand (build123d)
│   ├── games/
│   │   └── nfc-scavenger-hunt/     # NFC-based scavenger hunt game
│   └── components/                 # Reusable ESP-IDF components
│       ├── improv-wifi/            # Improv-WiFi BLE provisioning
│       ├── ota-github/             # GitHub Releases OTA updater
│       ├── log-udp/                # SoftAP + UDP broadcast log sink (SSID and password via Kconfig)
│       └── thinkpack-*/            # ThinkPack shared components (audio, mesh, nfc, power, …)
├── .github/workflows/              # CI/CD pipelines
├── tools/scaffold/                 # Project scaffolding scripts
├── docs/                           # decisions/, requirements/, prompts/, reference/
└── justfile                        # Build coordination
```

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Firmware | C/C++ with ESP-IDF v5.5 |
| Build system | CMake (via ESP-IDF), justfile |
| Simulation | Python 3.12, Pymunk, NumPy |
| Python package manager | uv |
| C/C++ formatter | clang-format (Google style, 4-space indent) |
| C/C++ linter | cppcheck |
| Python formatter | ruff |
| Python linter | ruff |
| Python type checker | ty (astral) |
| Secret scanning | gitleaks |
| Pre-commit | pre-commit (8 hooks) |
| CI/CD | GitHub Actions (11+ workflows) |
| Containers | Docker + docker-compose |
| Carrier PCBs | KiCad 10.0.6 (local; CI is KiCad-free) |

## Build Commands

### Quick Reference

```bash
# Show all available commands
just --list

# Build all projects (containerized — no local ESP-IDF needed)
just build-all

# Robocar (XIAO ESP32-S3 Sense; the port is auto-detected by USB VID)
just robocar::build
just robocar::flash-monitor
just robocar::test    # host unit tests, no ESP-IDF needed
```

### Code Quality

```bash
# Lint all code
just lint                    # Runs lint-c + lint-python

# Format all code
just format                  # Runs format-c + format-python

# Check formatting (CI-safe, no modifications)
just format-check

# Install dev tools + pre-commit hooks
just install-dev-tools

# Run pre-commit hooks manually
pre-commit run --all-files
```

### Python Simulation

```bash
cd packages/robotics/robocar/simulation
uv sync                      # Install dependencies
uv run pytest tests/ --cov   # Run tests with coverage
```

### Docker (all ESP-IDF builds are containerized — no local ESP-IDF needed)

```bash
just docker-build            # Build development images
just docker-dev              # Interactive ESP-IDF shell
just setup-all               # Docker images + dev tools
```

## Project Conventions

### Commit Messages
Use conventional commits: `feat:`, `fix:`, `docs:`, `chore:`, `refactor:`, `test:`, `ci:`, `build:`

Examples:
- `feat: Add WiFi reconnection logic to robocar`
- `fix: Correct UART baud rate for camera communication`
- `docs: Update hardware connection pinout`

### Code Style
- **C/C++**: Google style with 4-space indent, 100-char line limit. Config in `.clang-format`.
- **Python**: Ruff formatting and linting. Follow standard ruff defaults.
- Pre-commit hooks enforce formatting automatically on commit.

### File Organization
- ESP32 firmware projects go in `packages/<domain>/<project-name>/` (pick an
  existing domain folder that fits, or create a new one)
- A subproject of an existing project goes beneath that project
  (`packages/robotics/robocar/bringup/`). It sits one level deeper, so its
  justfile imports `../../../../tools/...` and its docs reach the repo root
  with four `../`
- Each ESP32 project has its own `CMakeLists.txt`, `main/`, and `sdkconfig.defaults`
- Repo-wide reusable components live in `packages/components/` (improv-wifi,
  ota-github)
- Domain-local components live in `packages/<domain>/components/`

### Credentials
- **Never commit** `credentials.h`, `wifi_config.h`, or files ending in `.key`, `.secret`, `.token`
- Pre-commit hooks block credential files automatically
- Use `sdkconfig.defaults` for non-sensitive ESP-IDF config; sensitive values go in `credentials.h` (gitignored)

## Architecture Notes

### Robot Car (robocar)
One XIAO ESP32-S3 Sense runs camera capture (OV3660), the Gemini Robotics-ER
planner, a 30 Hz reactive executor, motors and servos (PCA9685 behind a TCA9548A
mux), and Gemini TTS voice through a MAX98357A. Architecture, core affinity and
the rules for changing it are in
[`packages/robotics/robocar/CLAUDE.md`](packages/robotics/robocar/CLAUDE.md).

`integrated/`, `bringup/`, `simulation/` and `twin/` beneath it are separate
projects, each with its own justfile; the `mod` lines in the root `justfile`
name their modules.

The earlier dual-ESP32 design (Heltec main controller + ESP32-CAM over I2C,
ADR-002 and ADR-007) has been removed; it remains in git history.

Until 2026-11 the project was `robocar-unified` in `packages/robocar/unified/`.
Releases up to 0.2.23 are tagged `robocar-unified-v<version>` and carry
`robocar-unified.bin`, and from 0.2.18 `robocar-unified.elf`; later ones are
tagged `robocar-v<version>` and carry `robocar.bin` and `robocar.elf`.
CHANGELOG entries and quoted PR titles from before the move keep the old name,
as do some dated incident notes under `.claude/`.

## CI/CD Workflows

Most workflows delegate to reusable workflows from [`laurigates/.github`](https://github.com/laurigates/.github).

| Workflow | Trigger | Purpose | Reusable? |
|----------|---------|---------|-----------|
| `build.yml` | Push/PR (path-filtered) | Single CI build for every firmware project (ESP-IDF/ESPHome/Pico). Discovers changed projects from `.github/project-matrix.json` (an entry's `exclude_paths` drops the projects nested inside it from its triggers) and fans out via the `_ci-build-*.yml` reusables | Local reusable `_ci-build-*.yml` |
| `build-python-simulation.yml` | Push/PR to main/develop | Build the Python simulation package (import check) | No (local) |
| `test.yml` | Push/PR to main/develop | Pre-commit, simulation pytest, cppcheck, format check, and the Python Suites matrix: every uv project test suite in `.github/python-suites.json` (tools/enclosure, mechanism, cadstd, glitchtip bridge, the robocar behaviour twin per [ADR-030](docs/decisions/ADR-030-robocar-behaviour-twin.md), the live probe, the TTS generator, and the make-only component host suites, `just components::test`) on its own path changes, with `UV_LOCKED=1`. The CI Config Hooks job runs, on every event, the hooks that guard workflows, ESP-IDF version pins, the root justfile, `project-matrix.json` and the just pin, since a PR touching only those runs no Code Quality Checks; `tools/test_workflows.py` fails when such a hook is left uncovered (#1089) | No (local) |
| `build-firmware.yml` | Release published / manual | Check the tag is `<component>-v<semver>`, then build + attach the flasher-enabled ESP-IDF binaries to the release and deploy the web flasher; a Pico SDK project's release (balancebot) also gets its `.uf2`, built by `_ci-build-pico.yml` at the tag. Release-asset check: `tools/verify-release-assets.sh` | No (local); Pico via local `_ci-build-pico.yml` |
| `release-please.yml` | Push to main | Auto-generate release PRs from conventional commits | `reusable-release-please.yml` |
| `claude.yml` | @claude mention | Interactive Claude assistance in issues/PRs | `reusable-claude.yml` |
| `auto-fix.yml` | CI failure / manual | Analyze and auto-fix CI failures with Claude | `reusable-auto-fix.yml` |
| `enforce-conventional-commits.yml` | PR opened/edited/sync | Auto-fix PR titles to conventional commits format | `reusable-enforce-conventional-commits.yml` |
| `security-secrets.yml` | PR to main/develop | gitleaks over the PR's commits, with a planted-key canary; no actor gate, so bot PRs are scanned too (the Claude scan was removed in #860) | No (local) |
| `auto-resolve-conflicts.yml` | Push to main / schedule | Auto-resolve merge conflicts in PRs with Claude | `reusable-auto-resolve-conflicts.yml` |
| `fix-release-conflicts.yml` | Push to main / schedule | Fix conflicts in release-please PRs | `reusable-fix-release-conflicts.yml` |
| `sync-ai-rules.yml` | Weekly / manual | Sync AI coding rules from `.github` repo | `reusable-sync-ai-rules.yml` |
| `carrier-check.yml` | Push/PR (path-filtered) | KiCad-free carrier check: `tools/kicad` tests, vendored-library hashes, schematic regenerates byte-identical, board pads match the join, the check stamp matches the committed files, every BOM part has an `lcsc` or `buy` source (`kicad.sourcing`) ([ADR-027](docs/decisions/ADR-027-kicad-carrier-generation.md)) | No (local) |
| `refresh-idf-locks.yml` | Monthly / manual | Regenerate every git-tracked ESP-IDF `dependencies.lock` and open one PR with the diff — see [`.claude/rules/esp-idf-dependency-locks.md`](.claude/rules/esp-idf-dependency-locks.md) | No (local) |
| `issue-labels.yml` | Issue and PR events / push of the taxonomy / weekly / manual | Type and routing labels (`build:`, `area:`, `core:`) from titles, quoted paths and changed files; clears `blocked` when every linked blocker closes; weekly label audit. No model. Runs with `dry-run: true`: it reports and writes nothing | `reusable-issue-labels.yml` |

## Important Paths

- `tools/glitchtip/` — self-hosted GlitchTip plus the MQTT bridge that forwards robocar resets, crash backtraces and planner notes into it ([ADR-033](docs/decisions/ADR-033-robocar-error-reporting.md)); the repo `.mcp.json` registers GlitchTip's built-in MCP server as `glitchtip` (exports from `just glitchtip::mcp-env`), and `just glitchtip::issues` / `::event` read issues as text without it. Triage workflow: the `glitchtip-triage` skill
- `tools/esp32.just` — Shared justfile config imported by all ESP-IDF projects (container_cmd, port detection, require-port, _serial-monitor)
- `tools/hardware/` — the board × header × parts join ([ADR-021](docs/decisions/ADR-021-hardware-source-of-truth.md)): reads a project's `hardware.toml`, its `pin_config.h` and the repo-level `docs/reference/boards/` page the sidecar names; `just hardware::gen` regenerates everything it feeds: `docs/auto/pin_defs.typ`; the tables between `<!-- BEGIN GENERATED -->` markers in any Markdown git would track under a project, at any depth, such as `WIRING.md`; the board pinout images; the schematic SVG/PNG (via `schematics::render-all`); and the build-guide PDFs; `just hardware::check` is the CI gate (`hardware-check.yml`)
- `tools/kicad/` — KiCad carrier PCBs generated from the hardware join ([ADR-027](docs/decisions/ADR-027-kicad-carrier-generation.md)): the stdlib schematic writer, the KiCad-free board and stamp checks CI runs (`just kicad::check`, `carrier-check.yml`), the pcbnew scripts under `pcb/`, and `lib/`, the vendored KiCad 10.0.6 and Seeed XIAO library subset pinned by `MANIFEST.tsv`. Workflow and traps: the `carrier-pcb` skill
- `tools/mechanism/` — printed mechanisms (servo pan-tilts, brackets, linkages) in build123d: servo keep-out envelopes, the clash sweep over a design's travel, tools/enclosure's wall gate, and VTK renders (`just mechanism::check|showcase|animate <design dir>`). Contract, checklist and the design tournament: the `printed-mechanism` skill. Worked example: `packages/robotics/robocar/hardware/pan-tilt/`
- `tools/cadstd/` — the shared dimensions of printed parts ([ADR-035](docs/decisions/ADR-035-printed-part-dimension-standard.md)): metric fasteners, fits by class per print process (`fdm`, `sla`, `mjf`), `MIN_WALL`, the 10 mm M3 mounting grid, build123d cutters, and the calibration coupon (`just cadstd::test|coupon <process>`). `tools/enclosure` and `tools/mechanism` build from it
- `tools/labels/` — parts-drawer stickers: TOML data files to A4 print-and-cut pages, printed on the Epson and kiss-cut on the Silhouette Portrait 2 through registration marks, headless (`just labels::build|print|cut`). `labels::candidates` drafts labels for unlabelled items in the private parts-stock repo, read-only. Workflow and cutter traps: the `component-labels` skill
- `tools/kicad.just` — Shared carrier recipes a project imports (`carrier-schematic`, `carrier-board`, `carrier-board-checked`, `carrier-check`, `carrier-review`, `carrier-fab`); a project's `hardware/carrier/board.toml` holds placement, fixed tracks, silkscreen, net classes, the pour and the ground-via fanout
- `docs/reference/parts/` — the parts registry: one TOML per physical part a carrier mounts (KiCad symbol, footprint, LCSC number, pin map and where the pin order came from), named from `hardware.toml` by `part = "<slug>"`
- `docs/reference/blocks/` — circuit blocks: a stage (buck, pack-input protection) designed once from registry parts, used from `hardware.toml` by `[blocks.<id>]` and expanded when the file is read (`tools/hardware/blocks.py`, [ADR-031](docs/decisions/ADR-031-circuit-blocks.md))
- `docker-compose.yml` — ESP-IDF container service definition, built from the `Dockerfile` (`espressif/idf:v5.5.6`). The ESP-IDF version is set in the `Dockerfile`, the compose image tag and `esp-idf-tools` volume name, and `IDF_VERSION` in `_ci-build-esp32.yml`, `build-firmware.yml` and `refresh-idf-locks.yml`; `tools/kconfig_symbols/test_kconfig_symbols.py` fails when they disagree
- `tools/kconfig_symbols/` — checks every tracked `sdkconfig.defaults` without a build: each symbol must be defined by ESP-IDF (`idf-v<version>.txt`, extracted from the `espressif/idf` image by `just kconfig-symbols::idf`), the project's own Kconfig files, or its managed components (`components/<ns>__<name>@<version>.txt`, recorded from a build's `managed_components/` by `just kconfig-symbols::component`). ESP-IDF ignores an unknown symbol with only a build-log warning (#707). Runs as the `kconfig-symbols` pre-commit hook; a test fails when the ESP-IDF list and the Dockerfile's IDF version disagree
- `tools/refresh-idf-locks.py` — the one update path for git-tracked `dependencies.lock` files (`just refresh-idf-locks`, and the monthly `refresh-idf-locks.yml`). Nothing else re-resolves a tracked lock while its `idf_component.yml` is unchanged; editing the manifest re-resolves on the next build. Procedure: [`.claude/rules/esp-idf-dependency-locks.md`](.claude/rules/esp-idf-dependency-locks.md)
- `packages/components/host-test/` — `unity_compat.h`, `esp_log.h` and `esp_err.h`, shared by every component host suite (`packages/<...>/components/<name>/test/Makefile`); `just components::test` builds and runs all of them (#1077)
- `.clang-format` — C/C++ formatting rules
- `.pre-commit-config.yaml` — Pre-commit hook definitions
- `.gitleaks.toml` — Secret scanning allowlist
- `.github/python-suites.json` — the test suites `test.yml`'s Python Suites job runs: each one's uv project `dir` (none for the make-only `components` suite), its `run` command, any `apt` packages and the `paths` that select it (`tools/python-suites.py` does the selection). Add a suite here, not a new workflow. `tools/test_python_suites.py` (pre-commit, and the Python Suites Drift Test job) fails when a tracked `uv.lock` is installed without `--locked` or by no workflow, when a `tools/*/justfile` `test` recipe runs nowhere in CI, when a component's `test/` directory is one `just components::test` would not run, or when a workflow installs uv or just other than through `.github/actions/setup-tools` (the one place both are pinned)
- `.github/issue-labels.json` — the label taxonomy `issue-labels.yml` reads from the default branch: each `build:` and `area:` label's colour and description, plus the path globs and title scopes that select every `build:`, `area:` and `core:` label. A new `build:` or `area:` label starts as an entry here. The workflow's provision job creates it on the next push to `main` once `dry-run` is off (#1066); while the caller sets `dry-run: true`, provision only reports, so the label also has to be created by hand with the entry's name, colour and description. `tools/test_issue_labels_taxonomy.py` (pre-commit, and the Label Taxonomy Drift Test job in `test.yml`) fails on a field or value the script would reject, a glob that matches no tracked path, and a scope two entries share
- `tools/scaffold/new-esp32-project.sh` — New project scaffolding
- `docs/flasher/index.html` — ESP Web Tools browser-based firmware flasher page
