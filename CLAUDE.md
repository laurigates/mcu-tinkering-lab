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
│   │   │   ├── simulation/         # Python 3.12 plant model for the behaviour twin (Pymunk, ADR-036) and the legacy demo
│   │   │   └── twin/               # Behaviour twin: planner and executor C on a virtual clock + scenario harness (ADR-030, ADR-036)
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
│       ├── host-test/              # Shared headers for the component host suites
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
| CI/CD | GitHub Actions (see CI/CD Workflows below) |
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
uv sync --extra dev          # numpy, pymunk, pyyaml and the test tools
uv run pytest tests/ --cov   # Run tests with coverage
just plant-demo              # Step the plant alone; the legacy viewer needs --extra viz
```

### Docker (all ESP-IDF builds are containerized — no local ESP-IDF needed)

```bash
just docker-build            # Build development images
just docker-dev              # Interactive ESP-IDF shell
just setup-all               # Docker images + dev tools
```

## Project Conventions

### Commit Messages
Conventional commits scoped by the project's `just` module, e.g.
`feat(robocar): add WiFi reconnection logic`. Types, release-please and PR
title rules: `.claude/rules/commit-and-release.md`; types and examples for
humans: [CONTRIBUTING.md § Commit messages](CONTRIBUTING.md#commit-messages).

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

The table of every workflow (trigger, purpose, reusable) is in
[`.claude/rules/ci-workflow-inventory.md`](.claude/rules/ci-workflow-inventory.md),
which loads when a file under `.github/` is read.

## Important Paths

One line each; the full entries (which check fails on what, where versions are
pinned, the procedures) are in
[`.claude/rules/tooling-paths.md`](.claude/rules/tooling-paths.md), which loads
when one of these paths is read.

- `tools/glitchtip/` — self-hosted GlitchTip and the MQTT bridge that forwards robocar resets, backtraces and planner notes (ADR-033); `.mcp.json` registers its MCP server. Triage: the `glitchtip-triage` skill
- `tools/esp32.just` — shared justfile config every ESP-IDF project imports (container_cmd, port detection, require-port, _serial-monitor)
- `tools/hardware/` — the board × header × parts join (ADR-021); `just hardware::gen` regenerates everything it feeds, `just hardware::check` is the CI gate
- `tools/kicad/` — KiCad carrier PCBs generated from the join (ADR-027); `just kicad::check` is the KiCad-free CI gate. Workflow: the `carrier-pcb` skill
- `tools/mechanism/` — printed servo mechanisms in build123d, gated by a clash sweep (`just mechanism::check`). Workflow: the `printed-mechanism` skill
- `tools/cadstd/` — shared printed-part dimensions: fasteners, fits per print process, `MIN_WALL`, the M3 grid (ADR-035)
- `tools/labels/` — parts-drawer stickers, printed and kiss-cut headless (`just labels::build|print|cut`). Workflow: the `component-labels` skill
- `tools/kicad.just` — shared carrier recipes a project imports; layout lives in `hardware/carrier/board.toml`
- `docs/reference/parts/` — the parts registry: one TOML per physical part a carrier mounts
- `docs/reference/blocks/` — circuit blocks, used from `hardware.toml` by `[blocks.<id>]` (ADR-031)
- `docker-compose.yml` — the ESP-IDF container service (`espressif/idf:v5.5.6`); the ESP-IDF version is pinned in several files, and a test fails when they disagree
- `tools/kconfig_symbols/` — checks every `sdkconfig.defaults` symbol exists, without a build (`kconfig-symbols` pre-commit hook)
- `tools/check-md-links.py` — fails on a relative Markdown link whose target git does not track (pre-commit, every commit)
- `tools/refresh-idf-locks.py` — the one update path for git-tracked `dependencies.lock` files (`just refresh-idf-locks`)
- `packages/components/host-test/` — headers shared by every component host suite; `just components::test` runs them all
- `.clang-format` — C/C++ formatting rules
- `.pre-commit-config.yaml` — Pre-commit hook definitions
- `.gitleaks.toml` — Secret scanning allowlist
- `.github/python-suites.json` — the test suites `test.yml`'s Python Suites job runs; add a suite here, not a new workflow
- `.github/issue-labels.json` — the label taxonomy `issue-labels.yml` reads; a new `build:` or `area:` label starts as an entry here
- `tools/scaffold/new-esp32-project.sh` — New project scaffolding
- `docs/flasher/index.html` — ESP Web Tools browser-based firmware flasher page
