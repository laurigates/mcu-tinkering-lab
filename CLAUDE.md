# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

MCU Tinkering Lab is a production-ready embedded systems monorepo for ESP32, STM32, and Arduino platforms. The primary project is an AI-powered dual-ESP32 robot car with computer vision.

## Repository Structure

Projects are grouped by domain under `packages/`. Add new projects to the
folder that matches the domain, or create a new domain folder. Drop the
`esp32-` prefix — the category already implies it.

```
mcu-tinkering-lab/
├── packages/
│   ├── robocar/                    # Dual-ESP32 AI robot car system
│   │   ├── main/                   # Main controller (Heltec WiFi LoRa 32)
│   │   ├── camera/                 # ESP32-CAM vision (Claude/Ollama/Gemini)
│   │   ├── unified/                # Single-board XIAO ESP32-S3 Sense consolidation
│   │   ├── simulation/             # Python 3.11 physics simulation (Pymunk)
│   │   ├── docs/                   # Docs and coordination justfile
│   │   └── components/i2c-protocol/ # Robocar-internal I2C protocol + tests
│   ├── camera-vision/              # Standalone camera / AI vision projects
│   │   ├── cam-webserver/          # MJPEG streaming server
│   │   ├── cam-i2s-audio/          # Camera + I2S audio
│   │   ├── llm-telegram/           # LLM vision with Telegram bot
│   │   └── gemini-vision/          # Gemini Robotics-ER object detection
│   ├── audio/                      # Audio / synth / toys
│   │   ├── gamepad-synth/          # PS4 gamepad I2S synthesizer
│   │   ├── kids-audio-toy/         # Potentiometer-controlled audio toy
│   │   └── audiobook-player/       # RFID audiobook player (ESPHome)
│   ├── input-gaming/               # Gamepads and controller bridges
│   │   ├── xbox-switch-bridge/     # Xbox BLE → Switch USB bridge
│   │   └── switch-usb-proxy/       # Switch USB protocol proxy
│   ├── networking/                 # WiFi tests, VPN, network tools
│   │   ├── it-troubleshooter/      # IT troubleshooting assistant
│   │   ├── wifitest/               # WiFi AP test firmware
│   │   └── wireguard-ha/           # WireGuard + Home Assistant (ESPHome)
│   ├── games/
│   │   └── nfc-scavenger-hunt/     # NFC-based scavenger hunt game
│   └── components/                 # Reusable ESP-IDF components
│       ├── improv-wifi/            # Improv-WiFi BLE provisioning
│       └── ota-github/             # GitHub Releases OTA updater
├── .github/workflows/              # CI/CD pipelines
├── tools/scaffold/                 # Project scaffolding scripts
├── docs/                           # decisions/, requirements/, prompts/, reference/
└── justfile                        # Build coordination
```

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Firmware | C/C++ with ESP-IDF v5.4+ |
| Build system | CMake (via ESP-IDF), justfile |
| Simulation | Python 3.11, Pymunk, NumPy |
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

## Build Commands

### Quick Reference

```bash
# Show all available commands
just --list

# Build all projects (containerized — no local ESP-IDF needed)
just build-all

# Build specific robocar components
just robocar::build-main      # Main controller only
just robocar::build-cam       # Camera module only
just robocar::build-all       # Both controllers

# Development workflows (build + flash + monitor)
just robocar::develop-main    # Main controller
just robocar::develop-cam     # Camera module

# Flash (PORT=/dev/cu.usbserial-0001 by default)
PORT=/dev/ttyUSB0 just robocar::flash-main
PORT=/dev/ttyUSB1 just robocar::flash-cam
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
cd packages/robocar/simulation
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
- `feat: Add WiFi reconnection logic to robocar-main`
- `fix: Correct UART baud rate for camera communication`
- `docs: Update hardware connection pinout`

### Code Style
- **C/C++**: Google style with 4-space indent, 100-char line limit. Config in `.clang-format`.
- **Python**: Ruff formatting and linting. Follow standard ruff defaults.
- Pre-commit hooks enforce formatting automatically on commit.

### File Organization
- ESP32 firmware projects go in `packages/<domain>/<project-name>/` (pick an
  existing domain folder that fits, or create a new one)
- Each ESP32 project has its own `CMakeLists.txt`, `main/`, and `sdkconfig.defaults`
- Repo-wide reusable components live in `packages/components/` (improv-wifi,
  ota-github)
- Domain-local components live in `packages/<domain>/components/` (e.g.
  `packages/robocar/components/i2c-protocol/`)

### Credentials
- **Never commit** `credentials.h`, `wifi_config.h`, or files ending in `.key`, `.secret`, `.token`
- Pre-commit hooks block credential files automatically
- Use `sdkconfig.defaults` for non-sensitive ESP-IDF config; sensitive values go in `credentials.h` (gitignored)

## Architecture Notes

### Dual ESP32 Robot Car
The robot car uses two ESP32 boards connected via UART:
- **Main Controller** (Heltec WiFi LoRa 32 V1): Motor control via L298N, PCA9685 for LEDs/servos, OLED display, buzzer
- **Camera** (ESP32-CAM): OV2640 image capture, AI inference (Claude API or Ollama), MQTT telemetry

The camera sends single-character movement commands (F/B/L/R/S) and PAN:/TILT: commands over UART to the main controller.

### AI Backend Selection
AI backends are selected at compile time via `#define`:
- `USE_CLAUDE_API` — Anthropic Claude API (cloud, higher quality)
- `USE_OLLAMA` — Ollama (self-hosted, lower latency, no API costs)

### ESP32-CAM Pin Constraints
GPIO14/15 are used for UART to avoid PSRAM conflicts. The ESP32-CAM has very limited available GPIOs due to camera and PSRAM usage.

## CI/CD Workflows

Most workflows delegate to reusable workflows from [`laurigates/.github`](https://github.com/laurigates/.github).

| Workflow | Trigger | Purpose | Reusable? |
|----------|---------|---------|-----------|
| `esp32-build.yml` | Push/PR to main/develop | Build all ESP32 firmware + simulation | No (local) |
| `test.yml` | Push/PR to main/develop | Pre-commit, pytest, cppcheck, format check | No (local) |
| `build-firmware.yml` | Release published | Build + attach firmware binaries to release + deploy web flasher | No (local) |
| `release-please.yml` | Push to main | Auto-generate release PRs from conventional commits | `reusable-release-please.yml` |
| `claude-code-review.yml` | PR opened/updated | Auto-review PRs with Claude | `reusable-claude-review.yml` |
| `claude.yml` | @claude mention | Interactive Claude assistance in issues/PRs | `reusable-claude.yml` |
| `auto-fix.yml` | CI failure / manual | Analyze and auto-fix CI failures with Claude | `reusable-auto-fix.yml` |
| `enforce-conventional-commits.yml` | PR opened/edited/sync | Auto-fix PR titles to conventional commits format | `reusable-enforce-conventional-commits.yml` |
| `security-secrets.yml` | Push/PR to main/develop | AI-powered secret scanning on changed files | `reusable-security-secrets.yml` |
| `auto-resolve-conflicts.yml` | Push to main / schedule | Auto-resolve merge conflicts in PRs with Claude | `reusable-auto-resolve-conflicts.yml` |
| `fix-release-conflicts.yml` | Push to main / schedule | Fix conflicts in release-please PRs | `reusable-fix-release-conflicts.yml` |
| `sync-ai-rules.yml` | Weekly / manual | Sync AI coding rules from `.github` repo | `reusable-sync-ai-rules.yml` |

## Important Paths

- `tools/esp32.just` — Shared justfile config imported by all ESP-IDF projects (container_cmd, port detection, require-port, _serial-monitor)
- `docker-compose.yml` — ESP-IDF container service definition (`espressif/idf:v5.4`)
- `.clang-format` — C/C++ formatting rules
- `.pre-commit-config.yaml` — Pre-commit hook definitions
- `.gitleaks.toml` — Secret scanning allowlist
- `tools/scaffold/new-esp32-project.sh` — New project scaffolding
- `packages/robocar/docs/` — Robocar coordination justfile and docs
- `docs/flasher/index.html` — ESP Web Tools browser-based firmware flasher page
