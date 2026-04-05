# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

MCU Tinkering Lab is a production-ready embedded systems monorepo for ESP32, STM32, and Arduino platforms. The primary project is an AI-powered dual-ESP32 robot car with computer vision.

## Repository Structure

```
mcu-tinkering-lab/
├── packages/
│   └── esp32-projects/
│       ├── robocar-main/           # Main controller (Heltec WiFi LoRa 32 V1)
│       ├── robocar-camera/         # Vision system (ESP32-CAM + Claude/Ollama AI)
│       ├── robocar-simulation/     # Python 3.11 physics simulation (Pymunk)
│       ├── robocar-docs/           # Documentation and coordination justfile
│       ├── esp32cam-llm-telegram/  # Telegram bot with LLM vision
│       ├── esp32-cam-webserver/    # Live video streaming server
│       └── esp32-cam-i2s-audio/    # Camera + audio processing
├── .github/workflows/              # CI/CD (6 workflows)
├── tools/scaffold/                 # Project scaffolding scripts
├── docs/blueprint/                 # Architecture docs (PRDs, ADRs)
└── justfile                        # Build coordination (60+ targets)
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
| Python type checker | mypy |
| Secret scanning | gitleaks |
| Pre-commit | pre-commit (8 hooks) |
| CI/CD | GitHub Actions (6 workflows) |
| Containers | Docker + docker-compose |

## Build Commands

### Quick Reference

```bash
# Show all available commands
just --list

# Build all projects (requires ESP-IDF)
just build-all

# Build specific robocar components
just robocar-build-main      # Main controller only
just robocar-build-cam       # Camera module only
just robocar-build-all       # Both controllers

# Development workflows (build + flash + monitor)
just robocar-develop-main    # Main controller
just robocar-develop-cam     # Camera module

# Flash (PORT=/dev/cu.usbserial-0001 by default)
PORT=/dev/ttyUSB0 just robocar-flash-main
PORT=/dev/ttyUSB1 just robocar-flash-cam
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
cd packages/esp32-projects/robocar-simulation
uv sync                      # Install dependencies
uv run pytest tests/ --cov   # Run tests with coverage
```

### Docker

```bash
just docker-build            # Build development images
just docker-dev              # Interactive ESP-IDF shell
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
- ESP32 firmware projects go in `packages/esp32-projects/<project-name>/`
- Each ESP32 project has its own `CMakeLists.txt`, `main/`, and `sdkconfig.defaults`
- Shared components are planned for `packages/shared-libs/`

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

- `.clang-format` — C/C++ formatting rules
- `.pre-commit-config.yaml` — Pre-commit hook definitions
- `.gitleaks.toml` — Secret scanning allowlist
- `tools/scaffold/new-esp32-project.sh` — New project scaffolding
- `packages/esp32-projects/robocar-docs/` — Robocar coordination justfile and docs
- `docs/flasher/index.html` — ESP Web Tools browser-based firmware flasher page
