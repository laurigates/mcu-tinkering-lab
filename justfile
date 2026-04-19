# MCU Tinkering Lab — Unified Build System
# Run `just --list` to see available recipes

mod robocar 'packages/robocar/docs'
mod robocar-main 'packages/robocar/main'
mod robocar-camera 'packages/robocar/camera'
mod robocar-unified 'packages/robocar/unified'
mod sim 'packages/robocar/simulation'
mod audiobook 'packages/audio/audiobook-player'
mod wireguard 'packages/networking/wireguard-ha'
mod kids-audio 'packages/audio/kids-audio-toy'
mod xbox 'packages/input-gaming/xbox-switch-bridge'
mod troubleshooter 'packages/networking/it-troubleshooter'
mod webserver 'packages/camera-vision/cam-webserver'
mod i2s-audio 'packages/camera-vision/cam-i2s-audio'
mod telegram 'packages/camera-vision/llm-telegram'
mod wifitest 'packages/networking/wifitest'
mod switch-proxy 'packages/input-gaming/switch-usb-proxy'
mod switch-probe 'tools/switch-controller-usb-test'
mod synth 'packages/audio/gamepad-synth'
mod gemini-vision 'packages/camera-vision/gemini-vision'
mod thinkpack-mesh-demo 'packages/thinkpack/mesh-demo'
mod thinkpack-glowbug 'packages/thinkpack/glowbug'
mod thinkpack-boombox 'packages/thinkpack/boombox'
mod thinkpack-brainbox 'packages/thinkpack/brainbox'

# Auto-detect ESP32-S3 USB-Serial-JTAG by Espressif VID; override with S3_PORT env var
s3_port := env("S3_PORT", `tools/detect-esp32s3-port.sh --quiet 2>/dev/null || true`)

port := env("PORT", "/dev/cu.usbserial-0001")

robocar_sim_dir := "packages/robocar/simulation"

default:
    @just --list

# Verify development environment setup
[group: "environment"]
check-environment:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Container Engine:"
    if command -v docker >/dev/null 2>&1; then
        echo "  Docker: $(docker --version)"
        if docker compose version >/dev/null 2>&1; then
            echo "  Compose: $(docker compose version --short)"
        else
            echo "  Compose: NOT found — install Docker Compose"
        fi
    else
        echo "  Docker: NOT found — install Docker Desktop"
    fi
    echo ""
    echo "Serial Port:"
    if [ -e "{{port}}" ]; then
        echo "  Serial port available: {{port}}"
    else
        echo "  Serial port not found: {{port}}"
        echo "  Connect device or set PORT variable"
    fi
    echo ""
    echo "Project Structure:"
    [ -d "packages/robocar/main" ] && [ -d "packages/robocar/camera" ] \
        && echo "  Robocar projects found" || echo "  Robocar projects missing"
    [ -d "packages" ] \
        && echo "  packages/ directory found" || echo "  packages/ directory missing"

# Full dev environment: Docker images + dev tools
[group: "setup"]
setup-all: docker-build install-dev-tools
    @echo ""
    @echo "Full development environment setup complete!"
    @echo ""
    @echo "Next steps:"
    @echo "  1. just check-environment        # Verify setup"
    @echo "  2. just list-projects             # See available projects"
    @echo "  3. just robocar::build-all        # Build the robocar"

# Install development tools and pre-commit hooks
[group: "setup"]
install-dev-tools:
    #!/usr/bin/env bash
    set -euo pipefail
    pip install --upgrade pip
    pip install pre-commit ruff ty pytest pytest-cov uv
    pre-commit install
    command -v clang-format >/dev/null 2>&1 || echo "clang-format not found — install with: brew install clang-format"
    command -v cppcheck >/dev/null 2>&1 || echo "cppcheck not found — install with: brew install cppcheck"
    echo "Development tools installed"

##########
# ESP32-S3 Device (shared across S3 projects)
##########

# List all connected MCU and USB-serial devices
[group: "device"]
list-devices:
    python3 tools/list-usb-devices.py

# Auto-detect ESP32-S3 USB-Serial-JTAG port
[group: "device"]
detect-s3-port:
    tools/detect-esp32s3-port.sh

# Reset ESP32-S3 board via USB-Serial-JTAG CDC control signals
[group: "device"]
reset-s3 port=s3_port:
    tools/esp32s3-reset.sh "{{port}}"

# Serial monitor for ESP32-S3 via USB-Serial-JTAG (Ctrl-C to stop)
[group: "device"]
monitor-s3 port=s3_port:
    tools/esp32s3-monitor.sh "{{port}}"

# Scan for nearby WiFi networks (optional SSID for targeted scan)
[group: "device"]
wifi-scan ssid="":
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Scanning for WiFi networks..."
    echo ""
    if [ -n "{{ssid}}" ]; then
        python3 tools/wifi-scan.py "{{ssid}}"
    else
        python3 tools/wifi-scan.py
    fi
    echo ""
    echo "Current connection:"
    /usr/sbin/networksetup -getairportnetwork en0 2>/dev/null || echo "  (unknown)"

# Connect to a WiFi network
[group: "device"]
wifi-connect ssid password:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Connecting to {{ssid}}..."
    if /usr/sbin/networksetup -setairportnetwork en0 "{{ssid}}" "{{password}}" 2>&1 | grep -q "Could not find"; then
        echo "FAILED: Network '{{ssid}}' not found"
        exit 1
    fi
    echo "Connected! IP: $(ipconfig getifaddr en0 2>/dev/null || echo 'pending...')"

# Listen for UDP log broadcast from an ESP32 device
[group: "device"]
log-listen udp_port="4444":
    #!/usr/bin/env bash
    set -euo pipefail
    echo "Listening for UDP logs on port {{udp_port}} — Ctrl-C to stop"
    echo ""
    exec socat -u UDP-RECV:{{udp_port}} -

# Run all linters (C/C++ and Python)
[group: "quality"]
lint: lint-c lint-python
    @echo "All lint checks passed"

# Lint C/C++ code with cppcheck
[group: "quality"]
lint-c:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v cppcheck >/dev/null 2>&1 || { echo "Error: cppcheck not found — brew install cppcheck"; exit 1; }
    mkdir -p tmp
    find packages -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
        ! -path "*/managed_components/*" \
        ! -path "*/components/esp-idf-lib/*" \
        ! -path "*/external/*" \
        ! -path "*/build/*" \
        ! -path "*/.esphome/*" \
        ! -path "*/.venv/*" \
        -print0 | \
        xargs -0 cppcheck \
            --enable=warning,style,performance,portability \
            --suppress=missingIncludeSystem \
            --suppress=unmatchedSuppression \
            --suppress=unusedStructMember \
            --inline-suppr \
            --error-exitcode=1 \
            --template=gcc \
            2>&1 | tee tmp/cppcheck-report.txt
    echo "C/C++ lint checks passed"

# Lint Python code with ruff
[group: "quality"]
lint-python:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v ruff >/dev/null 2>&1 || { echo "Warning: ruff not found — pip install ruff"; exit 0; }
    ruff check .
    echo "Python lint checks passed"

# Format all code (C/C++ and Python)
[group: "quality"]
format: format-c format-python
    @echo "All code formatting complete"

# Format C/C++ code with clang-format
[group: "quality"]
format-c:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v clang-format >/dev/null 2>&1 || { echo "Warning: clang-format not found — brew install clang-format"; exit 0; }
    find packages -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
        ! -path "*/managed_components/*" \
        ! -path "*/components/esp-idf-lib/*" \
        ! -path "*/external/*" \
        ! -path "*/build/*" \
        ! -path "*/.esphome/*" \
        ! -path "*/.venv/*" \
        -print0 | \
        xargs -0 clang-format -i --style=file
    echo "C/C++ code formatted"

# Format Python code with ruff
[group: "quality"]
format-python:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v ruff >/dev/null 2>&1 || { echo "Warning: ruff not found"; exit 0; }
    ruff format .
    echo "Python code formatted"

# Check formatting without modifying files
[group: "quality"]
format-check: format-check-c format-check-python
    @echo "All format checks passed"

# Check C/C++ formatting without modifying
[group: "quality"]
format-check-c:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v clang-format >/dev/null 2>&1 || { echo "Error: clang-format not found"; exit 1; }
    find packages -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
        ! -path "*/managed_components/*" \
        ! -path "*/components/esp-idf-lib/*" \
        ! -path "*/external/*" \
        ! -path "*/build/*" \
        ! -path "*/.esphome/*" \
        ! -path "*/.venv/*" \
        -print0 | \
        xargs -0 clang-format --dry-run --Werror --style=file
    echo "C/C++ formatting check passed"

# Check Python formatting without modifying
[group: "quality"]
format-check-python:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v ruff >/dev/null 2>&1 || { echo "Warning: ruff not found"; exit 0; }
    ruff format --check .
    echo "Python formatting check passed"

# Build robocar projects (use `just <module>::build` for other projects)
[group: "build"]
build-all:
    just robocar::build-all
    @echo "Robocar projects built. Other projects: just <module>::build"

# Clean all project builds
[confirm("Clean all build directories?")]
[group: "clean"]
clean-all:
    just robocar::clean-all
    @echo "All project builds cleaned"

# List all projects in the monorepo
[group: "info"]
list-projects:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "MCU Tinkering Lab — Project Inventory"
    echo ""
    echo "AI-Powered Robot Car (Primary):"
    if [ -d "packages/robocar/main" ] && [ -d "packages/robocar/camera" ]; then
        echo "  Dual ESP32 autonomous robot with AI vision"
        echo "    Main Controller: Heltec WiFi LoRa 32 V1"
        echo "    Vision System:   ESP32-CAM with Claude/Ollama AI"
        echo "    Simulation:      Python 3.11 physics simulation"
    else
        echo "  Not found"
    fi
    echo ""
    echo "ESP32 Package Projects:"
    [ -d "packages/camera-vision/cam-webserver" ]  && echo "  esp32-cam-webserver         — Live video streaming web server"         || true
    [ -d "packages/camera-vision/cam-i2s-audio" ]  && echo "  esp32-cam-i2s-audio         — Camera + I2S audio processing"           || true
    [ -d "packages/camera-vision/llm-telegram" ] && echo "  esp32cam-llm-telegram       — AI vision with Telegram bot"             || true
    [ -d "packages/input-gaming/xbox-switch-bridge" ]          && echo "  xbox-switch-bridge          — Xbox BLE to Switch USB bridge"           || true
    [ -d "packages/networking/it-troubleshooter" ]           && echo "  it-troubleshooter           — IT troubleshooting assistant"            || true
    [ -d "packages/input-gaming/switch-usb-proxy" ]            && echo "  switch-usb-proxy            — Switch USB protocol proxy"               || true
    [ -d "packages/networking/wifitest" ]              && echo "  esp32-wifitest              — WiFi AP test firmware"                   || true
    [ -d "packages/audio/kids-audio-toy" ]              && echo "  kids-audio-toy              — Potentiometer-controlled audio toy"      || true
    [ -d "packages/audio/audiobook-player" ]            && echo "  audiobook-player            — ESPHome audiobook player"                || true
    [ -d "packages/networking/wireguard-ha" ]  && echo "  esp32-wireguard-ha-example  — WireGuard + Home Assistant (ESPHome)"    || true
    echo ""
    echo "Use 'just <module>::build' to build individual projects."
    echo "Module names: just --list --list-submodules"

# Show system information
[group: "info"]
info: check-environment
    @echo ""
    @echo "MCU Tinkering Lab"
    @echo ""
    @echo "Primary Features:"
    @echo "  AI-powered autonomous robot car with dual ESP32 architecture"
    @echo "  Pluggable AI backends (Claude API, Ollama self-hosted)"
    @echo "  Structured I2C communication protocol between controllers"
    @echo "  Real-time computer vision and scene analysis"
    @echo ""
    @echo "Quick Start:"
    @echo "  just robocar::info          # Detailed robocar system info"
    @echo "  just list-projects          # Show all available projects"
    @echo "  just robocar::build-all     # Build both robocar modules"
    @echo "  just robocar::develop-main  # Start developing main controller"

# Build Docker development images
[group: "docker"]
docker-build:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v docker >/dev/null 2>&1 || { echo "Error: Docker not found"; exit 1; }
    docker compose build

# Start interactive shell in ESP-IDF Docker container
[group: "docker"]
docker-dev:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v docker >/dev/null 2>&1 || { echo "Error: Docker not found"; exit 1; }
    docker compose run --rm esp-idf

# Start Docker services in background
[group: "docker"]
docker-up:
    docker compose up -d

# Stop Docker services
[group: "docker"]
docker-down:
    docker compose down

# Clean Docker containers and volumes (images preserved)
[group: "docker"]
docker-clean:
    docker compose down -v
    @echo "Containers and volumes removed. Images preserved."

# Show Docker service logs
[group: "docker"]
docker-logs:
    docker compose logs -f

##########
# NotebookLM knowledge base
##########

# Audit curated NotebookLM notebooks: source counts + recent ADR/PRD changes
[group: "notebooks"]
notebooks-status:
    tools/notebooks-status.sh

##########
# Git
##########

# Check for credential files in the git staging area
[group: "git"]
git-check-credentials:
    #!/usr/bin/env bash
    set -euo pipefail
    if git diff --cached --name-only | grep -E "(credentials\.h|\.(key|secret|token))" > /dev/null 2>&1; then
        echo "Warning: Potential credentials files in staging area:"
        git diff --cached --name-only | grep -E "(credentials\.h|\.(key|secret|token))"
        echo "These files should be in .gitignore"
        exit 1
    else
        echo "No credential files in staging area"
    fi
