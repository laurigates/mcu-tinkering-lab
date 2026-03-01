# MCU Tinkering Lab — Unified Build System
# Run `just --list` to see available recipes

mod robocar 'packages/esp32-projects/robocar-docs'
mod robocar-main 'packages/esp32-projects/robocar-main'
mod robocar-camera 'packages/esp32-projects/robocar-camera'
mod sim 'packages/esp32-projects/robocar-simulation'
mod audiobook 'packages/esp32-projects/audiobook-player'
mod wireguard 'packages/esp32-projects/esp32-wireguard-ha-example'
mod kids-audio 'packages/esp32-projects/kids-audio-toy'

idf_path := env("IDF_PATH", home_directory() + "/repos/esp-idf")
idf_version := "v5.3.2"
idf_targets := "esp32,esp32s3,esp32c3"
idf_github_assets := "dl.espressif.com/github_assets"
port := env("PORT", "/dev/cu.usbserial-0001")

robocar_sim_dir := "packages/esp32-projects/robocar-simulation"
esp32_webserver_dir := "packages/esp32-projects/esp32-cam-webserver"
esp32_audio_dir := "packages/esp32-projects/esp32-cam-i2s-audio"
esp32_llm_telegram_dir := "packages/esp32-projects/esp32cam-llm-telegram"

default:
    @just --list

# Verify development environment setup
[group: "environment"]
check-environment:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "ESP-IDF Framework:"
    if [ -d "{{idf_path}}" ]; then
        echo "  ESP-IDF found at {{idf_path}}"
    else
        echo "  ESP-IDF NOT found at {{idf_path}}"
        echo "  Run 'just setup-idf' to install"
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
    [ -d "packages/esp32-projects/robocar-main" ] && [ -d "packages/esp32-projects/robocar-camera" ] \
        && echo "  Robocar projects found" || echo "  Robocar projects missing"
    [ -d "packages/esp32-projects" ] \
        && echo "  ESP32 packages directory found" || echo "  ESP32 packages missing"

[private]
check-idf:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -d "{{idf_path}}" ]; then
        echo "Error: ESP-IDF not found at {{idf_path}}"
        echo "Run 'just setup-idf' to install"
        exit 1
    fi

# Show installed ESP-IDF version
[group: "environment"]
check-idf-version:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ -d "{{idf_path}}" ]; then
        VERSION=$(cd {{idf_path}} && git describe --tags 2>/dev/null || echo "unknown")
        echo "ESP-IDF Version:    $VERSION"
        echo "Install Path:       {{idf_path}}"
        echo "Configured Targets: {{idf_targets}}"
    else
        echo "ESP-IDF not installed at {{idf_path}}"
        echo "Run 'just setup-idf' to install"
    fi

# Install ESP-IDF (clones repo and installs toolchain)
[group: "setup"]
setup-idf:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "ESP-IDF Setup — version {{idf_version}}, targets: {{idf_targets}}"
    echo ""
    if [ -d "{{idf_path}}" ]; then
        echo "ESP-IDF already installed at {{idf_path}}"
        echo "To update: just update-idf"
    else
        echo "Installing ESP-IDF {{idf_version}}..."
        mkdir -p "$(dirname {{idf_path}})"
        git clone --recursive -b {{idf_version}} https://github.com/espressif/esp-idf.git {{idf_path}}
        echo "Installing toolchain for targets: {{idf_targets}}..."
        cd {{idf_path}} && IDF_GITHUB_ASSETS="{{idf_github_assets}}" ./install.sh {{idf_targets}}
        echo ""
        echo "ESP-IDF {{idf_version}} installed successfully!"
        echo ""
        echo "Add to your shell profile:"
        echo "  alias get_idf='. {{idf_path}}/export.sh'"
    fi

# Update ESP-IDF to the configured version
[group: "setup"]
update-idf:
    #!/usr/bin/env bash
    set -euo pipefail
    if [ ! -d "{{idf_path}}" ]; then
        echo "ESP-IDF not found — run 'just setup-idf' first"
        exit 1
    fi
    echo "Fetching updates..."
    cd {{idf_path}} && git fetch --all --tags
    CURRENT=$(cd {{idf_path}} && git describe --tags 2>/dev/null || echo "unknown")
    if [ "$CURRENT" = "{{idf_version}}" ]; then
        echo "Already at version {{idf_version}}"
    else
        echo "Switching from $CURRENT to {{idf_version}}..."
        cd {{idf_path}} && git checkout {{idf_version}} && git submodule update --init --recursive
        echo "Reinstalling toolchain..."
        cd {{idf_path}} && IDF_GITHUB_ASSETS="{{idf_github_assets}}" ./install.sh {{idf_targets}}
        echo "Updated to ESP-IDF {{idf_version}}"
    fi

# Full dev environment: ESP-IDF + dev tools
[group: "setup"]
setup-all: setup-idf install-dev-tools
    @echo ""
    @echo "Full development environment setup complete!"
    @echo ""
    @echo "Next steps:"
    @echo "  1. Add to your shell profile:"
    @echo "     alias get_idf='. {{idf_path}}/export.sh'"
    @echo "  2. just check-environment   # Verify setup"
    @echo "  3. just list-projects       # See available projects"
    @echo "  4. just robocar-build-all   # Build the robocar"

# Install development tools and pre-commit hooks
[group: "setup"]
install-dev-tools:
    #!/usr/bin/env bash
    set -euo pipefail
    pip install --upgrade pip
    pip install pre-commit==3.8.0 ruff==0.6.9 mypy==1.11.2 pytest==8.3.3 pytest-cov==5.0.0 uv==0.4.18
    pre-commit install
    command -v clang-format >/dev/null 2>&1 || echo "clang-format not found — install with: brew install clang-format"
    command -v cppcheck >/dev/null 2>&1 || echo "cppcheck not found — install with: brew install cppcheck"
    echo "Development tools installed"

# Build both main controller and camera module
[group: "robocar"]
robocar-build-all: check-idf
    just robocar::build-all

# Build main controller only
[group: "robocar"]
robocar-build-main: check-idf
    just robocar::build-main

# Build camera module only
[group: "robocar"]
robocar-build-cam: check-idf
    just robocar::build-cam

# Flash both controllers (prompts between each)
[group: "robocar"]
robocar-flash-all: check-idf
    #!/usr/bin/env bash
    set -euo pipefail
    just robocar::flash-main
    echo "Now connect GPIO0 to GND on ESP32-CAM, then press Enter..."
    read -r
    just robocar::flash-cam

# Flash main controller
[group: "robocar"]
robocar-flash-main: check-idf
    just robocar::flash-main

# Flash camera module (GPIO0 must be connected to GND)
[group: "robocar"]
robocar-flash-cam: check-idf
    just robocar::flash-cam

# Build, flash, and monitor main controller
[group: "robocar"]
robocar-develop-main: check-idf
    just robocar::develop-main

# Build, flash, and monitor camera module
[group: "robocar"]
robocar-develop-cam: check-idf
    just robocar::develop-cam

# Monitor main controller
[group: "robocar"]
robocar-monitor-main: check-idf
    just robocar::monitor-main

# Monitor camera module
[group: "robocar"]
robocar-monitor-cam: check-idf
    just robocar::monitor-cam

# Setup camera credentials file
[group: "robocar"]
robocar-credentials:
    just robocar::credentials

# Clean robocar builds
[group: "robocar"]
robocar-clean:
    just robocar::clean-all

# Show robocar system information
[group: "robocar"]
robocar-info:
    just robocar::info

# Shortcuts
dev-main: robocar-develop-main
dev-cam: robocar-develop-cam

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
    find packages/esp32-projects -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
        ! -path "*/managed_components/*" \
        ! -path "*/components/esp-idf-lib/*" \
        ! -path "*/build/*" \
        ! -path "*/.venv/*" \
        -print0 | \
        xargs -0 cppcheck \
            --enable=warning,style,performance,portability \
            --suppress=missingIncludeSystem \
            --suppress=unmatchedSuppression \
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
    cd {{robocar_sim_dir}} && ruff check .
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
    find packages/esp32-projects -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
        ! -path "*/managed_components/*" \
        ! -path "*/components/esp-idf-lib/*" \
        ! -path "*/build/*" \
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
    cd {{robocar_sim_dir}} && ruff format .
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
    find packages/esp32-projects -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
        ! -path "*/managed_components/*" \
        ! -path "*/components/esp-idf-lib/*" \
        ! -path "*/build/*" \
        -print0 | \
        xargs -0 clang-format --dry-run --Werror --style=file
    echo "C/C++ formatting check passed"

# Check Python formatting without modifying
[group: "quality"]
format-check-python:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v ruff >/dev/null 2>&1 || { echo "Warning: ruff not found"; exit 0; }
    cd {{robocar_sim_dir}} && ruff format --check .
    echo "Python formatting check passed"

# Build all projects
[group: "build"]
build-all: robocar-build-all
    @echo "All projects built"

# Clean all project builds
[confirm("Clean all build directories?")]
[group: "clean"]
clean-all: robocar-clean
    @echo "All project builds cleaned"

# List all projects in the monorepo
[group: "info"]
list-projects:
    #!/usr/bin/env bash
    set -euo pipefail
    echo "MCU Tinkering Lab — Project Inventory"
    echo ""
    echo "AI-Powered Robot Car (Primary):"
    if [ -d "packages/esp32-projects/robocar-main" ] && [ -d "packages/esp32-projects/robocar-camera" ]; then
        echo "  Dual ESP32 autonomous robot with AI vision"
        echo "    Main Controller: Heltec WiFi LoRa 32 V1"
        echo "    Vision System:   ESP32-CAM with Claude/Ollama AI"
        echo "    Simulation:      Python 3.11 physics simulation"
    else
        echo "  Not found"
    fi
    echo ""
    echo "ESP32 Package Projects:"
    [ -d "{{esp32_webserver_dir}}" ]    && echo "  esp32-cam-webserver    — Live video streaming web server"    || true
    [ -d "{{esp32_audio_dir}}" ]        && echo "  esp32-cam-i2s-audio    — Camera + I2S audio processing"      || true
    [ -d "{{esp32_llm_telegram_dir}}" ] && echo "  esp32cam-llm-telegram  — AI vision with Telegram bot"        || true
    echo ""
    echo "Platform Directories:"
    [ -d "packages/arduino-projects" ] && echo "  Arduino projects" || echo "  Arduino projects (empty)"
    [ -d "packages/stm32-projects" ]   && echo "  STM32 projects"   || echo "  STM32 projects (empty)"
    [ -d "packages/shared-libs" ]      && echo "  Shared libraries" || echo "  Shared libraries (empty)"

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
    @echo "  just robocar-info          # Detailed robocar system info"
    @echo "  just list-projects         # Show all available projects"
    @echo "  just robocar-build-all     # Build both robocar modules"
    @echo "  just robocar-develop-main  # Start developing main controller"

# Build Docker development images
[group: "docker"]
docker-build:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v docker >/dev/null 2>&1 || { echo "Error: Docker not found"; exit 1; }
    docker-compose build

# Start interactive shell in ESP-IDF Docker container
[group: "docker"]
docker-dev:
    #!/usr/bin/env bash
    set -euo pipefail
    command -v docker >/dev/null 2>&1 || { echo "Error: Docker not found"; exit 1; }
    docker-compose run --rm esp-idf

# Start Docker services in background
[group: "docker"]
docker-up:
    docker-compose up -d

# Stop Docker services
[group: "docker"]
docker-down:
    docker-compose down

# Clean Docker containers and volumes (images preserved)
[group: "docker"]
docker-clean:
    docker-compose down -v
    @echo "Containers and volumes removed. Images preserved."

# Show Docker service logs
[group: "docker"]
docker-logs:
    docker-compose logs -f

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
