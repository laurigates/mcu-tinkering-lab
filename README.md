# MCU Tinkering Lab

> Production-ready embedded systems monorepo for ESP32, STM32, and Arduino platforms with AI-powered robotics projects

[![ESP32 Build](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/esp32-build.yml/badge.svg)](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/esp32-build.yml)
[![Tests](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/test.yml/badge.svg)](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/test.yml)

## 🚀 Quick Start

Get up and running in **under 5 minutes** using Docker, or 30 minutes with native setup.

### Option 1: Docker (Recommended)

```bash
# Clone the repository
git clone https://github.com/laurigates/mcu-tinkering-lab.git
cd mcu-tinkering-lab

# Build Docker images
just docker-build

# Start interactive development shell
just docker-dev

# Inside container: build all projects
just build-all
```

### Option 2: Native Setup

**Prerequisites:**
- ESP-IDF v5.4+ (for ESP32 projects)
- Python 3.11+ (for simulation)
- Git, [just](https://github.com/casey/just), CMake

```bash
# Install ESP-IDF
mkdir -p ~/repos
cd ~/repos
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
git checkout v5.4
./install.sh esp32
source export.sh

# Clone this repository
cd ~
git clone https://github.com/laurigates/mcu-tinkering-lab.git
cd mcu-tinkering-lab

# Install development tools
just install-dev-tools

# Build all projects
just build-all
```

## 📁 Repository Structure

```
mcu-tinkering-lab/
├── packages/
│   ├── esp32-projects/
│   │   ├── robocar-main/           # 🤖 AI robot car controller (Heltec WiFi LoRa 32)
│   │   ├── robocar-camera/         # 📹 ESP32-CAM vision with Claude/Ollama AI
│   │   ├── robocar-simulation/     # 🎮 Python 3.11 physics simulation
│   │   ├── robocar-docs/           # 📚 Documentation and coordination justfile
│   │   ├── esp32cam-llm-telegram/  # 💬 Telegram bot with LLM vision
│   │   ├── esp32-cam-webserver/    # 🌐 Live video streaming server
│   │   └── esp32-cam-i2s-audio/    # 🔊 Camera + audio processing
│   ├── arduino-projects/           # 🚧 Coming soon
│   ├── stm32-projects/             # 🚧 Coming soon
│   └── shared-libs/                # 📦 Shared code libraries (planned)
├── .github/workflows/              # 🔄 CI/CD pipelines
├── tools/                          # 🛠️ Build scripts and utilities
├── docs/                           # 📖 Documentation
└── justfile                        # 🎯 Root build coordination (60+ targets)
```

## 🎯 Featured Projects

### AI-Powered Robot Car (Primary Project)
Dual ESP32 autonomous robot with AI vision and physics simulation.

**Components:**
- **Main Controller:** Heltec WiFi LoRa 32 V1 - Motor control, LEDs, servos
- **Vision System:** ESP32-CAM - Claude/Ollama AI backends, MQTT telemetry
- **Simulation:** Python 3.11 with Pymunk 2D physics engine

**Quick Start:**
```bash
# Build both controllers
just robocar-build-all

# Flash main controller
just robocar-flash-main PORT=/dev/ttyUSB0

# Flash camera (connect GPIO0 to GND first)
just robocar-flash-cam PORT=/dev/ttyUSB1

# Start development workflow (build + flash + monitor)
just robocar-develop-main
```

### ESP32-CAM LLM Telegram Bot
Vision-enabled Telegram bot with Claude or Ollama AI integration.

```bash
just llm-telegram-build
just llm-telegram-flash PORT=/dev/ttyUSB0
just llm-telegram-monitor
```

## 🏗️ Build System

The justfile provides **60+ targets** for all projects:

```bash
# Show all available commands
just --list

# Build commands
just build-all              # Build all projects
just build-esp32            # Build all ESP32 projects
just robocar-build-all      # Build robocar main + camera

# Development workflows (build + flash + monitor)
just robocar-develop-main   # Main controller
just robocar-develop-cam    # Camera module

# Code quality
just lint                   # Run all linters
just format                 # Format all code
just format-check           # Check formatting without modifying

# Docker
just docker-build           # Build development images
just docker-dev             # Interactive development shell
just docker-run CMD="just build-all"  # Run command in container

# Utilities
just list-projects          # Show all projects
just check-environment      # Verify development setup
just clean-all              # Clean all builds
```

## 🧪 Testing

```bash
# Run all tests
just test-all                  # (Coming soon)

# Python simulation tests
cd packages/esp32-projects/robocar-simulation
uv sync
uv run pytest tests/ --cov

# ESP32 host-based tests (planned)
just test-host-all

# Hardware-in-loop tests (planned)
just test-hil-all
```

## 🐳 Docker Development

**Benefits:**
- ✅ Zero manual ESP-IDF setup
- ✅ Consistent environment across platforms
- ✅ Works identically on Windows/Mac/Linux
- ✅ Isolated from host system

**Commands:**
```bash
# Build images
just docker-build

# Interactive shell
just docker-dev

# Run specific commands
just docker-run CMD="just build-all"
just docker-run CMD="just robocar-build-main"

# Manage containers
just docker-up              # Start services in background
just docker-down            # Stop services
just docker-logs            # View logs
just docker-clean           # Remove containers and volumes
```

**USB Device Access (for flashing):**
Uncomment the `devices` section in `docker-compose.yml`:
```yaml
devices:
  - /dev/ttyUSB0:/dev/ttyUSB0
  - /dev/ttyUSB1:/dev/ttyUSB1
```

## 🔧 Development Tools

### Code Quality

**C/C++ Tools:**
- `clang-format` - Code formatting (Google style, 4-space indent)
- `cppcheck` - Static analysis

**Python Tools:**
- `ruff` - Fast linting and formatting
- `mypy` - Type checking
- `pytest` - Testing framework

### Pre-commit Hooks

Automatically run code quality checks before each commit:

```bash
# Install hooks
just install-dev-tools

# Or manually
pip install pre-commit
pre-commit install

# Run hooks manually
pre-commit run --all-files
```

**Checks:**
- ✅ C/C++ formatting (clang-format)
- ✅ Python formatting (ruff)
- ✅ Trailing whitespace
- ✅ YAML validation
- ✅ Credential file detection
- ✅ Build artifact detection

## 📊 CI/CD Pipeline

Automated checks on every push and pull request:

**Build Pipeline** (`.github/workflows/esp32-build.yml`):
- ✅ Build all ESP32 projects in parallel
- ✅ Generate size analysis reports
- ✅ Archive firmware binaries (30-day retention)
- ✅ Check binary size limits for OTA

**Test Pipeline** (`.github/workflows/test.yml`):
- ✅ Pre-commit hook validation
- ✅ Python simulation tests with coverage
- ✅ C/C++ linting (cppcheck)
- ✅ Format checking (clang-format)
- ✅ ESP32 host-based tests (when added)

**Status Badges:**
Add to your PRs to see build status at a glance.

## 🚧 Creating New Projects

### ESP32 Project Template

```bash
# Copy template (manual for now)
cp -r packages/esp32-projects/esp32-cam-webserver packages/esp32-projects/my-new-project
cd packages/esp32-projects/my-new-project

# Update CMakeLists.txt
sed -i 's/esp32-cam-webserver/my-new-project/g' CMakeLists.txt

# Build
just esp32-my-new-project-build
```

**Automated scaffolding tool coming soon!**

## 📚 Documentation

- [Architecture Overview](packages/esp32-projects/robocar-docs/README.md) - System design and communication protocols
- [Hardware Connections](packages/esp32-projects/robocar-docs/hardware-connections.md) - Pin mappings and wiring
- [WiFi Setup](packages/esp32-projects/robocar-main/WIFI_SETUP.md) - Network configuration
- [OTA Updates](packages/esp32-projects/robocar-docs/PARTITION_UPDATE_NOTES.md) - Over-the-air firmware updates
- [Simulation Guide](packages/esp32-projects/robocar-simulation/README.md) - Physics simulation setup

## 🤝 Contributing

We welcome contributions! Please follow these guidelines:

1. **Code Style:**
   - C/C++: Run `just format-c` before committing
   - Python: Run `just format-python` before committing
   - All: Pre-commit hooks will enforce standards

2. **Testing:**
   - Add tests for new features
   - Ensure existing tests pass
   - Aim for >70% coverage

3. **Commit Messages:**
   - Use conventional commits: `feat:`, `fix:`, `docs:`, etc.
   - Be descriptive: "feat: Add WiFi reconnection logic" not "update code"

4. **Pull Requests:**
   - Create feature branches: `feat/my-feature`
   - Ensure CI passes
   - Request review from maintainers

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

## 🛠️ Troubleshooting

### ESP-IDF Not Found
```bash
export IDF_PATH=$HOME/repos/esp-idf
source $IDF_PATH/export.sh
```

### Serial Port Permission Denied
```bash
# Linux
sudo usermod -a -G dialout $USER
# Log out and back in

# Or use sudo
sudo just robocar-flash-main
```

### Docker USB Device Not Found
Ensure `privileged: true` is set in `docker-compose.yml` and devices are mapped correctly.

### Build Fails with "No Space Left"
```bash
# Clean all builds
just clean-all

# In Docker, clean volumes
just docker-clean
```

## 📈 Project Status

| Platform | Status | Projects | Tests | CI/CD |
|----------|--------|----------|-------|-------|
| **ESP32** | ✅ Active | 7 projects | 🚧 In progress | ✅ Automated |
| **Arduino** | 🚧 Planned | 0 | ❌ N/A | ❌ N/A |
| **STM32** | 🚧 Planned | 0 | ❌ N/A | ❌ N/A |
| **Simulation** | ✅ Active | Python 3.11 | ✅ pytest | ✅ Automated |

## 🔗 Links

- **Repository:** https://github.com/laurigates/mcu-tinkering-lab
- **Issues:** https://github.com/laurigates/mcu-tinkering-lab/issues
- **Discussions:** https://github.com/laurigates/mcu-tinkering-lab/discussions
- **ESP-IDF:** https://docs.espressif.com/projects/esp-idf/en/latest/

## 📄 License

This project is licensed under the MIT License - see individual project directories for specific licenses.

## ⭐ Acknowledgments

- **ESP-IDF** by Espressif Systems
- **esp-idf-lib** component library
- **Claude API** and **Ollama** for AI vision
- **Pymunk** for 2D physics simulation
- Open source community

---

**Happy tinkering! 🔧🤖**

For questions or support, please open an issue or discussion on GitHub.
