# ESP32 Robot Car Simulation Makefile

# Configuration - Use Python 3.11 for Swift-sim compatibility
# Swift-sim has asyncio issues with Python 3.13+
PYTHON := python3.11
PIP := pip3.11
SRC_DIR := src
TEST_DIR := tests
CONFIG_DIR := config
REQUIREMENTS := requirements.txt

# Default target
.PHONY: help
help:
	@echo "ESP32 Robot Car Simulation"
	@echo "=========================="
	@echo ""
	@echo "Setup:"
	@echo "  install      Install dependencies"
	@echo "  install-dev  Install development dependencies"
	@echo "  check-deps   Check if dependencies are installed"
	@echo ""
	@echo "Simulation:"
	@echo "  run          Run simulation (headless by default)"
	@echo "  run-visual   Run simulation with GUI window"
	@echo "  run-browser  Run simulation with browser interface"
	@echo "  run-headless Run simulation without visualization"
	@echo "  run-demo     Run demo movement pattern (headless)"
	@echo "  demo-visual  Run visual demo with GUI window"
	@echo "  demo-browser Run demo with browser interface"
	@echo "  run-serial   Run with ESP32 serial connection (headless)"
	@echo "  serial-visual Run with ESP32 + GUI visualization"
	@echo ""
	@echo "Testing:"
	@echo "  test         Run all tests"
	@echo "  test-robot   Test robot model only"
	@echo "  test-comm    Test communication bridge"
	@echo "  test-viz     Test visualization"
	@echo ""
	@echo "Development:"
	@echo "  lint         Run code linting"
	@echo "  format       Format code"
	@echo "  type-check   Run type checking"
	@echo "  clean        Clean temporary files"
	@echo ""
	@echo "Hardware:"
	@echo "  validate     Validate simulation against hardware"
	@echo "  benchmark    Run performance benchmarks"
	@echo ""
	@echo "Quick Start:"
	@echo "  make demo-visual           # Visual demo with GUI window"
	@echo "  make demo-browser          # Demo in web browser"
	@echo "  make serial-visual SERIAL_PORT=/dev/ttyUSB0  # ESP32 + visualization"

# Setup targets
.PHONY: install
install:
	@echo "Installing dependencies..."
	$(PIP) install -r $(REQUIREMENTS)
	@echo "Installation complete!"

.PHONY: install-dev
install-dev: install
	@echo "Installing development dependencies..."
	$(PIP) install pytest pytest-asyncio black flake8 mypy
	@echo "Development setup complete!"

.PHONY: check-deps
check-deps:
	@echo "Checking dependencies..."
	@$(PYTHON) -c "import roboticstoolbox, swift, numpy, scipy, matplotlib, websockets, serial, yaml, trimesh" && \
		echo "✓ All core dependencies available" || \
		(echo "✗ Missing dependencies - run 'make install'" && exit 1)

# Simulation targets
.PHONY: run
run: check-deps
	@echo "Starting ESP32 Robot Car Simulation (headless)..."
	cd $(SRC_DIR) && $(PYTHON) main.py --headless

.PHONY: run-visual
run-visual: check-deps
	@echo "Starting ESP32 Robot Car Simulation with GUI window..."
	cd $(SRC_DIR) && $(PYTHON) main.py --visual

.PHONY: run-browser
run-browser: check-deps
	@echo "Starting ESP32 Robot Car Simulation with browser interface..."
	@echo "Access visualization at: http://localhost:52000"
	cd $(SRC_DIR) && $(PYTHON) main.py --browser

.PHONY: run-headless
run-headless: check-deps
	@echo "Starting simulation in headless mode..."
	cd $(SRC_DIR) && $(PYTHON) main.py --headless

.PHONY: run-demo
run-demo: check-deps
	@echo "Starting demo simulation (headless)..."
	cd $(SRC_DIR) && $(PYTHON) main.py --demo-only --headless

.PHONY: demo-visual
demo-visual: check-deps
	@echo "Starting visual demo with GUI window..."
	@echo "Watch the robot move in the 3D visualization window!"
	cd $(SRC_DIR) && $(PYTHON) main.py --demo-only --visual

.PHONY: demo-browser
demo-browser: check-deps
	@echo "Starting demo with browser interface..."
	@echo "Access visualization at: http://localhost:52000"
	@echo "Watch the robot move in your web browser!"
	cd $(SRC_DIR) && $(PYTHON) main.py --demo-only --browser

.PHONY: run-serial
run-serial: check-deps
	@if [ -z "$(SERIAL_PORT)" ]; then \
		echo "Error: SERIAL_PORT not specified"; \
		echo "Usage: make run-serial SERIAL_PORT=/dev/ttyUSB0"; \
		exit 1; \
	fi
	@echo "Starting simulation with serial connection to $(SERIAL_PORT)..."
	cd $(SRC_DIR) && $(PYTHON) main.py --serial $(SERIAL_PORT) --headless

.PHONY: serial-visual
serial-visual: check-deps
	@if [ -z "$(SERIAL_PORT)" ]; then \
		echo "Error: SERIAL_PORT not specified"; \
		echo "Usage: make serial-visual SERIAL_PORT=/dev/ttyUSB0"; \
		exit 1; \
	fi
	@echo "Starting simulation with ESP32 serial connection and GUI visualization..."
	@echo "Serial port: $(SERIAL_PORT)"
	@echo "Watch the robot move in the 3D window based on ESP32 commands!"
	cd $(SRC_DIR) && $(PYTHON) main.py --serial $(SERIAL_PORT) --visual

# Testing targets
.PHONY: test
test:
	@echo "Running test suite..."
	cd $(TEST_DIR) && $(PYTHON) -m pytest -v

.PHONY: test-robot
test-robot:
	@echo "Testing robot model..."
	cd $(TEST_DIR) && $(PYTHON) -m pytest test_robot_model.py::TestDifferentialDriveRobot -v

.PHONY: test-motor
test-motor:
	@echo "Testing motor dynamics..."
	cd $(TEST_DIR) && $(PYTHON) -m pytest test_robot_model.py::TestDCMotor -v

.PHONY: test-accuracy
test-accuracy:
	@echo "Testing simulation accuracy..."
	cd $(TEST_DIR) && $(PYTHON) -m pytest test_robot_model.py::TestSimulationAccuracy -v

.PHONY: test-comm
test-comm:
	@echo "Testing communication bridge..."
	@echo "Note: Communication tests require manual setup"
	cd $(SRC_DIR) && $(PYTHON) -c "from communication_bridge import ESP32CommunicationBridge; print('Communication bridge imports OK')"

.PHONY: test-viz
test-viz:
	@echo "Testing visualization..."
	cd $(SRC_DIR) && $(PYTHON) -c "from swift_visualizer import SwiftSimulation, HAS_SWIFT; print(f'Swift available: {HAS_SWIFT}')"

# Development targets
.PHONY: lint
lint:
	@echo "Running code linting..."
	flake8 $(SRC_DIR) $(TEST_DIR) --max-line-length=100 --ignore=E203,W503

.PHONY: format
format:
	@echo "Formatting code..."
	black $(SRC_DIR) $(TEST_DIR) --line-length=100

.PHONY: type-check
type-check:
	@echo "Running type checking..."
	mypy $(SRC_DIR) --ignore-missing-imports

.PHONY: clean
clean:
	@echo "Cleaning temporary files..."
	find . -type f -name "*.pyc" -delete
	find . -type d -name "__pycache__" -delete
	find . -type d -name "*.egg-info" -exec rm -rf {} +
	find . -type f -name ".coverage" -delete
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name ".mypy_cache" -exec rm -rf {} +

# Hardware validation targets
.PHONY: validate
validate:
	@if [ -z "$(SERIAL_PORT)" ]; then \
		echo "Error: SERIAL_PORT not specified"; \
		echo "Usage: make validate SERIAL_PORT=/dev/ttyUSB0"; \
		exit 1; \
	fi
	@echo "Running hardware validation..."
	@echo "Connecting to ESP32 at $(SERIAL_PORT)..."
	cd $(SRC_DIR) && $(PYTHON) main.py --serial $(SERIAL_PORT) --validate

.PHONY: benchmark
benchmark:
	@echo "Running performance benchmarks..."
	cd $(TEST_DIR) && $(PYTHON) -m pytest test_robot_model.py::TestSimulationAccuracy::test_simulation_stability -v -s

# Quick test of specific components
.PHONY: test-quick
test-quick:
	@echo "Running quick smoke tests..."
	cd $(SRC_DIR) && $(PYTHON) -c "from robot_model import DifferentialDriveRobot; print('✓ Robot model')"
	cd $(SRC_DIR) && $(PYTHON) -c "from communication_bridge import ESP32CommunicationBridge; print('✓ Communication bridge')"
	@$(PYTHON) -c "import numpy, scipy, matplotlib, yaml; print('✓ Core dependencies')"

# Demo scripts
.PHONY: demo-basic
demo-basic:
	@echo "Running basic robot demo..."
	cd $(SRC_DIR) && $(PYTHON) robot_model.py

.PHONY: demo-comm
demo-comm:
	@echo "Running communication demo..."
	cd $(SRC_DIR) && $(PYTHON) communication_bridge.py

.PHONY: demo-viz
demo-viz:
	@echo "Running visualization demo..."
	cd $(SRC_DIR) && $(PYTHON) swift_visualizer.py

# Configuration validation
.PHONY: check-config
check-config:
	@echo "Validating configuration..."
	@$(PYTHON) -c "import yaml; yaml.safe_load(open('$(CONFIG_DIR)/robot_config.yaml')); print('✓ Configuration valid')"

# Information targets
.PHONY: info
info:
	@echo "ESP32 Robot Car Simulation Information"
	@echo "====================================="
	@echo "Python version: $$($(PYTHON) --version)"
	@echo "Working directory: $$(pwd)"
	@echo "Configuration: $(CONFIG_DIR)/robot_config.yaml"
	@echo ""
	@echo "Available serial ports:"
	@$(PYTHON) -c "import serial.tools.list_ports; [print(f'  {p.device}: {p.description}') for p in serial.tools.list_ports.comports()]" 2>/dev/null || echo "  pyserial not installed"

.PHONY: status
status:
	@echo "Simulation Status Check"
	@echo "======================"
	@echo -n "Robot model: "
	@cd $(SRC_DIR) && $(PYTHON) -c "from robot_model import DifferentialDriveRobot; print('✓ OK')" 2>/dev/null || echo "✗ Error"
	@echo -n "Communication bridge: "
	@cd $(SRC_DIR) && $(PYTHON) -c "from communication_bridge import ESP32CommunicationBridge; print('✓ OK')" 2>/dev/null || echo "✗ Error"
	@echo -n "Swift visualizer: "
	@cd $(SRC_DIR) && $(PYTHON) -c "from swift_visualizer import HAS_SWIFT; print('✓ Available' if HAS_SWIFT else '⚠ Not available')" 2>/dev/null || echo "✗ Error"
	@echo -n "Configuration: "
	@$(PYTHON) -c "import yaml; yaml.safe_load(open('$(CONFIG_DIR)/robot_config.yaml')); print('✓ Valid')" 2>/dev/null || echo "✗ Invalid"

# All-in-one setup
.PHONY: setup
setup: install check-deps check-config
	@echo ""
	@echo "Setup complete! Try:"
	@echo "  make run          # Start simulation"
	@echo "  make test         # Run tests"
	@echo "  make info         # Show system info"
