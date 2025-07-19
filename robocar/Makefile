# ESP32 Robocar Project Makefile
# ESP-IDF framework build automation for dual ESP32 robot car

# Configuration
IDF_PATH ?= $(HOME)/repos/esp-idf
PORT ?= /dev/cu.usbserial-0001
CAM_PORT ?= $(PORT)
MAIN_PORT ?= $(PORT)

# Colors for output
RED = \033[0;31m
GREEN = \033[0;32m
YELLOW = \033[1;33m
BLUE = \033[0;34m
CYAN = \033[0;36m
NC = \033[0m # No Color

# Default target shows help
.DEFAULT_GOAL := help

help:
	@echo "$(CYAN)ESP32 AI-Powered Robot Car$(NC)"
	@echo "ESP-IDF framework build automation for dual ESP32 system"
	@echo ""
	@echo "$(GREEN)Build Commands:$(NC)"
	@echo "  make build-main      - Build main controller (Heltec WiFi LoRa 32)"
	@echo "  make build-cam       - Build camera module (ESP32-CAM)"
	@echo "  make build-all       - Build both projects"
	@echo ""
	@echo "$(GREEN)Flash Commands:$(NC)"
	@echo "  make flash-main      - Flash main controller"
	@echo "  make flash-cam       - Flash camera module (requires GPIO0->GND)"
	@echo ""
	@echo "$(GREEN)Monitor Commands:$(NC)"
	@echo "  make monitor-main    - Monitor main controller serial output"
	@echo "  make monitor-cam     - Monitor camera module serial output"
	@echo ""
	@echo "$(GREEN)Development Workflow:$(NC)"
	@echo "  make develop-main    - Build, flash, and monitor main controller"
	@echo "  make develop-cam     - Build, flash, and monitor camera module"
	@echo ""
	@echo "$(GREEN)AI Backend Commands:$(NC)"
	@echo "  make ollama-start    - Start Ollama server with automatic mDNS advertising"
	@echo "  make ollama-advertise - Advertise Ollama via mDNS (standalone)"
	@echo "  make ollama-setup    - Start and advertise Ollama service (legacy)"
	@echo "  make ollama-stop     - Stop Ollama server and mDNS services"
	@echo ""
	@echo "$(GREEN)Simulation Commands:$(NC)"
	@echo "  make sim-start       - Start ESP32 robot car simulation"
	@echo "  make sim-headless    - Start simulation without visualization"
	@echo "  make sim-demo        - Run demo movement simulation"
	@echo "  make sim-connect     - Connect simulation to ESP32 hardware"
	@echo "  make sim-test        - Run simulation test suite"
	@echo ""
	@echo "$(GREEN)Utility Commands:$(NC)"
	@echo "  make clean-main      - Clean main controller build"
	@echo "  make clean-cam       - Clean camera build"
	@echo "  make clean-all       - Clean all builds"
	@echo "  make credentials     - Setup camera credentials file"
	@echo "  make info            - Show system configuration"
	@echo ""
	@echo "$(YELLOW)Configuration:$(NC)"
	@echo "  Main Controller Port: $(MAIN_PORT)"
	@echo "  Camera Module Port:   $(CAM_PORT)"
	@echo "  ESP-IDF Path:         $(IDF_PATH)"

# === ESP-IDF Framework Targets ===

check-idf:
	@if [ ! -d "$(IDF_PATH)" ]; then \
		echo "$(RED)Error: ESP-IDF not found at $(IDF_PATH)$(NC)"; \
		echo "$(YELLOW)Please install ESP-IDF or set IDF_PATH variable$(NC)"; \
		exit 1; \
	fi

build-main: check-idf
	@echo "$(BLUE)Building main controller (ESP-IDF)...$(NC)"
	@cd idf-robocar && $(MAKE) build

build-cam: check-idf
	@echo "$(BLUE)Building camera module (ESP-IDF)...$(NC)"
	@cd esp32-cam-idf && $(MAKE) build

build-all: build-main build-cam

flash-main: check-idf
	@echo "$(BLUE)Flashing main controller (ESP-IDF)...$(NC)"
	@cd idf-robocar && $(MAKE) flash PORT=$(MAIN_PORT)

flash-cam: check-idf
	@echo "$(BLUE)Flashing camera module (ESP-IDF)...$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for programming$(NC)"
	@cd esp32-cam-idf && $(MAKE) flash PORT=$(CAM_PORT)

monitor-main: check-idf
	@echo "$(BLUE)Monitoring main controller...$(NC)"
	@echo "$(YELLOW)Press Ctrl+] to exit monitor$(NC)"
	@cd idf-robocar && $(MAKE) monitor PORT=$(MAIN_PORT)

monitor-cam: check-idf
	@echo "$(BLUE)Monitoring camera module...$(NC)"
	@echo "$(YELLOW)Press Ctrl+] to exit monitor$(NC)"
	@cd esp32-cam-idf && $(MAKE) monitor PORT=$(CAM_PORT)

clean-main: check-idf
	@cd idf-robocar && $(MAKE) clean

clean-cam: check-idf
	@cd esp32-cam-idf && $(MAKE) clean

clean-all: clean-main clean-cam

credentials:
	@echo "$(BLUE)Setting up camera credentials...$(NC)"
	@cd esp32-cam-idf && $(MAKE) credentials

develop-main: build-main
	@echo "$(BLUE)Starting development workflow for main controller...$(NC)"
	@echo "$(YELLOW)This will flash and monitor. Press Ctrl+] to exit monitor.$(NC)"
	@cd idf-robocar && $(MAKE) flash-monitor PORT=$(MAIN_PORT)

develop-cam: build-cam
	@echo "$(BLUE)Starting development workflow for camera module...$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for programming$(NC)"
	@echo "$(YELLOW)This will flash and monitor. Press Ctrl+] to exit monitor.$(NC)"
	@cd esp32-cam-idf && $(MAKE) flash-monitor PORT=$(CAM_PORT)

# === AI Backend Targets ===

ollama-start:
	@echo "$(BLUE)Starting Ollama server with automatic service discovery...$(NC)"
	@echo "$(YELLOW)Starting Ollama with host binding for network access$(NC)"
	OLLAMA_HOST=0.0.0.0 ollama serve &
	@echo "$(GREEN)Ollama server started on 0.0.0.0:11434$(NC)"
	@echo "$(BLUE)Advertising Ollama service via mDNS...$(NC)"
	@if command -v dns-sd >/dev/null 2>&1; then \
		echo "$(YELLOW)Using dns-sd (macOS)$(NC)"; \
		dns-sd -R "Ollama AI" _ollama._tcp . 11434 path=/api/generate & \
	elif command -v avahi-publish-service >/dev/null 2>&1; then \
		echo "$(YELLOW)Using avahi-publish-service (Linux)$(NC)"; \
		avahi-publish-service "Ollama AI" _ollama._tcp 11434 path=/api/generate & \
	else \
		echo "$(YELLOW)Warning: No mDNS service found (dns-sd or avahi-publish-service)$(NC)"; \
		echo "$(YELLOW)Service discovery will not be available$(NC)"; \
	fi
	@echo "$(GREEN)Ollama backend fully configured and ready$(NC)"

ollama-advertise:
	@echo "$(BLUE)Advertising Ollama service via mDNS...$(NC)"
	@echo "$(YELLOW)Publishing mDNS service for automatic discovery$(NC)"
	@if command -v dns-sd >/dev/null 2>&1; then \
		echo "$(YELLOW)Using dns-sd (macOS)$(NC)"; \
		dns-sd -R "Ollama AI" _ollama._tcp . 11434 path=/api/generate & \
	elif command -v avahi-publish-service >/dev/null 2>&1; then \
		echo "$(YELLOW)Using avahi-publish-service (Linux)$(NC)"; \
		avahi-publish-service "Ollama AI" _ollama._tcp 11434 path=/api/generate & \
	else \
		echo "$(YELLOW)Warning: No mDNS service found (dns-sd or avahi-publish-service)$(NC)"; \
		echo "$(YELLOW)Service discovery will not be available$(NC)"; \
		exit 1; \
	fi
	@echo "$(GREEN)Ollama service advertised via mDNS$(NC)"

ollama-setup: ollama-start
	@echo "$(GREEN)Ollama backend fully configured and ready$(NC)"
	@echo "$(CYAN)ESP32-CAM will auto-discover the Ollama service$(NC)"

ollama-stop:
	@echo "$(BLUE)Stopping Ollama services...$(NC)"
	@pkill -f "ollama serve" || true
	@pkill -f "dns-sd.*Ollama" || true
	@pkill -f "avahi-publish-service.*Ollama" || true
	@echo "$(GREEN)Ollama services stopped$(NC)"

# === Utility Targets ===

info:
	@echo "$(CYAN)ESP32 AI-Powered Robot Car$(NC)"
	@echo ""
	@echo "$(GREEN)Hardware Configuration:$(NC)"
	@echo "  Main Controller: Heltec WiFi LoRa 32 V1 (ESP32-S3)"
	@echo "  Camera Module:   ESP32-CAM (AI Thinker) with OV2640"
	@echo "  Serial Ports:    Main=$(MAIN_PORT), Camera=$(CAM_PORT)"
	@echo ""
	@echo "$(GREEN)Software Framework:$(NC)"
	@echo "  ESP-IDF Path:    $(IDF_PATH)"
	@echo "  ESP-IDF Status:  $(shell [ -d '$(IDF_PATH)' ] && echo 'Available' || echo 'Not found')"
	@echo ""
	@echo "$(GREEN)Project Structure:$(NC)"
	@echo "  idf-robocar/     - Main controller with motor/LED/servo control"
	@echo "  esp32-cam-idf/   - AI vision system with Claude API integration"
	@echo ""
	@echo "$(GREEN)Features:$(NC)"
	@echo "  - AI-powered autonomous navigation using Claude API"
	@echo "  - Dual ESP32 architecture with serial inter-board communication"  
	@echo "  - Real-time image processing and scene analysis"
	@echo "  - Motor control with RGB LEDs and servo-mounted camera"
	@echo "  - OLED display for system status and diagnostics"
	@echo "  - Python simulation with Swift 3D visualization"

# === Simulation Targets ===

check-sim:
	@if [ ! -d "simulation" ]; then \
		echo "$(RED)Error: Simulation directory not found$(NC)"; \
		echo "$(YELLOW)The simulation environment needs to be set up first$(NC)"; \
		exit 1; \
	fi
	@if [ ! -f "simulation/Makefile" ]; then \
		echo "$(RED)Error: Simulation Makefile not found$(NC)"; \
		echo "$(YELLOW)Run the setup process to create simulation environment$(NC)"; \
		exit 1; \
	fi

sim-start: check-sim
	@echo "$(BLUE)Starting ESP32 Robot Car Simulation...$(NC)"
	@echo "$(YELLOW)Launching with Swift 3D visualization$(NC)"
	@cd simulation && $(MAKE) run

sim-headless: check-sim
	@echo "$(BLUE)Starting headless simulation...$(NC)"
	@echo "$(YELLOW)Running without visualization for performance$(NC)"
	@cd simulation && $(MAKE) run-headless

sim-demo: check-sim
	@echo "$(BLUE)Starting demo simulation...$(NC)"
	@echo "$(YELLOW)Running autonomous movement patterns$(NC)"
	@cd simulation && $(MAKE) run-demo

sim-connect: check-sim
	@if [ -z "$(SERIAL_PORT)" ]; then \
		echo "$(RED)Error: SERIAL_PORT not specified$(NC)"; \
		echo "$(YELLOW)Usage: make sim-connect SERIAL_PORT=/dev/ttyUSB0$(NC)"; \
		exit 1; \
	fi
	@echo "$(BLUE)Connecting simulation to ESP32 hardware...$(NC)"
	@echo "$(YELLOW)Serial port: $(SERIAL_PORT)$(NC)"
	@cd simulation && $(MAKE) run-serial SERIAL_PORT=$(SERIAL_PORT)

sim-test: check-sim
	@echo "$(BLUE)Running simulation test suite...$(NC)"
	@cd simulation && $(MAKE) test

sim-install:
	@echo "$(BLUE)Installing simulation dependencies...$(NC)"
	@if [ ! -d "simulation" ]; then \
		echo "$(YELLOW)Simulation directory not found - please ensure simulation is set up$(NC)"; \
		exit 1; \
	fi
	@cd simulation && $(MAKE) install

sim-status:
	@echo "$(BLUE)Checking simulation status...$(NC)"
	@if [ ! -d "simulation" ]; then \
		echo "$(RED)Simulation not available$(NC)"; \
		exit 1; \
	fi
	@cd simulation && $(MAKE) status

sim-info:
	@echo "$(BLUE)Simulation system information...$(NC)"
	@if [ ! -d "simulation" ]; then \
		echo "$(RED)Simulation not available$(NC)"; \
		echo "$(YELLOW)Please set up the simulation environment first$(NC)"; \
	else \
		cd simulation && $(MAKE) info; \
	fi

.PHONY: help check-idf build-main build-cam build-all flash-main flash-cam
.PHONY: monitor-main monitor-cam clean-main clean-cam clean-all credentials
.PHONY: develop-main develop-cam ollama-start ollama-advertise ollama-setup ollama-stop info
.PHONY: check-sim sim-start sim-headless sim-demo sim-connect sim-test sim-install sim-status sim-info