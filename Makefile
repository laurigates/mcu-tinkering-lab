# MCU Tinkering Lab - Unified Build System
# Root-level Makefile to coordinate all embedded projects in the monorepo

# Configuration
IDF_PATH ?= $(HOME)/repos/esp-idf
PORT ?= /dev/cu.usbserial-0001
DEFAULT_PORT := $(PORT)

# Colors for output
RED = \033[0;31m
GREEN = \033[0;32m
YELLOW = \033[1;33m
BLUE = \033[0;34m
CYAN = \033[0;36m
MAGENTA = \033[0;35m
NC = \033[0m # No Color

# Project directories
ROBOCAR_DIR = robocar
ESP32_PACKAGES_DIR = packages/esp32-projects
ARDUINO_PACKAGES_DIR = packages/arduino-projects
STM32_PACKAGES_DIR = packages/stm32-projects

# ESP32 package projects
ESP32_WEBSERVER_DIR = $(ESP32_PACKAGES_DIR)/esp32-cam-webserver
ESP32_AUDIO_DIR = $(ESP32_PACKAGES_DIR)/esp32-cam-i2s-audio

# Default target shows comprehensive help
.DEFAULT_GOAL := help

help:
	@echo "$(CYAN)MCU Tinkering Lab - Unified Build System$(NC)"
	@echo "Coordinate all embedded projects in the monorepo"
	@echo ""
	@echo "$(GREEN)🤖 AI-Powered Robot Car (Primary Project):$(NC)"
	@echo "  make robocar-help         - Show robocar-specific commands"
	@echo "  make robocar-build-all    - Build both main controller and camera"
	@echo "  make robocar-flash-all    - Flash both controllers (GPIO0->GND for camera)"
	@echo "  make robocar-develop-main - Build, flash, and monitor main controller"
	@echo "  make robocar-develop-cam  - Build, flash, and monitor camera module"
	@echo "  make robocar-credentials  - Setup camera credentials"
	@echo "  make robocar-info         - Show robocar system configuration"
	@echo ""
	@echo "$(GREEN)📦 ESP32 Package Projects:$(NC)"
	@echo "  make esp32-list           - List all ESP32 package projects"
	@echo "  make esp32-webserver-build - Build ESP32-CAM webserver project"
	@echo "  make esp32-webserver-flash - Flash ESP32-CAM webserver (GPIO0->GND)"
	@echo "  make esp32-audio-build    - Build ESP32-CAM I2S audio project"
	@echo "  make esp32-audio-flash    - Flash ESP32-CAM I2S audio (GPIO0->GND)"
	@echo ""
	@echo "$(GREEN)🔧 Utility Commands:$(NC)"
	@echo "  make clean-all            - Clean all project builds"
	@echo "  make list-projects        - List all projects in monorepo"
	@echo "  make check-environment    - Verify development environment"
	@echo "  make info                 - Show comprehensive system information"
	@echo ""
	@echo "$(GREEN)🏗️  Build All Projects:$(NC)"
	@echo "  make build-all            - Build all projects in the monorepo"
	@echo "  make build-esp32          - Build all ESP32 projects"
	@echo ""
	@echo "$(YELLOW)Configuration:$(NC)"
	@echo "  Default Serial Port: $(DEFAULT_PORT)"
	@echo "  ESP-IDF Path:        $(IDF_PATH)"
	@echo "  Robocar Directory:   $(ROBOCAR_DIR)"
	@echo ""
	@echo "$(YELLOW)Environment Variables:$(NC)"
	@echo "  PORT=<device>         - Override default serial port"
	@echo "  IDF_PATH=<path>       - Override ESP-IDF installation path"

# === Environment Checks ===

check-environment:
	@echo "$(BLUE)Checking development environment...$(NC)"
	@echo ""
	@echo "$(GREEN)ESP-IDF Framework:$(NC)"
	@if [ -d "$(IDF_PATH)" ]; then \
		echo "  ✓ ESP-IDF found at $(IDF_PATH)"; \
	else \
		echo "  ✗ ESP-IDF not found at $(IDF_PATH)"; \
		echo "    $(YELLOW)Install ESP-IDF or set IDF_PATH variable$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Serial Port:$(NC)"
	@if [ -e "$(DEFAULT_PORT)" ]; then \
		echo "  ✓ Serial port available: $(DEFAULT_PORT)"; \
	else \
		echo "  ✗ Serial port not found: $(DEFAULT_PORT)"; \
		echo "    $(YELLOW)Connect device or set PORT variable$(NC)"; \
	fi
	@echo ""
	@echo "$(GREEN)Project Structure:$(NC)"
	@if [ -d "$(ROBOCAR_DIR)" ]; then \
		echo "  ✓ Robocar project found"; \
	else \
		echo "  ✗ Robocar project missing"; \
	fi
	@if [ -d "$(ESP32_PACKAGES_DIR)" ]; then \
		echo "  ✓ ESP32 packages directory found"; \
	else \
		echo "  ✗ ESP32 packages directory missing"; \
	fi

check-idf:
	@if [ ! -d "$(IDF_PATH)" ]; then \
		echo "$(RED)Error: ESP-IDF not found at $(IDF_PATH)$(NC)"; \
		echo "$(YELLOW)Please install ESP-IDF or set IDF_PATH variable$(NC)"; \
		exit 1; \
	fi

# === AI-Powered Robot Car Commands ===

robocar-help:
	@echo "$(CYAN)AI-Powered Robot Car - Build Commands$(NC)"
	@echo ""
	@cd $(ROBOCAR_DIR) && $(MAKE) help

robocar-build-all: check-idf
	@echo "$(BLUE)Building AI-powered robot car (all controllers)...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) build-all

robocar-build-main: check-idf
	@echo "$(BLUE)Building robot car main controller...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) build-main

robocar-build-cam: check-idf
	@echo "$(BLUE)Building robot car camera module...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) build-cam

robocar-flash-all: check-idf
	@echo "$(BLUE)Flashing AI-powered robot car (both controllers)...$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for ESP32-CAM programming$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) flash-main PORT=$(PORT)
	@echo "$(YELLOW)Now connect GPIO0 to GND on ESP32-CAM and press Enter...$(NC)"
	@read -p ""
	@cd $(ROBOCAR_DIR) && $(MAKE) flash-cam PORT=$(PORT)

robocar-flash-main: check-idf
	@echo "$(BLUE)Flashing robot car main controller...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) flash-main PORT=$(PORT)

robocar-flash-cam: check-idf
	@echo "$(BLUE)Flashing robot car camera module...$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for programming$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) flash-cam PORT=$(PORT)

robocar-develop-main: check-idf
	@echo "$(BLUE)Development workflow: main controller$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) develop-main PORT=$(PORT)

robocar-develop-cam: check-idf
	@echo "$(BLUE)Development workflow: camera module$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for programming$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) develop-cam PORT=$(PORT)

robocar-monitor-main: check-idf
	@echo "$(BLUE)Monitoring robot car main controller...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) monitor-main PORT=$(PORT)

robocar-monitor-cam: check-idf
	@echo "$(BLUE)Monitoring robot car camera module...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) monitor-cam PORT=$(PORT)

robocar-credentials:
	@echo "$(BLUE)Setting up robot car credentials...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) credentials

robocar-clean:
	@echo "$(BLUE)Cleaning robot car builds...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) clean-all

robocar-info:
	@echo "$(BLUE)Robot car system information...$(NC)"
	@cd $(ROBOCAR_DIR) && $(MAKE) info

# === ESP32 Package Projects ===

esp32-list:
	@echo "$(CYAN)ESP32 Package Projects$(NC)"
	@echo ""
	@echo "$(GREEN)Available Projects:$(NC)"
	@if [ -d "$(ESP32_WEBSERVER_DIR)" ]; then \
		echo "  📹 esp32-cam-webserver  - Live video streaming web server"; \
	fi
	@if [ -d "$(ESP32_AUDIO_DIR)" ]; then \
		echo "  🔊 esp32-cam-i2s-audio  - Camera + I2S audio processing"; \
	fi
	@echo ""
	@echo "$(GREEN)Build Commands:$(NC)"
	@echo "  make esp32-webserver-build - Build webserver project"
	@echo "  make esp32-audio-build     - Build audio project"
	@echo ""
	@echo "$(GREEN)Flash Commands:$(NC)"
	@echo "  make esp32-webserver-flash - Flash webserver (GPIO0->GND)"
	@echo "  make esp32-audio-flash     - Flash audio (GPIO0->GND)"

esp32-webserver-build: check-idf
	@echo "$(BLUE)Building ESP32-CAM webserver project...$(NC)"
	@if [ -d "$(ESP32_WEBSERVER_DIR)" ]; then \
		cd $(ESP32_WEBSERVER_DIR) && idf.py build; \
	else \
		echo "$(RED)Error: ESP32 webserver project not found$(NC)"; \
		exit 1; \
	fi

esp32-webserver-flash: check-idf
	@echo "$(BLUE)Flashing ESP32-CAM webserver project...$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for ESP32-CAM programming$(NC)"
	@if [ -d "$(ESP32_WEBSERVER_DIR)" ]; then \
		cd $(ESP32_WEBSERVER_DIR) && idf.py flash -p $(PORT); \
	else \
		echo "$(RED)Error: ESP32 webserver project not found$(NC)"; \
		exit 1; \
	fi

esp32-webserver-monitor: check-idf
	@echo "$(BLUE)Monitoring ESP32-CAM webserver...$(NC)"
	@if [ -d "$(ESP32_WEBSERVER_DIR)" ]; then \
		cd $(ESP32_WEBSERVER_DIR) && idf.py monitor -p $(PORT); \
	else \
		echo "$(RED)Error: ESP32 webserver project not found$(NC)"; \
		exit 1; \
	fi

esp32-audio-build: check-idf
	@echo "$(BLUE)Building ESP32-CAM I2S audio project...$(NC)"
	@if [ -d "$(ESP32_AUDIO_DIR)" ]; then \
		cd $(ESP32_AUDIO_DIR) && idf.py build; \
	else \
		echo "$(RED)Error: ESP32 audio project not found$(NC)"; \
		exit 1; \
	fi

esp32-audio-flash: check-idf
	@echo "$(BLUE)Flashing ESP32-CAM I2S audio project...$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for ESP32-CAM programming$(NC)"
	@if [ -d "$(ESP32_AUDIO_DIR)" ]; then \
		cd $(ESP32_AUDIO_DIR) && idf.py flash -p $(PORT); \
	else \
		echo "$(RED)Error: ESP32 audio project not found$(NC)"; \
		exit 1; \
	fi

esp32-audio-monitor: check-idf
	@echo "$(BLUE)Monitoring ESP32-CAM I2S audio...$(NC)"
	@if [ -d "$(ESP32_AUDIO_DIR)" ]; then \
		cd $(ESP32_AUDIO_DIR) && idf.py monitor -p $(PORT); \
	else \
		echo "$(RED)Error: ESP32 audio project not found$(NC)"; \
		exit 1; \
	fi

# === Build All Projects ===

build-all: build-esp32
	@echo "$(GREEN)✓ All projects built successfully$(NC)"

build-esp32: robocar-build-all esp32-webserver-build esp32-audio-build
	@echo "$(GREEN)✓ All ESP32 projects built successfully$(NC)"

# === Clean Operations ===

clean-all: robocar-clean esp32-clean
	@echo "$(GREEN)✓ All project builds cleaned$(NC)"

esp32-clean:
	@echo "$(BLUE)Cleaning ESP32 package projects...$(NC)"
	@if [ -d "$(ESP32_WEBSERVER_DIR)" ]; then \
		echo "  Cleaning ESP32 webserver..."; \
		cd $(ESP32_WEBSERVER_DIR) && idf.py fullclean || true; \
	fi
	@if [ -d "$(ESP32_AUDIO_DIR)" ]; then \
		echo "  Cleaning ESP32 audio..."; \
		cd $(ESP32_AUDIO_DIR) && idf.py fullclean || true; \
	fi

# === Information and Discovery ===

list-projects:
	@echo "$(CYAN)MCU Tinkering Lab - Project Inventory$(NC)"
	@echo ""
	@echo "$(GREEN)🤖 AI-Powered Robot Car (Primary):$(NC)"
	@if [ -d "$(ROBOCAR_DIR)" ]; then \
		echo "  ✓ Dual ESP32 autonomous robot with AI vision"; \
		echo "    └── Main Controller: Heltec WiFi LoRa 32 V1"; \
		echo "    └── Vision System: ESP32-CAM with Claude/Ollama AI"; \
	else \
		echo "  ✗ Not found"; \
	fi
	@echo ""
	@echo "$(GREEN)📦 ESP32 Package Projects:$(NC)"
	@if [ -d "$(ESP32_WEBSERVER_DIR)" ]; then \
		echo "  ✓ esp32-cam-webserver - Live video streaming web server"; \
	fi
	@if [ -d "$(ESP32_AUDIO_DIR)" ]; then \
		echo "  ✓ esp32-cam-i2s-audio - Camera + I2S audio processing (placeholder)"; \
	fi
	@echo ""
	@echo "$(GREEN)🔧 Platform Directories:$(NC)"
	@if [ -d "$(ARDUINO_PACKAGES_DIR)" ]; then \
		echo "  ✓ Arduino projects directory"; \
	else \
		echo "  ○ Arduino projects (empty)"; \
	fi
	@if [ -d "$(STM32_PACKAGES_DIR)" ]; then \
		echo "  ✓ STM32 projects directory"; \
	else \
		echo "  ○ STM32 projects (empty)"; \
	fi
	@if [ -d "packages/shared-libs" ]; then \
		echo "  ✓ Shared libraries directory"; \
	else \
		echo "  ○ Shared libraries (empty)"; \
	fi

info: check-environment
	@echo ""
	@echo "$(CYAN)MCU Tinkering Lab - System Overview$(NC)"
	@echo ""
	@echo "$(GREEN)Repository Structure:$(NC)"
	@echo "  📁 Root Directory:     $(shell pwd)"
	@echo "  📁 Robocar Project:    $(ROBOCAR_DIR)/"
	@echo "  📁 ESP32 Packages:     $(ESP32_PACKAGES_DIR)/"
	@echo "  📁 Arduino Packages:   $(ARDUINO_PACKAGES_DIR)/"
	@echo "  📁 STM32 Packages:     $(STM32_PACKAGES_DIR)/"
	@echo ""
	@echo "$(GREEN)Primary Features:$(NC)"
	@echo "  🤖 AI-powered autonomous robot car with dual ESP32 architecture"
	@echo "  📹 Pluggable AI backends (Claude API, Ollama self-hosted)"
	@echo "  🔌 Structured I2C communication protocol between controllers"
	@echo "  🎮 Real-time computer vision and scene analysis"
	@echo "  🔧 Comprehensive build automation and development workflows"
	@echo ""
	@echo "$(GREEN)Quick Start:$(NC)"
	@echo "  make robocar-info         # Detailed robocar system information"
	@echo "  make list-projects        # Show all available projects"
	@echo "  make build-all           # Build all projects"
	@echo "  make robocar-develop-main # Start developing the main controller"

# === Development Shortcuts ===

# Quick access to most commonly used robocar commands
dev-main: robocar-develop-main
dev-cam: robocar-develop-cam
build: build-all
clean: clean-all

# === Advanced Operations ===

# Git operations for the entire monorepo
git-status:
	@echo "$(BLUE)Git status for entire monorepo...$(NC)"
	@git status

git-check-credentials:
	@echo "$(BLUE)Checking for credentials in git staging area...$(NC)"
	@if git diff --cached --name-only | grep -E "(credentials\\.h|\\.(key|secret|token))" > /dev/null 2>&1; then \
		echo "$(RED)⚠️  Warning: Potential credentials files in staging area:$(NC)"; \
		git diff --cached --name-only | grep -E "(credentials\\.h|\\.(key|secret|token))"; \
		echo "$(YELLOW)These files should be in .gitignore$(NC)"; \
		exit 1; \
	else \
		echo "$(GREEN)✓ No credential files in staging area$(NC)"; \
	fi

# Documentation generation
docs-generate:
	@echo "$(BLUE)Generating project documentation...$(NC)"
	@echo "$(YELLOW)Documentation generation not yet implemented$(NC)"
	@echo "Future: Auto-generate docs from code comments and README files"

# === Help for specific subsystems ===

help-robocar: robocar-help

help-esp32: esp32-list

help-build:
	@echo "$(CYAN)Build System Help$(NC)"
	@echo ""
	@echo "$(GREEN)Build Commands:$(NC)"
	@echo "  make build-all            - Build all projects in monorepo"
	@echo "  make build-esp32          - Build all ESP32 projects"
	@echo "  make robocar-build-all    - Build robocar main + camera"
	@echo "  make esp32-webserver-build - Build ESP32 webserver"
	@echo "  make esp32-audio-build    - Build ESP32 audio project"
	@echo ""
	@echo "$(GREEN)Flash Commands:$(NC)"
	@echo "  make robocar-flash-all    - Flash both robocar controllers"
	@echo "  make esp32-webserver-flash - Flash ESP32 webserver (GPIO0->GND)"
	@echo "  make esp32-audio-flash    - Flash ESP32 audio (GPIO0->GND)"
	@echo ""
	@echo "$(GREEN)Development Workflows:$(NC)"
	@echo "  make dev-main             - Shortcut for robocar main development"
	@echo "  make dev-cam              - Shortcut for robocar camera development"
	@echo ""
	@echo "$(YELLOW)ESP32-CAM Programming Note:$(NC)"
	@echo "  All ESP32-CAM modules require GPIO0 connected to GND during"
	@echo "  programming, then disconnected after successful flash."

# Declare phony targets
.PHONY: help check-environment check-idf list-projects info clean-all build-all
.PHONY: robocar-help robocar-build-all robocar-build-main robocar-build-cam
.PHONY: robocar-flash-all robocar-flash-main robocar-flash-cam
.PHONY: robocar-develop-main robocar-develop-cam robocar-monitor-main robocar-monitor-cam
.PHONY: robocar-credentials robocar-clean robocar-info
.PHONY: esp32-list esp32-webserver-build esp32-webserver-flash esp32-webserver-monitor
.PHONY: esp32-audio-build esp32-audio-flash esp32-audio-monitor esp32-clean
.PHONY: build-esp32 dev-main dev-cam build clean
.PHONY: git-status git-check-credentials docs-generate
.PHONY: help-robocar help-esp32 help-build