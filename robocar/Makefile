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
	@cd idf-robocar && $(MAKE) monitor PORT=$(MAIN_PORT)

monitor-cam: check-idf
	@echo "$(BLUE)Monitoring camera module...$(NC)"
	@cd esp32-cam-idf && $(MAKE) monitor PORT=$(CAM_PORT)

clean-main: check-idf
	@cd idf-robocar && $(MAKE) clean

clean-cam: check-idf
	@cd esp32-cam-idf && $(MAKE) clean

clean-all: clean-main clean-cam

credentials:
	@echo "$(BLUE)Setting up camera credentials...$(NC)"
	@cd esp32-cam-idf && $(MAKE) credentials

develop-main: flash-main monitor-main

develop-cam: flash-cam monitor-cam

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

.PHONY: help check-idf build-main build-cam build-all flash-main flash-cam
.PHONY: monitor-main monitor-cam clean-main clean-cam clean-all credentials
.PHONY: develop-main develop-cam info