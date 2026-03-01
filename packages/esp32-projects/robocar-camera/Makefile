# ESP32-CAM AI Vision System Makefile
# Simplifies ESP-IDF build, flash, and monitor operations

# Configuration
IDF_PATH ?= $(HOME)/repos/esp-idf
PORT ?= /dev/cu.usbserial-0001
BAUD_RATE ?= 115200
PROJECT_NAME = esp32-cam-robocar

# Colors for output
RED = \033[0;31m
GREEN = \033[0;32m
YELLOW = \033[1;33m
BLUE = \033[0;34m
NC = \033[0m # No Color

# Detect shell and set appropriate ESP-IDF environment
SHELL_NAME := $(shell basename $$SHELL)

# Auto-detect ESP-IDF version and Python environment
IDF_VERSION := $(shell if [ -f "$(IDF_PATH)/version.txt" ]; then cat $(IDF_PATH)/version.txt | head -1; else echo "unknown"; fi)
PYTHON_VERSION := $(shell python3 --version 2>/dev/null | cut -d' ' -f2 | cut -d'.' -f1,2)
IDF_PYTHON_ENV := $(HOME)/.espressif/python_env/idf$(IDF_VERSION)_py$(PYTHON_VERSION)_env

ifeq ($(SHELL_NAME),fish)
    # For Fish shell, use the Python virtual environment directly
    IDF_ENV_CMD = export PATH="$(IDF_PYTHON_ENV)/bin:$$PATH"
else
    # For bash/zsh, use the standard ESP-IDF export script
    IDF_ENV_CMD = source $(IDF_PATH)/export.sh
endif

# Check if ESP-IDF is available
check-idf:
	@if [ ! -d "$(IDF_PATH)" ]; then \
		echo "$(RED)Error: ESP-IDF not found at $(IDF_PATH)$(NC)"; \
		echo "$(YELLOW)Please install ESP-IDF or set IDF_PATH variable$(NC)"; \
		exit 1; \
	fi
	@echo "$(GREEN)Using ESP-IDF at: $(IDF_PATH)$(NC)"
	@echo "$(BLUE)Shell: $(SHELL_NAME), Environment: $(IDF_ENV_CMD)$(NC)"

# Setup credentials file if it doesn't exist
credentials:
	@if [ ! -f main/credentials.h ]; then \
		echo "$(YELLOW)Creating credentials.h from template...$(NC)"; \
		cp main/credentials.h.example main/credentials.h; \
		echo "$(RED)Please edit main/credentials.h with your WiFi and API credentials$(NC)"; \
	else \
		echo "$(GREEN)Credentials file exists$(NC)"; \
	fi

# Build the project
build: check-idf credentials
	@echo "$(BLUE)Building ESP32-CAM firmware...$(NC)"
	@if [ ! -d "$(IDF_PYTHON_ENV)" ]; then \
		echo "$(YELLOW)Python environment not found, running ESP-IDF install...$(NC)"; \
		cd $(IDF_PATH) && ./install.sh; \
	fi
	@$(IDF_ENV_CMD) && idf.py build

# Clean build files
clean: check-idf
	@echo "$(BLUE)Cleaning build files...$(NC)"
	@$(IDF_ENV_CMD) && idf.py clean

# Full clean (removes build directory)
fullclean: check-idf
	@echo "$(BLUE)Full clean - removing build directory...$(NC)"
	@$(IDF_ENV_CMD) && idf.py fullclean

# Flash firmware to ESP32-CAM
flash: check-idf
	@echo "$(BLUE)Flashing firmware to ESP32-CAM...$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for programming mode$(NC)"
	@$(IDF_ENV_CMD) && idf.py -p $(PORT) flash

# Monitor serial output
monitor: check-idf
	@echo "$(BLUE)Starting serial monitor on $(PORT)...$(NC)"
	@echo "$(YELLOW)Press Ctrl+] to exit monitor$(NC)"
	@$(IDF_ENV_CMD) && idf.py -p $(PORT) monitor

# Flash and immediately start monitoring
flash-monitor: check-idf
	@echo "$(BLUE)Flashing and monitoring ESP32-CAM...$(NC)"
	@echo "$(YELLOW)Make sure GPIO0 is connected to GND for programming mode$(NC)"
	@$(IDF_ENV_CMD) && idf.py -p $(PORT) flash monitor

# Show project configuration
menuconfig: check-idf
	@echo "$(BLUE)Opening configuration menu...$(NC)"
	@$(IDF_ENV_CMD) && idf.py menuconfig

# Show project information
info: check-idf
	@echo "$(GREEN)ESP32-CAM AI Vision System$(NC)"
	@echo "Project: $(PROJECT_NAME)"
	@echo "ESP-IDF Path: $(IDF_PATH)"
	@echo "Serial Port: $(PORT)"
	@echo "Baud Rate: $(BAUD_RATE)"
	@echo ""
	@echo "$(BLUE)Available commands:$(NC)"
	@echo "  make build        - Build the firmware"
	@echo "  make flash        - Flash firmware to ESP32-CAM"
	@echo "  make monitor      - Monitor serial output"
	@echo "  make flash-monitor- Flash and monitor"
	@echo "  make clean        - Clean build files"
	@echo "  make fullclean    - Full clean (removes build dir)"
	@echo "  make menuconfig   - Open configuration menu"
	@echo "  make credentials  - Create credentials file"
	@echo "  make info         - Show this information"
	@echo ""
	@echo "$(YELLOW)Programming ESP32-CAM:$(NC)"
	@echo "1. Connect GPIO0 to GND"
	@echo "2. Power on or reset the board"
	@echo "3. Run 'make flash' or 'make flash-monitor'"
	@echo "4. Disconnect GPIO0 from GND after flashing"
	@echo "5. Reset the board to run the new firmware"
	@echo ""
	@echo "$(YELLOW)Fish Shell Users:$(NC)"
	@echo "Makefile automatically detects Fish shell and uses Python virtual environment directly"
	@echo "No need to manually activate ESP-IDF environment"

# Default target
.DEFAULT_GOAL := info

# Declare phony targets
.PHONY: check-idf credentials build clean fullclean flash monitor flash-monitor menuconfig info
