# Audiobook Player Makefile
# ESPHome-based RFID audiobook player

.PHONY: help install config compile upload logs clean monitor wireless

# Configuration
DEVICE_NAME := audiobook-player
CONFIG_FILE := $(DEVICE_NAME).yaml
SECRETS_FILE := secrets.yaml
SECRETS_EXAMPLE := secrets.yaml.example

help:
	@echo "Audiobook Player - ESPHome Project"
	@echo ""
	@echo "Available targets:"
	@echo "  install      - Install ESPHome and dependencies"
	@echo "  config       - Copy secrets.yaml.example to secrets.yaml"
	@echo "  validate     - Validate configuration"
	@echo "  compile      - Compile firmware"
	@echo "  upload       - Compile and upload via USB"
	@echo "  wireless     - Upload via WiFi (OTA)"
	@echo "  logs         - View device logs"
	@echo "  monitor      - Monitor logs in real-time"
	@echo "  clean        - Clean build files"
	@echo "  dashboard    - Open ESPHome dashboard"

install:
	@echo "Installing ESPHome..."
	pip install --upgrade esphome

config:
	@if [ ! -f $(SECRETS_FILE) ]; then \
		echo "Creating secrets.yaml from template..."; \
		cp $(SECRETS_EXAMPLE) $(SECRETS_FILE); \
		echo "Please edit $(SECRETS_FILE) with your WiFi credentials"; \
	else \
		echo "$(SECRETS_FILE) already exists"; \
	fi

validate:
	@echo "Validating configuration..."
	esphome config $(CONFIG_FILE)

compile:
	@echo "Compiling firmware..."
	esphome compile $(CONFIG_FILE)

upload:
	@if [ ! -f $(SECRETS_FILE) ]; then \
		echo "Error: $(SECRETS_FILE) not found. Run 'make config' first."; \
		exit 1; \
	fi
	@echo "Compiling and uploading via USB..."
	esphome run $(CONFIG_FILE)

wireless:
	@echo "Uploading via WiFi (OTA)..."
	esphome run $(CONFIG_FILE) --device $(DEVICE_NAME).local

logs:
	@echo "Viewing logs..."
	esphome logs $(CONFIG_FILE)

monitor: logs

clean:
	@echo "Cleaning build files..."
	rm -rf .esphome/
	rm -f *.bin *.elf *.map

dashboard:
	@echo "Starting ESPHome dashboard..."
	@echo "Access at http://localhost:6052"
	esphome dashboard .

# Development workflow shortcuts
.PHONY: init first-flash dev

init: install config
	@echo ""
	@echo "Initialization complete!"
	@echo "Next steps:"
	@echo "  1. Edit $(SECRETS_FILE) with your WiFi credentials"
	@echo "  2. Run 'make upload' to flash the device via USB"
	@echo "  3. Run 'make logs' to view device logs"

first-flash: config upload
	@echo ""
	@echo "First flash complete!"
	@echo "Future updates can be done wirelessly with 'make wireless'"

dev: upload logs

# Quick reference
.PHONY: status pins

status:
	@echo "Device: $(DEVICE_NAME)"
	@echo "Config: $(CONFIG_FILE)"
	@echo "Secrets: $(SECRETS_FILE)"
	@if [ -f $(SECRETS_FILE) ]; then \
		echo "Status: Configured ✓"; \
	else \
		echo "Status: Not configured - run 'make config'"; \
	fi

pins:
	@echo "Pin Assignment Reference:"
	@echo ""
	@echo "RC522 RFID Reader (SPI):"
	@echo "  SDA (CS) → D8 (GPIO15)"
	@echo "  SCK      → D5 (GPIO14)"
	@echo "  MOSI     → D7 (GPIO13)"
	@echo "  MISO     → D6 (GPIO12)"
	@echo "  3.3V     → 3.3V"
	@echo "  GND      → GND"
	@echo ""
	@echo "Buttons:"
	@echo "  Green (Play)  → D1 (GPIO5) + GND"
	@echo "  Red (Pause)   → D2 (GPIO4) + GND"
	@echo ""
	@echo "Status LED: D4 (GPIO2) - Built-in"
