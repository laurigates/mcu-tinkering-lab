# ESP32 WireGuard Home Assistant Example Makefile
# ESPHome-based ESP32 with WireGuard VPN and Home Assistant integration

.PHONY: help install config compile upload logs clean monitor wireless

# Configuration
DEVICE_NAME := esp32-wireguard-ha
CONFIG_FILE := $(DEVICE_NAME).yaml
SECRETS_FILE := secrets.yaml
SECRETS_EXAMPLE := secrets.yaml.example

help:
	@echo "ESP32 WireGuard HA Example - ESPHome Project"
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
	@echo "  wg-keys      - Generate WireGuard key pair"

install:
	@echo "Installing ESPHome..."
	pip install --upgrade esphome

config:
	@if [ ! -f $(SECRETS_FILE) ]; then \
		echo "Creating secrets.yaml from template..."; \
		cp $(SECRETS_EXAMPLE) $(SECRETS_FILE); \
		echo ""; \
		echo "⚠️  IMPORTANT: Edit $(SECRETS_FILE) with your configuration:"; \
		echo "   1. WiFi credentials"; \
		echo "   2. WireGuard keys and server details"; \
		echo "   3. API encryption key (will be generated on first run)"; \
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

# WireGuard key generation
wg-keys:
	@echo "Generating WireGuard key pair..."
	@if command -v wg >/dev/null 2>&1; then \
		echo ""; \
		echo "Private key:"; \
		wg genkey | tee /tmp/wg_private.key | wg pubkey > /tmp/wg_public.key; \
		cat /tmp/wg_private.key; \
		echo ""; \
		echo "Public key (share with server):"; \
		cat /tmp/wg_public.key; \
		echo ""; \
		echo "⚠️  Save the private key in secrets.yaml"; \
		echo "⚠️  Share the public key with your WireGuard server admin"; \
		rm -f /tmp/wg_private.key /tmp/wg_public.key; \
	else \
		echo "Error: wireguard-tools not installed"; \
		echo "Install with: sudo apt-get install wireguard-tools"; \
		echo "Or manually generate keys at https://www.wireguardconfig.com/"; \
	fi

# Development workflow shortcuts
.PHONY: init first-flash dev

init: install config
	@echo ""
	@echo "Initialization complete!"
	@echo "Next steps:"
	@echo "  1. Run 'make wg-keys' to generate WireGuard keys"
	@echo "  2. Edit $(SECRETS_FILE) with all required values"
	@echo "  3. Configure your WireGuard server with this device's public key"
	@echo "  4. Run 'make upload' to flash the device via USB"
	@echo "  5. Run 'make logs' to monitor WireGuard connection status"

first-flash: config upload
	@echo ""
	@echo "First flash complete!"
	@echo "Future updates can be done wirelessly with 'make wireless'"

dev: upload logs

# Quick reference
.PHONY: status info

status:
	@echo "Device: $(DEVICE_NAME)"
	@echo "Config: $(CONFIG_FILE)"
	@echo "Secrets: $(SECRETS_FILE)"
	@if [ -f $(SECRETS_FILE) ]; then \
		echo "Status: Configured ✓"; \
	else \
		echo "Status: Not configured - run 'make config'"; \
	fi

info:
	@echo "ESP32 WireGuard HA Example"
	@echo ""
	@echo "This project demonstrates:"
	@echo "  • WireGuard VPN connection from ESP32"
	@echo "  • Home Assistant integration via native API"
	@echo "  • Remote device access over VPN"
	@echo "  • Secure encrypted communication"
	@echo ""
	@echo "Requirements:"
	@echo "  • ESP32 development board"
	@echo "  • WireGuard VPN server"
	@echo "  • Home Assistant instance"
	@echo "  • WiFi network"
	@echo ""
	@echo "For detailed setup instructions, see README.md"
