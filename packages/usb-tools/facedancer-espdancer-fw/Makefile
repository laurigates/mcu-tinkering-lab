# facedancer-espdancer-fw — Project Makefile
# Prefer justfile: `just --list`

PROJECT_NAME := facedancer-espdancer-fw
PORT ?= /dev/cu.usbserial-0001
BAUD ?= 460800
CONTAINER_CMD ?= docker
PROJECT_DIR := packages/usb-tools/facedancer-espdancer-fw
COMPOSE_FILE := ../../../docker-compose.yml

.PHONY: build flash monitor flash-monitor clean menuconfig info help

build: ## Build firmware (containerized)
	$(CONTAINER_CMD) compose -f $(COMPOSE_FILE) run --rm \
		-w /workspace/$(PROJECT_DIR) \
		esp-idf \
		bash -c "idf.py set-target esp32s3 && idf.py build"

flash: ## Flash firmware to device (native)
	esptool -p $(PORT) -b $(BAUD) \
		--before default-reset --after hard-reset \
		write-flash @build/flash_args

monitor: ## Start serial monitor (native)
	python3 -m esp_idf_monitor -p $(PORT) build/$(PROJECT_NAME).elf

flash-monitor: flash monitor ## Flash and monitor

clean: ## Clean build artifacts (containerized)
	$(CONTAINER_CMD) compose -f $(COMPOSE_FILE) run --rm \
		-w /workspace/$(PROJECT_DIR) esp-idf idf.py fullclean

menuconfig: ## Open configuration menu (containerized)
	$(CONTAINER_CMD) compose -f $(COMPOSE_FILE) run --rm \
		-w /workspace/$(PROJECT_DIR) esp-idf idf.py menuconfig

info: ## Show project info
	@echo "Project: $(PROJECT_NAME)  Target: ESP32-S3  Port: $(PORT)"

help: ## Show this help
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  %-15s %s\n", $$1, $$2}'