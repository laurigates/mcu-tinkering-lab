# Xbox-Switch Bridge - Project Makefile
# Prefer justfile: `just --list` or `just xbox::build`

PROJECT_NAME := xbox-switch-bridge
PORT ?= /dev/cu.usbserial-0001
BAUD ?= 460800
CONTAINER_CMD ?= docker
PROJECT_DIR := packages/input-gaming/xbox-switch-bridge
COMPOSE_FILE := ../../../docker-compose.yml

# Colors
BLUE := \033[1;34m
GREEN := \033[1;32m
YELLOW := \033[1;33m
NC := \033[0m

.PHONY: build flash monitor flash-monitor clean menuconfig info help

build: ## Build firmware (containerized)
	@echo "$(BLUE)Building $(PROJECT_NAME) (containerized)...$(NC)"
	@$(CONTAINER_CMD) compose -f $(COMPOSE_FILE) run --rm \
		-w /workspace/$(PROJECT_DIR) \
		esp-idf \
		bash -c "idf.py set-target esp32s3 && idf.py build"
	@echo "$(GREEN)Build complete!$(NC)"

flash: ## Flash firmware to device (native)
	esptool -p $(PORT) -b $(BAUD) \
		--before default-reset --after hard-reset \
		write-flash @build/flash_args

monitor: ## Start serial monitor (native)
	python3 -m esp_idf_monitor -p $(PORT) build/$(PROJECT_NAME).elf

flash-monitor: flash monitor ## Flash and start monitor

clean: ## Clean build artifacts (containerized)
	@$(CONTAINER_CMD) compose -f $(COMPOSE_FILE) run --rm \
		-w /workspace/$(PROJECT_DIR) \
		esp-idf \
		idf.py fullclean
	@echo "$(GREEN)Clean complete$(NC)"

menuconfig: ## Open configuration menu (containerized)
	@$(CONTAINER_CMD) compose -f $(COMPOSE_FILE) run --rm \
		-w /workspace/$(PROJECT_DIR) \
		esp-idf \
		idf.py menuconfig

info: ## Show project info
	@echo "$(BLUE)Project: $(PROJECT_NAME)$(NC)"
	@echo "  Target:    ESP32-S3"
	@echo "  Port:      $(PORT)"
	@echo "  Baud:      $(BAUD)"
	@echo "  Container: $(CONTAINER_CMD)"
	@echo ""
	@echo "  USB VID: 0x057E (Nintendo)"
	@echo "  USB PID: 0x2009 (Pro Controller)"

help: ## Show this help
	@echo "$(BLUE)$(PROJECT_NAME) - Available targets:$(NC)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  $(GREEN)%-15s$(NC) %s\n", $$1, $$2}'
