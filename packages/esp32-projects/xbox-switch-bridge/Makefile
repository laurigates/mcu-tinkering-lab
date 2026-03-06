# Xbox-Switch Bridge - Project Makefile
# Wraps idf.py commands for convenience

PROJECT_NAME := xbox-switch-bridge
PORT ?= /dev/ttyUSB0
BAUD ?= 460800

# Colors
BLUE := \033[1;34m
GREEN := \033[1;32m
YELLOW := \033[1;33m
NC := \033[0m

# ESP-IDF environment detection
IDF_PATH ?= $(HOME)/esp/esp-idf
IDF_ENV_CMD := . $(IDF_PATH)/export.sh 2>/dev/null

.PHONY: build flash monitor flash-monitor clean fullclean menuconfig info help

build: ## Build firmware
	@echo "$(BLUE)Building $(PROJECT_NAME)...$(NC)"
	@$(IDF_ENV_CMD) && idf.py set-target esp32s3 && idf.py build
	@echo "$(GREEN)Build complete!$(NC)"

flash: ## Flash firmware to device
	@echo "$(BLUE)Flashing $(PROJECT_NAME) to $(PORT)...$(NC)"
	@$(IDF_ENV_CMD) && idf.py -p $(PORT) -b $(BAUD) flash

monitor: ## Start serial monitor
	@echo "$(BLUE)Starting monitor on $(PORT)...$(NC)"
	@$(IDF_ENV_CMD) && idf.py -p $(PORT) monitor

flash-monitor: ## Flash and start monitor
	@$(IDF_ENV_CMD) && idf.py -p $(PORT) -b $(BAUD) flash monitor

clean: ## Clean build artifacts
	@$(IDF_ENV_CMD) && idf.py fullclean
	@echo "$(GREEN)Clean complete$(NC)"

fullclean: ## Full clean including sdkconfig
	@$(IDF_ENV_CMD) && idf.py fullclean
	@rm -f sdkconfig sdkconfig.old
	@echo "$(GREEN)Full clean complete$(NC)"

menuconfig: ## Open configuration menu
	@$(IDF_ENV_CMD) && idf.py menuconfig

info: ## Show project info
	@echo "$(BLUE)Project: $(PROJECT_NAME)$(NC)"
	@echo "  Target: ESP32-S3"
	@echo "  Port: $(PORT)"
	@echo "  Baud: $(BAUD)"
	@echo ""
	@echo "  USB VID: 0x057E (Nintendo)"
	@echo "  USB PID: 0x2009 (Pro Controller)"

help: ## Show this help
	@echo "$(BLUE)$(PROJECT_NAME) - Available targets:$(NC)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  $(GREEN)%-15s$(NC) %s\n", $$1, $$2}'
