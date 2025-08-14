# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a sophisticated dual-ESP32 autonomous robot car with AI-powered vision and comprehensive MQTT logging capabilities. The system features a pluggable AI backend architecture supporting both cloud-based and self-hosted AI services, with advanced hardware control capabilities and real-time telemetry.

## Build and Development Commands

**Framework:** ESP-IDF v5.4+ (primary implementation). ESP-IDF is installed at `~/repos/esp-idf`. The Makefiles automatically handle environment setup for different shells:

Run `make` to see the available commands.

## Key Development Features

### Complete Development Stack
- **One-Command Setup**: `make dev-stack-start` launches complete testing environment
- **AI Backend**: Ollama server with automatic mDNS service discovery
- **MQTT Broker**: Mosquitto broker with service advertising for ESP32 auto-discovery
- **Service Discovery**: Both services advertised via mDNS for seamless ESP32 integration

### MQTT Logging Integration
- **Real-time Telemetry**: Comprehensive logging from ESP32-CAM to MQTT broker
- **JSON Format**: Structured log messages with timestamps, levels, and system metrics
- **Offline Buffering**: Queued messages when broker unavailable
- **Service Discovery**: ESP32 auto-discovers MQTT broker via mDNS
- **Default Broker**: `mqtt://192.168.0.100:1883` with automatic fallback

### Available Commands
```bash
# Complete Development Stack
make dev-stack-start    # Start Ollama + MQTT with service discovery
make dev-stack-stop     # Stop complete development stack

# Individual Services
make ollama-start       # Start Ollama AI backend
make mosquitto-start    # Start MQTT broker
make ollama-stop        # Stop Ollama services
make mosquitto-stop     # Stop MQTT services

# Development Workflow
make develop-main       # Build, flash, monitor main controller
make develop-cam        # Build, flash, monitor camera module
```

## Software Architecture

This project implements a **Clean Architecture** approach with strict separation of concerns, hardware abstraction layers, and type-safe interfaces. The architecture prioritizes maintainability, testability, and modularity while avoiding common anti-patterns.

### System Components
- **Main Controller**: Motor control, LED management, servo control (Heltec WiFi LoRa 32)
- **AI Vision System**: Camera, AI processing, MQTT logging (ESP32-CAM)
- **Development Stack**: Ollama AI backend + Mosquitto MQTT broker with mDNS
- **Communication**: Serial inter-board + MQTT telemetry + AI API integration

