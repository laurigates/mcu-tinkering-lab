# esp32-cam-webserver

Live MJPEG video streaming web server for the ESP32-CAM (AI-Thinker) module.

[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.4-blue)](https://docs.espressif.com/projects/esp-idf/en/v5.4/)
[![Target](https://img.shields.io/badge/target-ESP32-green)](https://www.espressif.com/en/products/socs/esp32)
[![CI](https://img.shields.io/github/actions/workflow/status/laurigates/mcu-tinkering-lab/esp32-build.yml?label=build)](https://github.com/laurigates/mcu-tinkering-lab/actions/workflows/esp32-build.yml)

## Features

- MJPEG streaming at ~24 fps over HTTP (`/stream` endpoint)
- Embedded HTML viewer at root (`/`) for browser access
- OV2640 camera at SVGA (800x600) resolution
- WiFi station mode with auto-reconnect (up to 5 retries)
- Brownout protection tuned for ESP32-CAM power characteristics
- Reduced WiFi TX power to prevent brownout during initialization

## Tech Stack

| Component | Technology |
|-----------|-----------|
| MCU | ESP32 (AI-Thinker ESP32-CAM) |
| Camera | OV2640 via `espressif/esp32-camera` component |
| Framework | ESP-IDF v5.4+ |
| Build | CMake (via ESP-IDF), containerized with Docker |
| Task runner | just |

## Prerequisites

- [Docker](https://www.docker.com/) (all builds run in containers — no local ESP-IDF needed)
- [just](https://github.com/casey/just) command runner
- USB-to-serial adapter for flashing (CP2102, FTDI, or similar)
- [esptool](https://github.com/espressif/esptool) for flashing firmware

## Getting Started

### 1. Configure WiFi credentials

```bash
just credentials
# Then edit main/credentials.h with your WiFi SSID and password
```

### 2. Build

```bash
just build
```

### 3. Flash

Connect GPIO0 to GND on the ESP32-CAM to enter programming mode, then:

```bash
just flash
# After flashing, disconnect GPIO0 from GND and reset the board
```

### 4. Monitor

```bash
just monitor
```

The serial output will show the assigned IP address. Open `http://<ip-address>/` in a browser to view the live stream.

### Development cycle

```bash
# Build, flash, and monitor in one step
just develop
```

## Project Structure

```
esp32-cam-webserver/
├── main/
│   ├── main.c                 # Application entry point, WiFi, camera, HTTP server
│   ├── credentials.h.example  # WiFi credentials template
│   ├── credentials.h          # Your WiFi credentials (gitignored)
│   ├── CMakeLists.txt         # Component build config
│   └── idf_component.yml     # ESP-IDF component dependencies
├── CMakeLists.txt             # Top-level project CMake
├── sdkconfig.defaults         # ESP-IDF configuration defaults
└── justfile                   # Build/flash/monitor recipes
```

## HTTP Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | HTML page with embedded stream viewer |
| `/stream` | GET | Raw MJPEG stream (`multipart/x-mixed-replace`) |

## Available Commands

```bash
just --list       # Show all recipes
just build        # Build firmware (containerized)
just clean        # Clean build artifacts
just menuconfig   # Open ESP-IDF configuration menu
just flash        # Flash firmware to device
just monitor      # Serial monitor (Ctrl-C to stop)
just develop      # Build + flash + monitor
just lint         # Run cppcheck
just format       # Format with clang-format
just info         # Show project and programming info
```

## Part of mcu-tinkering-lab

This project is a subpackage of [mcu-tinkering-lab](https://github.com/laurigates/mcu-tinkering-lab), an embedded systems monorepo for ESP32 projects.
