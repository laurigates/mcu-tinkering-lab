# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

This is an embedded systems monorepo containing projects for various microcontroller platforms (ESP32, Arduino, STM32). The repository is organized by platform and functionality, with a focus on ESP32-based projects including AI-powered robotics and camera systems.

**Repository Structure:**
- **`robocar/`** - Complete dual-ESP32 AI-powered autonomous robot car project
- **`packages/`** - Individual projects organized by platform
- **`docs/`** - Global documentation (currently empty)
- **`tools/`** - Build scripts and utilities (currently empty)

## Primary Project: AI-Powered Robot Car

The main project is a sophisticated autonomous robot car in the `robocar/` directory featuring:

**Architecture:**
- **Dual ESP32 System**: Main controller (Heltec WiFi LoRa 32 V1) + ESP32-CAM vision module
- **Pluggable AI Backend**: Supports Claude API and Ollama (self-hosted) for scene analysis
- **Structured I2C Communication**: Packet-based protocol with checksums between controllers
- **Hardware Control**: Motor control, RGB LEDs, servo camera mount, OLED display, audio feedback

**Key Features:**
- AI-powered autonomous navigation using computer vision
- Real-time scene analysis and command generation
- Inter-board communication via I2C protocol
- Modular AI backend architecture for extensibility

## Build System and Commands

**Framework:** ESP-IDF v5.4+ (installed at `~/repos/esp-idf`)

### Robocar Project Commands (Primary Development)
From `/robocar/` directory:
```bash
# Build and flash both controllers
make build-all           # Build main controller and camera module
make flash-main          # Flash main controller
make flash-cam           # Flash camera module (GPIO0->GND required)

# Development workflow (build + flash + monitor)
make develop-main        # Complete workflow for main controller
make develop-cam         # Complete workflow for camera module

# Individual operations
make build-main          # Build main controller only
make build-cam           # Build camera module only
make monitor-main        # Monitor main controller serial output
make monitor-cam         # Monitor camera module serial output
make credentials         # Setup camera WiFi/API credentials
make info               # Show system configuration and status
```

### Package Projects
Individual ESP32 projects in `packages/esp32-projects/`:

**ESP32-CAM Webserver** (`packages/esp32-projects/esp32-cam-webserver/`):
- Live video streaming via HTTP server
- Web interface for camera viewing
- Standard ESP-IDF build system

**ESP32-CAM I2S Audio** (`packages/esp32-projects/esp32-cam-i2s-audio/`):
- Placeholder project for camera + audio processing
- Basic ESP-IDF structure, requires implementation

```bash
# Standard ESP-IDF commands for package projects
cd packages/esp32-projects/esp32-cam-webserver
idf.py build             # Build project
idf.py flash             # Flash to device
idf.py monitor           # Monitor serial output
```

**ESP32-CAM Programming Note:** All ESP32-CAM modules require GPIO0 connected to GND during programming, then disconnected after flashing.

## Software Architecture

### Robocar System Design

**Main Controller (idf-robocar/):**
- Motor control via TB6612FNG H-bridge
- RGB LED management through PCA9685 PWM controller
- Servo control for camera pan/tilt
- OLED display for system status
- I2C slave for receiving commands from ESP32-CAM
- Audio feedback via piezo buzzer

**AI Vision System (esp32-cam-idf/):**
- Image capture using OV2640 camera sensor
- Pluggable AI backend architecture:
  - **Claude API**: Commercial vision analysis via Anthropic
  - **Ollama**: Self-hosted inference with local models (privacy-focused)
- WiFi connectivity and service discovery
- I2C master for sending commands to main controller
- Structured packet communication protocol

### AI Backend Configuration

**Backend Selection:** Configure in `esp32-cam-idf/main/config.h`:
```c
// Choose backend (comment/uncomment as needed)
#define CONFIG_AI_BACKEND_CLAUDE
// #define CONFIG_AI_BACKEND_OLLAMA
```

**Claude API Setup:**
1. Obtain API key from Anthropic
2. Run `make credentials` in esp32-cam-idf/ directory
3. Edit `credentials.h` with API key

**Ollama Setup:**
1. Install and run Ollama server with vision model
2. Configure API URL and model in `config.h`
3. Optional: Enable mDNS service discovery for automatic server detection

### I2C Communication Protocol

**Features:**
- Packet-based commands with checksums
- Command types: movement, sound, servo control, display updates, status
- Automatic retry mechanisms and timeout protection
- Type-safe enumerated command structure

**Implementation Files:**
- `esp32-cam-idf/main/i2c_protocol.h` - Protocol definitions
- `esp32-cam-idf/main/i2c_master.c` - ESP32-CAM implementation
- `idf-robocar/main/i2c_slave.c` - Main controller implementation

## Hardware Configuration

### Pin Assignments

**Main Controller (Heltec WiFi LoRa 32 V1):**
- Motors (TB6612FNG): PWM and direction control via GPIO 23,2,4,13,15,12,25
- I2C (PCA9685/OLED): SDA=21, SCL=22
- Inter-board I2C: SDA=15, SCL=14
- Integrated OLED: SSD1306 display

**ESP32-CAM Module:**
- Camera: OV2640 sensor with dedicated pins
- Inter-board I2C: SDA=15, SCL=14
- Status LED: GPIO 4

**Pin Configuration Files:**
- Main controller: `idf-robocar/main/pin_config_idf.h`
- ESP32-CAM: `esp32-cam-idf/main/camera_pins.h`

### Hardware Components
- **Motor Driver**: TB6612FNG dual H-bridge
- **PWM Controller**: PCA9685 16-channel (RGB LEDs, servos)
- **Camera**: OV2640 2MP sensor
- **Display**: Integrated 128x64 OLED (SSD1306)
- **Audio**: Piezo buzzer for sound effects
- **Power**: 7.4V LiPo battery recommended

## Development Environment

### ESP-IDF Dependencies
**Native Components:**
- `esp_lcd` - OLED display driver
- `driver` - GPIO, I2C, UART, PWM hardware drivers  
- `esp_camera` - Camera sensor interface
- `esp_http_client` - HTTP client for AI API calls
- `mdns` - Service discovery for Ollama backends

**External Components** (ESP-IDF Component Manager):
- `pca9685` - PWM driver for LEDs and servos
- `i2cdev` - I2C device abstraction

### Monitor and Debugging
**Serial Monitor Exit:** Use `Ctrl+]` to exit ESP-IDF monitor (or `Ctrl+T Ctrl+X` if unavailable)

**Shell Compatibility:** Makefiles auto-detect Fish vs Bash/Zsh shells and handle ESP-IDF environment setup appropriately

**Common Debug Commands:**
```bash
make monitor-main        # Watch main controller operation
make monitor-cam         # Watch camera module and AI processing
make info               # System configuration check
```

## Package Projects Structure

### ESP32-CAM Webserver
**Location:** `packages/esp32-projects/esp32-cam-webserver/`
**Purpose:** Live video streaming web server
**Features:**
- HTTP server with live MJPEG streaming
- Web interface for camera viewing
- ~24 fps video stream at VGA resolution
- WiFi connectivity with configurable credentials

### ESP32-CAM I2S Audio
**Location:** `packages/esp32-projects/esp32-cam-i2s-audio/`
**Status:** Placeholder project with basic structure
**Planned Features:**
- Camera image capture
- HTTP communication with server
- I2S audio output via MAX98357A amplifier
- Audio data processing and playback

## Common Issues and Solutions

### ESP32-CAM Programming
- **"Failed to connect"**: Ensure GPIO0 connected to GND before power/reset
- **Post-flash issues**: Disconnect GPIO0 from GND and reset after upload
- **Brownout detector**: Use quality USB cable and ensure adequate power supply

### AI Backend Connectivity
- **Claude API errors**: Verify API key in `credentials.h` and check account limits
- **Ollama connection failed**: Ensure server running and network connectivity
- **Service discovery issues**: Check mDNS configuration and network multicast support

### I2C Communication
- **Command timeouts**: Verify I2C wiring (SDA=15, SCL=14)
- **Checksum errors**: Check electrical connections and packet integrity
- **No slave response**: Confirm main controller firmware and I2C address (0x42)

## Future Development

### Planned Enhancements
- Extended sensor integration (ultrasonic, IMU, GPS)
- SLAM capabilities and mapping
- Voice control integration
- Mobile app interface
- Multi-robot coordination

### Extensibility
- AI backend system designed for easy addition of new services
- Modular I2C protocol supports new command types
- Hardware abstraction allows different motor controllers and sensors

## Documentation Resources

**Project-Specific:**
- `robocar/README.md` - Complete system overview and setup
- `robocar/docs/` - Detailed technical documentation
- `robocar/CLAUDE.md` - Robocar-specific guidance (comprehensive)

**Official ESP-IDF Documentation:**
- Use Context7 MCP integration for up-to-date ESP-IDF documentation
- Library ID: `/espressif/esp-idf` for framework documentation
- Topic-specific queries supported (e.g., "esp_camera", "i2c_driver")