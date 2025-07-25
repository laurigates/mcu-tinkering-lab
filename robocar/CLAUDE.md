# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a sophisticated dual-ESP32 autonomous robot car with AI-powered vision. The system features a pluggable AI backend architecture supporting both cloud-based and self-hosted AI services, with advanced hardware control capabilities.

**Key Architecture Components:**
1. **Main Controller** (`idf-robocar/`) - Heltec WiFi LoRa 32 V1 with motor control, OLED display, and I2C slave
2. **AI Vision System** (`esp32-cam-idf/`) - ESP32-CAM with pluggable AI backends and I2C master communication
3. **Structured I2C Protocol** - Packet-based inter-board communication with checksums and error handling
4. **Pluggable AI Backend** - Modular architecture supporting Claude API, Ollama, and easy extensibility

**Hardware Systems:**
- **Motor Control**: TB6612FNG H-bridge with precise PWM speed control
- **Visual Feedback**: PCA9685-driven RGB LEDs, servo camera mount, integrated OLED display
- **Audio System**: Piezo buzzer with multiple sound patterns
- **Power Management**: 7.4V LiPo battery with safety timeout systems

## Build and Development Commands

**Framework:** ESP-IDF v5.4+ (primary implementation). ESP-IDF is installed at `~/repos/esp-idf`. The Makefiles automatically handle environment setup for different shells:

### Top-Level Commands (Recommended)
```bash
# Build both controllers
make build-main          # Build main controller only
make build-cam           # Build camera module only
make build-all           # Build both projects

# Flash firmware (camera requires GPIO0 to GND)
make flash-main          # Flash main controller
make flash-cam           # Flash camera (GPIO0->GND required)

# Development workflow (build + flash + monitor)
make develop-main        # Flash and monitor main controller
make develop-cam         # Flash and monitor camera module

# Monitoring and utilities
make monitor-main        # Monitor main controller serial
make monitor-cam         # Monitor camera module serial
make credentials         # Setup camera WiFi/API credentials
make info               # Show system configuration
```

### Individual Project Commands
```bash
# Main controller (idf-robocar/)
cd idf-robocar
make build              # Build firmware
make flash              # Flash to device
make monitor            # Monitor serial output
make flash-monitor      # Flash and monitor

# Camera module (esp32-cam-idf/)
cd esp32-cam-idf
make credentials        # Create credentials.h from template
make build              # Build firmware  
make flash              # Flash to device (GPIO0->GND required)
make monitor            # Monitor serial output
```

**Monitor Exit:** `Ctrl+]` to exit ESP-IDF monitor (or `Ctrl+T Ctrl+X` if Ctrl+] unavailable)

**Important Note:** If monitor seems unresponsive, try:
1. `Ctrl+]` (primary exit method for ESP-IDF monitor)
2. `Ctrl+T Ctrl+X` (alternative if bracket key unavailable)
3. `Ctrl+C` followed by `Ctrl+]` if process is stuck
4. In terminal: `Ctrl+Z` to suspend, then `kill %1` to terminate

**Shell Compatibility:** Makefiles auto-detect Fish vs Bash/Zsh and handle ESP-IDF environment setup appropriately.

## Hardware Configuration

### ESP32-CAM Programming Requirements
The ESP32-CAM requires special programming procedure:
1. **Connect GPIO0 to GND** before powering on
2. **Run flash command** (`make flash-cam` or `make develop-cam`)
3. **Disconnect GPIO0 from GND** and reset after successful upload

### Pin Configuration Files
- **Main Controller**: Pin definitions in `idf-robocar/main/pin_config_idf.h`
- **ESP32-CAM**: Pin mappings in `esp32-cam-idf/main/camera_pins.h`

**Key Pin Assignments:**
- **Motors (TB6612FNG)**: PWM and direction control via GPIO 23,2,4,13,15,12,25
- **I2C Communication**: SDA=21, SCL=22 (PCA9685 and OLED)
- **Inter-board I2C**: SDA=15, SCL=14 (ESP32-CAM to main controller)
- **OLED Display**: Integrated SSD1306 on Heltec board
- **Camera**: OV2640 sensor with dedicated pins on ESP32-CAM

## Software Architecture

This project implements a **Clean Architecture** approach with strict separation of concerns, hardware abstraction layers, and type-safe interfaces. The architecture prioritizes maintainability, testability, and modularity while avoiding common anti-patterns.

### Core Architectural Principles

**Design Patterns Implemented:**
- **Hardware Abstraction Layer (HAL)**: Complete abstraction of hardware components
- **Command Pattern**: Type-safe command processing with O(1) lookup tables
- **Single Responsibility Principle**: Each module has one clearly defined purpose
- **Dependency Injection**: Modular AI backend architecture
- **Observer Pattern**: Real-time display updates and action tracking

**Anti-Patterns Eliminated:**
- **God Object**: Monolithic main controller decomposed into focused modules
- **String-based Commands**: Replaced with type-safe enumerated command system
- **Direct Hardware Access**: All hardware access goes through abstraction layers

### Hardware Abstraction Layer (HAL)

The main controller implements a comprehensive HAL that abstracts all hardware components:

**Motor Controller (`motor_controller.c/.h`)**
- TB6612FNG H-bridge abstraction with movement primitives
- Speed control, direction management, safety timeouts
- High-level functions: `motor_move_forward()`, `motor_turn_left()`, `motor_stop()`

**Display Manager (`display_manager.c/.h`)**
- SSD1306 OLED abstraction with real-time debugging capabilities
- Action tracking, timing analysis, multi-source command identification
- Functions: `display_track_action()`, `display_update_status()`, `display_show_servo_position()`

**LED Controller (`led_controller.c/.h`)**
- PCA9685 RGB LED abstraction with advanced blinking patterns
- Color management, timer-based effects, position control
- Functions: `led_set_color()`, `led_blink()`, `led_turn_off_all()`

**Servo Controller (`servo_controller.c/.h`)**
- Pan/tilt servo abstraction with smooth motion capabilities
- Angle validation, position tracking, motion control
- Functions: `servo_set_angle()`, `servo_move_smooth()`, `servo_sweep()`

### Command Pattern Implementation

**Type-Safe Command System (`command_handler.c/.h`)**
- Replaces 127-line string parsing with O(1) lookup table architecture
- 35+ structured commands with parameter validation
- Command types: `CMD_TYPE_MOVE`, `CMD_TYPE_SERVO`, `CMD_TYPE_LED`, `CMD_TYPE_SOUND`
- 67% complexity reduction in command processing

**Command Structure:**
```c
typedef struct {
    command_type_t type;        // Type-safe command category
    uint8_t subtype;           // Specific command within category
    command_params_t params;   // Validated parameters
    char source[16];           // Command source tracking
    uint32_t timestamp;        // Execution timestamp
} robot_command_t;
```

**Lookup Table Architecture:**
- Function pointer-based command dispatch
- Parameter count validation
- Built-in command documentation
- Source tracking for multi-input debugging

### Pluggable AI Backend System
**Core Innovation**: Modular AI backend architecture allowing runtime switching between different AI services.

**Available Backends:**
- **Claude API**: High-quality commercial vision analysis via Anthropic's API
- **Ollama**: Self-hosted inference with local models (privacy-focused, offline-capable)
- **Extensible**: Easy to add new AI services via standard interface

**Configuration:** Backend selection and settings in `esp32-cam-idf/main/config.h`
```c
// Choose backend (comment/uncomment as needed)
#define CONFIG_AI_BACKEND_CLAUDE
// #define CONFIG_AI_BACKEND_OLLAMA

// Backend-specific settings
#define CLAUDE_MODEL "claude-3-haiku-20240307"
#define OLLAMA_MODEL "gemma3:4b"
```

### Structured I2C Communication Protocol
**Protocol Features:**
- **Packet-based**: Structured commands with checksums and error handling  
- **Type-safe**: Enumerated command types (movement, sound, servo, display, status)
- **Reliable**: Automatic retry mechanisms and timeout protection
- **Extensible**: Easy to add new command types

**Command Types:** Movement, sound effects, servo control, display updates, status queries, ping/health checks

**Implementation Files:**
- `esp32-cam-idf/main/i2c_protocol.h` - Protocol definitions and packet structures
- `esp32-cam-idf/main/i2c_master.c` - ESP32-CAM I2C master implementation
- `idf-robocar/main/i2c_slave.c` - Main controller I2C slave implementation

### AI Vision Workflow
1. **Image Capture**: ESP32-CAM captures QVGA (320x240) JPEG images
2. **Backend Processing**: Selected AI backend analyzes scene for navigation
3. **Command Generation**: AI responses parsed into structured robot commands
4. **I2C Transmission**: Commands sent to main controller via packet protocol
5. **Hardware Execution**: Main controller executes movement, sound, and visual feedback

## AI Backend Configuration

### Claude API Setup
1. **Obtain API Key** from Anthropic
2. **Configure credentials**:
   ```bash
   cd esp32-cam-idf
   make credentials  # Creates credentials.h from template
   # Edit credentials.h and add: #define CLAUDE_API_KEY "your-key-here"
   ```
3. **Select backend** in `esp32-cam-idf/main/config.h`:
   ```c
   #define CONFIG_AI_BACKEND_CLAUDE
   // #define CONFIG_AI_BACKEND_OLLAMA
   ```

### Ollama Self-Hosted Setup
1. **Install Ollama** on your local machine
2. **Start Ollama service** with a vision model:
   ```bash
   ollama run gemma3:4b  # or llava-phi3, llava:latest
   ```
3. **Enable mDNS service discovery** (optional but recommended):
   ```bash
   # macOS
   dns-sd -R "Ollama AI" _ollama._tcp . 11434 path=/api/generate
   
   # Linux
   avahi-publish-service "Ollama AI" _ollama._tcp 11434 path=/api/generate
   ```
4. **Configure backend** in `esp32-cam-idf/main/config.h`:
   ```c
   // #define CONFIG_AI_BACKEND_CLAUDE
   #define CONFIG_AI_BACKEND_OLLAMA
   
   #define OLLAMA_API_URL "http://192.168.0.115:11434/api/generate"
   #define OLLAMA_MODEL "gemma3:4b"
   #define OLLAMA_USE_SERVICE_DISCOVERY 1  // Enable auto-discovery
   ```

**Service Discovery:** The system automatically discovers Ollama servers via mDNS, eliminating hardcoded IP addresses. See `docs/OLLAMA_SERVICE_DISCOVERY.md` for detailed setup instructions.

## ESP-IDF Component Dependencies

**Native ESP-IDF Components:**
- `esp_lcd` - OLED display driver (SSD1306)
- `driver` - GPIO, I2C, UART, PWM hardware drivers
- `esp_camera` - Camera sensor interface
- `esp_http_client` - HTTP client for AI API calls
- `mdns` - Service discovery for Ollama backends

**External Components** (via ESP-IDF Component Manager):
- `pca9685` - PWM driver for RGB LEDs and servos
- `i2cdev` - I2C device abstraction layer

## Documentation Resources

### Context7 MCP Documentation Access
When working with ESP-IDF, use the Context7 MCP tool to access official documentation:

**ESP-IDF Library IDs for Context7:**
- `/espressif/esp-idf` - Main ESP-IDF framework documentation
- `/espressif/idf-component-manager` - Component manager documentation  
- `/espressif/idf-extra-components` - Additional ESP-IDF components

**Graphics and Display Libraries:**
- `/lvgl/docs` - LVGL graphics library documentation (3,671 snippets)
- `/lvgl/lvgl` - LVGL core library and examples (666 snippets)
- `/lilygo/esp32-oled0.96-ssd1306` - ESP32 + SSD1306 hardware examples (31 snippets)

**System and RTOS:**
- `/raspberrypi/freertos-kernel` - FreeRTOS kernel documentation (112 snippets)

**Networking and IoT:**
- `/espressif/esp-rainmaker` - ESP RainMaker IoT platform (183 snippets)
- `/esp32async/asynctcp` - Async TCP library for ESP32
- `/esp32async/espasyncwebserver` - Async web server for ESP32

**Usage Example:**
```
Use mcp__context7__get-library-docs with context7CompatibleLibraryID="/espressif/esp-idf" 
and topic="esp_lcd" to get detailed LCD driver documentation.

Use mcp__context7__get-library-docs with context7CompatibleLibraryID="/lvgl/docs"
and topic="display driver" to get LVGL integration examples.
```

This provides up-to-date official documentation for:
- Hardware drivers (GPIO, I2C, SPI, UART, etc.)
- Display drivers (esp_lcd, SSD1306 support)
- Graphics libraries (LVGL for advanced UI)
- Real-time operating system (FreeRTOS tasks, timers, queues)
- IoT platforms and networking
- Component configuration and usage
- API references and examples

## Development Guidelines

### Architectural Compliance

**HAL Module Standards:**
- All hardware modules must implement initialization, operation, and status functions
- Functions must return `esp_err_t` for consistent error handling
- Hardware state must be encapsulated in static structures
- No direct hardware access outside designated HAL modules

**Command System Standards:**
- All commands must use the Command Pattern lookup table system
- String-based command parsing is prohibited in new code
- Parameters must be validated using type-safe enumerations
- Command source tracking is mandatory for debugging

**Code Organization Standards:**
- Maximum function length: 200 lines
- Single responsibility principle enforcement
- Zero tolerance for God Object anti-pattern
- All modules must be unit testable

### Hardware Compatibility
- Main code uses ESP-IDF native functions for maximum performance and reliability
- Hardware abstraction layers enable easy porting to different microcontrollers
- Pin assignments are centralized in `pin_config_idf.h` for easy hardware modifications

### Safety Features
- Command timeout automatically stops motors if no commands received
- Motors stop on serial communication loss
- All movement commands include automatic timeout
- Hardware abstraction prevents invalid pin operations

### OLED Debugging Display

The main controller features a comprehensive real-time debugging display on the integrated SSD1306 OLED (128x64 pixels). This provides immediate visual feedback for development and troubleshooting.

**Display Layout (8 lines, 16 characters each):**
```
Line 0: ROBO #[counter]      - System status with action counter
Line 1: [STATE] [SOURCE]     - Current state and command source  
Line 2: ACT:[action]         - Last action taken with parameters
Line 3: PAN:[angle] TLT:[angle] - Real-time servo positions
Line 4: CMD:[time]ms ago     - Time elapsed since last command
Line 5-7: [Available]        - Reserved for Claude messages/alerts
```

**Real-time Action Tracking:**
- **Action Counter**: Total actions performed (`ROBO #123`)
- **Movement States**: FWD, BACK, LEFT, RGHT, CW, CCW, STOP
- **Command Sources**: 
  - `[UART]` - Serial console commands
  - `[I2C]` - ESP32-CAM AI commands
  - `[MOTOR]` - Internal motor operations
  - `[SERVO]` - Servo movements
  - `[SOUND]` - Audio feedback
- **Action Details**: Speed values, angles, command types
- **Timing**: Command frequency and response analysis

**Debug Benefits:**
- Immediate visual feedback for all robot operations
- Command source identification for multi-input debugging
- Action history tracking with parameters
- Real-time state verification
- Performance monitoring via timing displays

**Implementation:** The display automatically switches from startup messages to debug mode after initialization, providing continuous operational visibility.

### Architectural Quality Standards

**Code Quality Enforcement:**
- **No God Objects**: Maximum 200 lines per function, single responsibility enforcement
- **Type Safety**: All commands use enumerated types, no string-based operations
- **Hardware Abstraction**: Zero direct hardware access outside HAL modules
- **Error Handling**: All functions return `esp_err_t` with proper error propagation
- **Memory Safety**: No dynamic allocation, stack-based structures only

**Testing Standards:**
- **Unit Testable**: HAL modules can be tested independently
- **Integration Testing**: Command flow can be tested end-to-end
- **Hardware Mocking**: Abstract interfaces allow hardware simulation

**Performance Standards:**
- **O(1) Command Resolution**: Lookup table eliminates string parsing overhead
- **Real-time Constraints**: All critical paths complete within 10ms
- **Memory Efficiency**: Static allocation, minimal stack usage

### Key Source Files

**Main Controller Hardware Abstraction (`idf-robocar/main/`):**
- `motor_controller.c/.h` - TB6612FNG H-bridge abstraction (350+ lines)
- `display_manager.c/.h` - SSD1306 OLED with real-time debugging (450+ lines)
- `led_controller.c/.h` - PCA9685 RGB LED control with advanced effects (420+ lines)
- `servo_controller.c/.h` - Pan/tilt servo control with smooth motion (377 lines)
- `command_handler.c/.h` - Command Pattern with O(1) lookup tables (483 lines)
- `main.c` - Main application loop, task management, I2C slave handler
- `pin_config_idf.h` - Hardware pin definitions and motor control constants
- `i2c_slave.c` - I2C slave implementation for receiving commands from ESP32-CAM

**ESP32-CAM Vision System (`esp32-cam-idf/main/`):**
- `main.c` - Camera capture, AI backend integration, I2C master communication
- `ai_backend.c/.h` - Generic AI interface and backend selector
- `claude_backend.c/.h` - Claude API implementation
- `ollama_backend.c/.h` - Ollama server implementation with service discovery
- `i2c_master.c` - I2C master for sending commands to main controller
- `i2c_protocol.c/.h` - Structured packet protocol implementation
- `config.h` - System configuration and backend selection

**Architecture Documentation (`docs/`):**
- `PLUGGABLE_AI.md` - AI backend architecture and extensibility guide
- `OLLAMA_SERVICE_DISCOVERY.md` - mDNS service discovery setup and troubleshooting

## Testing and Development Workflow

### System Integration Testing
1. **Flash both controllers**:
   ```bash
   make develop-main    # In one terminal
   make develop-cam     # In another terminal (GPIO0->GND first)
   ```
2. **Monitor communication**: Watch I2C packet exchange in serial logs
3. **Test AI responses**: Verify image capture → AI analysis → command execution workflow

### Hardware Verification
1. **I2C Communication**: Check packet transmission with checksum validation
2. **Motor Control**: Test movement commands via I2C protocol  
3. **Visual Feedback**: Verify OLED display updates and LED patterns
4. **AI Integration**: Confirm backend connectivity and service discovery

### Backend Testing
```bash
# Test Claude API connectivity
curl -H "x-api-key: YOUR_KEY" -H "anthropic-version: 2023-06-01" \
     https://api.anthropic.com/v1/messages

# Test Ollama service discovery
avahi-browse -r _ollama._tcp -t  # Linux
dns-sd -B _ollama._tcp          # macOS
```

## Common Issues and Solutions

### ESP32-CAM Programming Issues
- **"Failed to connect"**: Ensure GPIO0 connected to GND before power/reset
- **"Timed out waiting for packet header"**: Use slower baud rate or different USB adapter
- **Post-flash issues**: Disconnect GPIO0 from GND and reset after successful upload

### AI Backend Connectivity
- **"AI backend not available"**: Check network connectivity and service configuration
- **Claude API errors**: Verify API key in `credentials.h` and account credits
- **Ollama connection failed**: Ensure Ollama server running and mDNS service published

### I2C Communication Problems
- **Command timeouts**: Check I2C wiring (SDA=15, SCL=14 on ESP32-CAM)
- **Checksum errors**: Verify packet integrity and electrical connections
- **No response from slave**: Confirm main controller firmware loaded and I2C address (0x42)

### Service Discovery Issues
- **mDNS resolution fails**: Ensure devices on same network and multicast traffic allowed
- **Ollama not discovered**: Verify mDNS service published correctly (`avahi-browse` or `dns-sd`)
- **Fallback not working**: Check static IP configuration in `config.h`