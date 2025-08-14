# ESP32 AI-Powered Robot Car

An ESP-IDF-powered autonomous robot car with AI vision capabilities and comprehensive MQTT logging, featuring dual ESP32 controllers, motor control, LED indicators, servo-controlled camera, AI-powered navigation, and real-time telemetry.

## Architecture

The system consists of two ESP32 boards working together:

1. **Main Controller** - Heltec WiFi LoRa 32 V1
   - Motor control and movement execution
   - RGB LED management via PCA9685
   - Servo control for camera pan/tilt
   - Sound effects and piezo buzzer
   - Serial command processing

2. **AI Vision System** - ESP32-CAM (AI Thinker)
   - Image capture and processing
   - Pluggable AI backend (Claude API or Ollama)
   - WiFi connectivity for cloud/local AI services
   - MQTT logging with real-time telemetry
   - Command generation based on visual input
   - Serial communication with main controller

## Hardware Specifications

### Main Controller (Heltec WiFi LoRa 32 V1)
- **Microcontroller**: ESP32-S3 dual-core processor
- **Display**: 128x64 OLED (SSD1306)
- **Connectivity**: WiFi, Bluetooth, LoRa
- **GPIO**: 36 pins available
- **Power**: USB-C or battery connector

### ESP32-CAM Module
- **Microcontroller**: ESP32 with 4MB PSRAM
- **Camera**: OV2640 with up to 2MP resolution
- **Storage**: MicroSD card slot
- **Connectivity**: WiFi
- **Programming**: Requires GPIO0 to GND for flash mode

### Additional Components
- **Motor Driver**: TB6612FNG dual H-bridge controller
- **PWM Driver**: PCA9685 16-channel PWM controller
- **Motors**: Two DC geared motors with wheels
- **LEDs**: RGB LEDs for turn signals and lighting
- **Servos**: Pan/tilt mechanism for camera positioning
- **Audio**: Piezo buzzer for sound effects
- **Power**: 7.4V LiPo battery (2S) recommended

## Pin Configuration

### Main Controller (ESP-IDF)
```c
// Motor Control (TB6612FNG)
#define MOTOR_RIGHT_PWM_PIN    35
#define MOTOR_RIGHT_IN1_PIN    39  
#define MOTOR_RIGHT_IN2_PIN    34
#define MOTOR_LEFT_PWM_PIN     12
#define MOTOR_LEFT_IN1_PIN     38
#define MOTOR_LEFT_IN2_PIN     37
#define MOTOR_STANDBY_PIN      36

// I2C (PCA9685 & OLED)
#define I2C_SDA_PIN            21
#define I2C_SCL_PIN            22

// Serial Communication
#define UART_RX_PIN            16
#define UART_TX_PIN            17

// Piezo Buzzer
#define PIEZO_PIN              13
```

### ESP32-CAM Configuration
```c
// Camera Pins (OV2640)
#define CAM_PIN_PWDN           32
#define CAM_PIN_RESET          -1
#define CAM_PIN_XCLK           0
#define CAM_PIN_SIOD           26
#define CAM_PIN_SIOC           27
// Data pins: D0-D7, VSYNC, HREF, PCLK

// Serial Communication (Fixed for PSRAM compatibility)
#define UART_TX_PIN            14
#define UART_RX_PIN            15

// Status LED
#define CAM_LED_PIN            4
```

## Software Features

### AI Vision Capabilities
- **Real-time Scene Analysis**: Captures images and sends to Claude API
- **Intelligent Navigation**: AI generates movement commands based on visual input
- **Object Recognition**: Identifies obstacles, paths, and navigation cues
- **Autonomous Operation**: Self-directed movement with minimal human intervention

### Motor Control System
- **Precise Movement**: Forward, backward, left, right, rotation
- **Speed Control**: Variable PWM-based speed adjustment
- **Safety Features**: Automatic timeout and emergency stop
- **Smooth Operation**: Gradual acceleration and deceleration

### Visual Feedback
- **RGB LED Indicators**: Turn signals, status lights, headlights
- **OLED Display**: System status, IP address, diagnostic info
- **Servo Camera Mount**: Pan and tilt control for optimal viewing angles

### Communication Protocol
Commands sent via serial from ESP32-CAM to main controller:
- Movement: `F`, `B`, `L`, `R`, `S` (forward, backward, left, right, stop)
- Rotation: `C`, `W` (clockwise, counter-clockwise)
- Sound: `SB`, `SM`, `SA` (beep, melody, alert)
- Servos: `PAN:90`, `TILT:45` (pan/tilt to specified angles)

## Pluggable AI Backend

The AI Vision System features a modular, pluggable backend that allows you to choose which AI service to use for image analysis. The system is designed to be easily extensible, so you can add support for other AI services in the future.

Currently supported backends:
- **Claude**: Uses the powerful vision models from Anthropic for high-quality scene analysis.
- **Ollama**: Allows you to connect to a self-hosted Ollama server running a multimodal model (like LLaVA), giving you an offline-capable, private AI backend.

For a detailed technical overview of the architecture, see [docs/PLUGGABLE_AI.md](./docs/PLUGGABLE_AI.md).

### Configuring the AI Backend

To select and configure the AI backend, edit `esp32-cam-idf/main/config.h`:

1.  **Choose your backend**: Comment out the backend you don't want to use and make sure the one you want is active.
    ```h
    // In esp32-cam-idf/main/config.h
    
    #define CONFIG_AI_BACKEND_CLAUDE
    // #define CONFIG_AI_BACKEND_OLLAMA
    ```
2.  **Configure API Settings**:
    - For **Claude**: Make sure your `CLAUDE_API_KEY` is set in `esp32-cam-idf/main/credentials.h`.
    - For **Ollama**: Set the `OLLAMA_API_URL` and `OLLAMA_MODEL` in `esp32-cam-idf/main/config.h` to point to your local Ollama server.

## Getting Started

### Prerequisites
- ESP-IDF v5.4+ installed at `~/repos/esp-idf`
- Claude API key from Anthropic (if using Claude backend)
- Ollama installed (if using local AI backend)
- Mosquitto MQTT broker (for logging - auto-started by development stack)
- WiFi network credentials
- USB-to-serial adapter for ESP32-CAM programming

### Quick Setup

1. **Clone and Setup**
   ```bash
   git clone <repository>
   cd robocar_test
   make info  # Check system configuration
   ```

2. **Configure Credentials**
   ```bash
   cd esp32-cam-idf
   cp main/credentials.h.example main/credentials.h
   # Edit credentials.h with your WiFi SSID and password. API keys are also stored here. See the "Pluggable AI Backend" section for more details.
   ```

3. **Start Development Stack**
   ```bash
   make dev-stack-start     # Start Ollama AI + MQTT broker with service discovery
   ```

4. **Build and Flash Main Controller**
   ```bash
   make build-main
   make flash-main
   ```

5. **Build and Flash ESP32-CAM**
   ```bash
   # Connect GPIO0 to GND for programming mode
   make build-cam
   make flash-cam
   # Disconnect GPIO0 and reset
   ```

6. **Monitor Operation**
   ```bash
   make monitor-main        # Monitor main controller
   make monitor-cam         # Monitor camera module
   # MQTT logs available at: mqtt://192.168.0.100:1883/robocar/logs
   ```

## Development Commands

### Complete Development Stack
```bash
make dev-stack-start     # Start Ollama AI + MQTT broker with service discovery
make dev-stack-stop      # Stop complete development stack
```

### Individual Services
```bash
make ollama-start        # Start Ollama AI backend
make mosquitto-start     # Start MQTT broker
make ollama-stop         # Stop Ollama services
make mosquitto-stop      # Stop MQTT services
```

### ESP-IDF Framework (Primary)
```bash
make build-main          # Build main controller
make build-cam           # Build camera module
make flash-main          # Flash main controller
make flash-cam           # Flash camera module
make monitor-main        # Monitor main controller
make monitor-cam         # Monitor camera module
make clean-main          # Clean main build
make clean-cam           # Clean camera build
```

### Development Shortcuts
```bash
make develop-main        # Build, flash, and monitor main controller
make develop-cam         # Build, flash, and monitor camera module
```

### System Information
```bash
make info                # Show configuration and status
make help                # Show all available commands
```

## System Architecture

### Main Controller Tasks
- **Command Processing**: Receives and executes movement commands
- **Motor Management**: Controls TB6612FNG motor driver
- **LED Control**: Manages PCA9685 PWM outputs for RGB LEDs
- **Servo Control**: Pan/tilt camera positioning
- **Sound Generation**: Piezo buzzer for audio feedback
- **Display Updates**: OLED status information

### ESP32-CAM Tasks
- **Image Capture**: Periodic camera snapshots
- **AI Processing**: Pluggable backend (Claude API or Ollama) for scene analysis
- **MQTT Logging**: Real-time telemetry and system status reporting
- **WiFi Management**: Network connectivity and service discovery
- **Command Generation**: Translates AI analysis to movement commands
- **Serial Communication**: Sends commands to main controller

### Inter-Controller Communication
- **Protocol**: UART serial at 115200 baud
- **Connection**: GPIO14/15 on ESP32-CAM to GPIO16/17 on main controller
- **Commands**: Text-based protocol for movement and control
- **Error Handling**: Timeout and retry mechanisms

## AI Integration

The ESP32-CAM integrates with a configurable AI vision backend for intelligent navigation:

1. **Image Capture**: High-resolution photos from OV2640 camera
2. **Base64 Encoding**: Images encoded for API transmission
3. **AI Analysis**: The configured backend analyzes the scene for navigation decisions
4. **Command Parsing**: AI responses converted to robot commands
5. **Execution**: Commands sent to main controller for immediate action

### API Configuration
- **Model**: Configurable based on the selected backend (e.g., `claude-3-haiku-20240307` or a local Ollama model like `llava-phi3`).
- **Image Format**: JPEG with base64 encoding
- **Prompt Engineering**: Optimized for robotics navigation tasks
- **Rate Limiting**: Configurable capture intervals to manage API usage

## Hardware Assembly

### Wiring Connections
1. **Power Distribution**: 7.4V battery to motor driver, 3.3V to ESP32s
2. **Motor Connections**: TB6612FNG to motors and main ESP32
3. **I2C Bus**: PCA9685 and OLED to main ESP32 (SDA/SCL)
4. **Serial Link**: ESP32-CAM to main ESP32 (cross-connected TX/RX)
5. **Camera Mount**: Servos connected to PCA9685 outputs

### Programming Setup
- **Main Controller**: Direct USB connection
- **ESP32-CAM**: Requires GPIO0 to GND during programming
- **Serial Adapter**: FTDI or similar for ESP32-CAM programming

## MQTT Logging System

The ESP32-CAM includes comprehensive MQTT logging for real-time telemetry and debugging:

### Features
- **JSON-formatted messages** with timestamps, log levels, and system metrics
- **Offline buffering** when MQTT broker unavailable
- **Service discovery** via mDNS for automatic broker detection
- **Configurable log levels** and topic structure
- **System metrics** including heap memory and WiFi signal strength

### Default Configuration
- **Broker URI**: `mqtt://192.168.0.100:1883`
- **Log Topic**: `robocar/logs`
- **Status Topic**: `robocar/status`
- **Service Type**: `_mqtt._tcp` (mDNS)

### Viewing Logs
```bash
# Start MQTT broker and subscribe to logs
make mosquitto-start
mosquitto_sub -h 192.168.0.100 -t "robocar/logs" -v

# Or use any MQTT client to connect to the broker
```

## Troubleshooting

### Common Issues
- **ESP32-CAM won't program**: Ensure GPIO0 connected to GND during flash
- **Serial communication fails**: Check cross-connected TX/RX wires
- **WiFi connection problems**: Verify credentials in credentials.h
- **Motor not moving**: Check power supply and TB6612FNG connections
- **OLED not displaying**: Verify I2C connections and address (0x3C)
- **MQTT not connecting**: Check broker is running with `make mosquitto-start`
- **Service discovery fails**: Verify mDNS is working on your network

### Debug Commands
```bash
make monitor-main        # Check main controller logs
make monitor-cam         # Check camera module logs
make clean-cam && make build-cam  # Clean rebuild camera
make dev-stack-start     # Start complete development environment
```

## Future Enhancements

- **Advanced AI Models**: Integration with more sophisticated vision models
- **SLAM Capabilities**: Simultaneous localization and mapping
- **Voice Control**: Audio command recognition
- **Mobile App**: Smartphone control interface
- **Multi-Robot Coordination**: Swarm robotics capabilities
- **Extended Sensors**: Ultrasonic, IMU, GPS integration

## License

This project is open source. See LICENSE file for details.

## Contributing

Contributions welcome! Please read CONTRIBUTING.md for guidelines.