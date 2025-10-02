# ESP32-CAM AI Vision System

ESP-IDF implementation of AI-powered vision system for robot car control using Claude API.

## Features

- ESP32-CAM image capture with optimized settings for robotics
- WiFi connectivity with automatic reconnection
- Claude AI API integration for image analysis
- Real-time robot command generation based on visual input
- Serial communication with main robot controller
- Status LED feedback system
- JSON response parsing for movement, sound, and servo commands

## Hardware Requirements

- ESP32-CAM (AI Thinker variant)
- MicroSD card (optional, for local image storage)
- WiFi network access
- Serial connection to main robot controller

## Pin Configuration

| Function | GPIO Pin |
|----------|----------|
| Camera Data 0-7 | 5, 18, 19, 21, 36, 39, 34, 35 |
| Camera Clock | 0 |
| Camera V/H Sync | 25, 23 |
| Camera PCLK | 22 |
| I2C SDA/SCL | 26, 27 |
| Status LED | 4 |
| UART TX/RX | 17, 16 |
| Power Down | 32 |

## Setup Instructions

### 1. Configure Credentials

Copy the credentials template and add your WiFi and API details:

```bash
cp main/credentials.h.example main/credentials.h
```

Edit `main/credentials.h` with your actual credentials:

```c
#define WIFI_SSID "YourWiFiNetwork"
#define WIFI_PASSWORD "YourWiFiPassword"
#define CLAUDE_API_KEY "sk-ant-api03-..."
```

### 2. Build and Flash

Set up ESP-IDF environment:

```bash
source ~/repos/esp-idf/export.sh
```

Build the project:

```bash
idf.py build
```

Flash to ESP32-CAM (put GPIO0 to GND before flashing):

```bash
idf.py -p /dev/cu.usbserial-0001 flash monitor
```

## System Operation

### Startup Sequence

1. Initialize serial communication with main controller
2. Initialize camera with optimized settings for AI analysis
3. Connect to WiFi network
4. Initialize Claude API client
5. Send startup commands to main controller
6. Begin capture-analyze-command cycle

### AI Analysis Cycle

Every 5 seconds (configurable):

1. Capture image from camera (320x240 JPEG)
2. Encode image to base64
3. Send to Claude API with vision prompt
4. Parse JSON response for commands
5. Send movement/sound/servo commands to main controller
6. Update status LED

### Command Protocol

Commands sent to main controller via UART2 (115200 baud):

**Movement Commands:**
- `F` - Forward
- `B` - Backward  
- `L` - Left
- `R` - Right
- `C` - Rotate clockwise
- `W` - Rotate counter-clockwise
- `S` - Stop

**Sound Commands:**
- `SB` - Beep
- `SM` - Melody
- `SA` - Alert
- `SC:freq:duration` - Custom tone
- `MO:TEXT:message` - Morse code
- `RT:name` - RTTTL ringtone

**Servo Commands:**
- `PAN:angle` - Pan servo (0-180°)
- `TILT:angle` - Tilt servo (0-180°)

### Status LED Indicators

- **Solid On**: System initializing
- **Fast Blink**: System ready, WiFi disconnected
- **Slow Blink**: Normal operation with WiFi
- **Brief Flash**: Image capture and analysis

## Configuration

Edit `main/config.h` to adjust:

- Capture interval (default: 5 seconds)
- Camera quality settings
- WiFi retry intervals
- Debug logging levels

## Troubleshooting

### Camera Issues
- Ensure PSRAM is enabled in sdkconfig
- Check camera module connections
- Verify GPIO0 is disconnected after flashing

### WiFi Issues
- Check credentials in credentials.h
- Verify network connectivity
- Monitor serial output for connection status

### API Issues
- Verify Claude API key is valid
- Check internet connectivity
- Monitor HTTP response codes in logs

### Serial Communication
- Verify UART pins match main controller
- Check baud rate (115200)
- Test with simple commands first

## Memory Usage

- Camera frame buffer: ~25KB (QVGA JPEG)
- Base64 encoded image: ~33KB
- HTTP request/response: ~40KB
- JSON parsing: ~2KB
- Total PSRAM usage: ~100KB

## Performance

- Image capture: ~50ms
- Base64 encoding: ~20ms
- HTTP API call: ~2-5 seconds
- JSON parsing: ~5ms
- Total cycle time: ~3-6 seconds

## Development

### Adding New Commands

1. Extend `ai_response_parser.c` to handle new JSON fields
2. Add command mapping in `parse_ai_response()`
3. Update Claude prompt in `claude_api.c`
4. Test with main controller

### Debugging

Enable verbose logging in `config.h`:
```c
#define ENABLE_VERBOSE_LOGGING 1
```

Monitor all activity:
```bash
idf.py monitor
```

## License

Part of the ESP32 RoboCar project. See main project documentation for license details.