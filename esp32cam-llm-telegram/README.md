# ESP32-CAM LLM Telegram Robot Controller

A test project for ESP32-CAM that captures images, analyzes them using LLMs (Claude or Ollama), and sends the results to Telegram with simulated motor control feedback.

## Features

- **Camera Capture**: Periodic image capture from ESP32-CAM
- **LLM Vision Analysis**: Send images to Claude or Ollama for interpretation
- **Telegram Bot Integration**:
  - Receive images and analysis results
  - Control the robot via commands
  - Get status updates and telemetry
- **Mock Motor Control**: Simulated motor movements based on vision analysis
- **Configurable**: Store settings in NVS flash

## Hardware Requirements

- ESP32-CAM module (AI-Thinker or compatible)
- USB-to-Serial adapter for programming
- 5V power supply

## Software Requirements

- ESP-IDF v5.0 or later
- Python 3.x for ESP-IDF tools
- Telegram Bot Token
- Claude API key or Ollama server

## Setup

### 1. Configure Credentials

Edit `main/config.h` and update:

```c
#define WIFI_SSID_DEFAULT "YOUR_WIFI_SSID"
#define WIFI_PASSWORD_DEFAULT "YOUR_WIFI_PASSWORD"
#define TELEGRAM_BOT_TOKEN "YOUR_BOT_TOKEN"
#define TELEGRAM_CHAT_ID "YOUR_CHAT_ID"
```

For Claude:
```c
#define LLM_BACKEND_TYPE 1
#define CLAUDE_API_KEY "YOUR_CLAUDE_API_KEY"
```

For Ollama:
```c
#define LLM_BACKEND_TYPE 0
#define OLLAMA_SERVER_URL "http://192.168.1.100:11434"
#define OLLAMA_MODEL "llava"
```

### 2. Create Telegram Bot

1. Message @BotFather on Telegram
2. Create a new bot with `/newbot`
3. Save the bot token
4. Get your chat ID by messaging the bot and checking:
   ```
   https://api.telegram.org/bot<YOUR_BOT_TOKEN>/getUpdates
   ```

### 3. Build and Flash

```bash
# Set up ESP-IDF environment
. $IDF_PATH/export.sh

# Configure the project
idf.py set-target esp32
idf.py menuconfig

# Build
idf.py build

# Flash (replace /dev/ttyUSB0 with your port)
idf.py -p /dev/ttyUSB0 flash

# Monitor output
idf.py -p /dev/ttyUSB0 monitor
```

## Telegram Commands

- `/start` - Show help and available commands
- `/capture` - Take a photo and analyze it
- `/forward` - Move forward
- `/backward` - Move backward
- `/left` - Turn left
- `/right` - Turn right
- `/stop` - Stop movement
- `/status` - Get system status
- `/auto [on/off]` - Toggle auto capture mode

## How It Works

1. **Image Capture**: ESP32-CAM captures JPEG images at configured intervals
2. **Vision Analysis**: Images are sent to Claude/Ollama with a prompt asking for:
   - Scene description
   - Object detection
   - Movement suggestions
   - Confidence level
3. **Telegram Updates**: Analysis results and images are sent to your Telegram chat
4. **Motor Control**: Simulated motor commands based on LLM suggestions
5. **Telemetry**: Distance traveled and heading updates sent to Telegram

## Configuration Options

Key settings in `main/config.h`:

- `CAMERA_FRAME_SIZE`: Image resolution (default: QVGA 320x240)
- `CAMERA_JPEG_QUALITY`: JPEG compression (0-63, lower = better)
- `CAMERA_CAPTURE_INTERVAL_MS`: Time between captures
- `MOTOR_DEFAULT_SPEED`: Simulated motor speed (0-255)

## Project Structure

```
esp32cam-llm-telegram/
├── CMakeLists.txt          # Main CMake configuration
├── main/
│   ├── CMakeLists.txt      # Component configuration
│   ├── main.c              # Application entry point
│   ├── config.h            # Configuration constants
│   ├── config_manager.*    # NVS configuration storage
│   ├── camera_handler.*    # Camera control
│   ├── telegram_bot.*      # Telegram API client
│   ├── llm_client.*        # Claude/Ollama client
│   ├── motor_controller.*  # Mock motor control
│   └── vision_interpreter.* # Vision to motor commands
├── data/
│   └── prompts.txt         # LLM prompts
├── partitions.csv          # Flash partition table
└── sdkconfig.defaults      # ESP-IDF configuration

```

## Testing

1. Power on the ESP32-CAM
2. Wait for WiFi connection and bot initialization
3. Open Telegram and message your bot
4. Send `/capture` to test image capture and analysis
5. Try movement commands like `/forward` or `/left`
6. Enable auto mode with `/auto on` for continuous operation

## Troubleshooting

- **Camera Init Failed**: Check camera module connection, ensure PSRAM is enabled
- **WiFi Connection Failed**: Verify SSID and password in config
- **Telegram Not Responding**: Check bot token and internet connectivity
- **LLM Timeout**: Ensure Claude API key is valid or Ollama server is running
- **Out of Memory**: Reduce image quality or frame size

## Notes

- Motor control is simulated - no actual motor pins are driven
- Distance and heading are calculated estimates
- LLM responses may vary in quality depending on image clarity
- Consider API rate limits when setting capture intervals

## License

This is a test/demo project for educational purposes.