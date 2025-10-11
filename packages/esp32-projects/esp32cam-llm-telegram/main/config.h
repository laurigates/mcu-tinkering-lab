#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID_DEFAULT "YOUR_WIFI_SSID"
#define WIFI_PASSWORD_DEFAULT "YOUR_WIFI_PASSWORD"
#define WIFI_MAXIMUM_RETRY 5

// Telegram Bot Configuration
#define TELEGRAM_BOT_TOKEN "YOUR_BOT_TOKEN"
#define TELEGRAM_CHAT_ID "YOUR_CHAT_ID"
#define TELEGRAM_API_URL "https://api.telegram.org/bot"
#define TELEGRAM_MAX_MESSAGE_LENGTH 4096
#define TELEGRAM_POLL_INTERVAL_MS 2000

// LLM Configuration (Claude/Ollama)
#define LLM_BACKEND_TYPE 1  // 0=Ollama, 1=Claude
#define CLAUDE_API_KEY "YOUR_CLAUDE_API_KEY"
#define CLAUDE_API_URL "https://api.anthropic.com/v1/messages"
#define CLAUDE_MODEL "claude-3-haiku-20240307"
#define OLLAMA_SERVER_URL "http://192.168.1.100:11434"
#define OLLAMA_MODEL "llava"

// Camera Configuration
#define CAMERA_MODEL_AI_THINKER 1
#define CAMERA_FRAME_SIZE FRAMESIZE_QVGA  // 320x240
#define CAMERA_JPEG_QUALITY 10  // 0-63 (lower = higher quality)
#define CAMERA_CAPTURE_INTERVAL_MS 5000  // Capture every 5 seconds

// Motor Control Configuration (Mock for testing)
#define MOTOR_LEFT_FORWARD_PIN 12
#define MOTOR_LEFT_BACKWARD_PIN 13
#define MOTOR_RIGHT_FORWARD_PIN 14
#define MOTOR_RIGHT_BACKWARD_PIN 15
#define MOTOR_PWM_FREQUENCY 1000
#define MOTOR_PWM_RESOLUTION 8
#define MOTOR_DEFAULT_SPEED 180  // 0-255

// System Configuration
#define LOG_LEVEL_DEBUG 1
#define TASK_STACK_SIZE 8192
#define CAMERA_TASK_PRIORITY 5
#define TELEGRAM_TASK_PRIORITY 4
#define LLM_TASK_PRIORITY 3
#define CONTROL_TASK_PRIORITY 6

// Buffer sizes
#define HTTP_BUFFER_SIZE 8192
#define IMAGE_BUFFER_SIZE 32768
#define JSON_BUFFER_SIZE 4096

// Vision Analysis Prompts
#define VISION_PROMPT_PREFIX "You are an autonomous robot car with camera vision. Analyze this image and provide:\n" \
                             "1. Description of what you see\n" \
                             "2. Any obstacles or interesting objects\n" \
                             "3. Suggested movement (forward/backward/left/right/stop)\n" \
                             "4. Confidence level (low/medium/high)\n" \
                             "Be concise, max 100 words."

#endif // CONFIG_H