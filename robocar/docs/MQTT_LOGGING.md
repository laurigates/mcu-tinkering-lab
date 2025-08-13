# MQTT Logging System

This document describes the comprehensive MQTT logging system implemented in the ESP32 AI-Powered Robot Car project. The system provides real-time telemetry, debugging capabilities, and seamless integration with development workflows.

## Overview

The MQTT logging system enables the ESP32-CAM to transmit structured log messages, system metrics, and status information to a central MQTT broker. This provides developers with real-time insights into the robot's operation, debugging information, and telemetry data.

## Architecture

### Components

1. **ESP32-CAM MQTT Logger** (`mqtt_logger.c/h`)
   - JSON-formatted message generation
   - Offline message buffering
   - Automatic reconnection handling
   - Configurable log levels and QoS

2. **Mosquitto MQTT Broker** (Computer-side)
   - Local broker for development and testing
   - Automatic startup via Makefile targets
   - mDNS service advertising for ESP32 discovery

3. **Service Discovery** (mDNS)
   - Automatic broker discovery by ESP32
   - Cross-platform support (macOS/Linux)
   - `_mqtt._tcp` service type

## Configuration

### ESP32-CAM Configuration

The MQTT logger is configured in `esp32-cam-idf/main/config.h`:

```c
#define MQTT_BROKER_URI               "mqtt://192.168.0.100:1883"
#define MQTT_CLIENT_ID                "robocar_esp32cam"
#define MQTT_LOG_TOPIC                "robocar/logs"
#define MQTT_STATUS_TOPIC             "robocar/status"
#define MQTT_COMMAND_TOPIC            "robocar/commands"
#define MQTT_QOS_LEVEL                1
#define MQTT_KEEPALIVE_INTERVAL       60
#define MQTT_BUFFER_SIZE              4096
```

### Logger Initialization

The logger is initialized in `main.c` with the following configuration:

```c
mqtt_logger_config_t mqtt_config = {
    .broker_uri = MQTT_BROKER_URI,
    .client_id = MQTT_CLIENT_ID,
    .log_topic = MQTT_LOG_TOPIC,
    .status_topic = MQTT_STATUS_TOPIC,
    .command_topic = MQTT_COMMAND_TOPIC,
    .buffer_size = MQTT_BUFFER_SIZE,
    .keepalive_interval = MQTT_KEEPALIVE_INTERVAL,
    .qos_level = MQTT_QOS_LEVEL,
    .min_level = ESP_LOG_INFO,
    .retain_status = true
};
```

## Message Format

### Log Messages

Log messages are transmitted as JSON objects with the following structure:

```json
{
    "timestamp": 1693123456789,
    "level": "INFO",
    "tag": "camera",
    "message": "Image captured successfully",
    "component": "esp32cam",
    "heap_free": 234567,
    "wifi_rssi": -45
}
```

### Status Messages

Status messages provide system health information:

```json
{
    "timestamp": 1693123456789,
    "device_id": "robocar_esp32cam",
    "uptime": 123456,
    "heap_free": 234567,
    "heap_min": 180000,
    "wifi_rssi": -45,
    "wifi_connected": true,
    "ai_backend": "ollama",
    "camera_status": "active",
    "mqtt_connected": true,
    "messages_sent": 1234,
    "messages_failed": 2
}
```

## Development Stack Integration

### Starting the Complete Stack

The development stack includes both Ollama AI backend and MQTT broker:

```bash
make dev-stack-start     # Start Ollama + MQTT with service discovery
make dev-stack-stop      # Stop complete development stack
```

### Individual Service Management

```bash
# MQTT Broker
make mosquitto-start     # Start MQTT broker with mDNS advertising
make mosquitto-advertise # Advertise MQTT service via mDNS (standalone)
make mosquitto-stop      # Stop MQTT broker and mDNS services

# AI Backend
make ollama-start        # Start Ollama server with mDNS advertising
make ollama-stop         # Stop Ollama server and mDNS services
```

## Service Discovery

### mDNS Configuration

The MQTT broker is advertised via mDNS with the following configuration:

- **Service Name**: "MQTT Broker"
- **Service Type**: `_mqtt._tcp`
- **Port**: 1883
- **Domain**: Local network

### Cross-Platform Support

The system supports both macOS and Linux:

- **macOS**: Uses `dns-sd` for mDNS advertising
- **Linux**: Uses `avahi-publish-service` for mDNS advertising

### ESP32 Discovery

The ESP32-CAM automatically discovers the MQTT broker through mDNS queries, allowing for seamless connectivity without hardcoded IP addresses.

## Message Flow

### Logging Process

1. **Log Generation**: Application code generates log messages using ESP_LOG macros or MQTT_LOG macros
2. **Message Queuing**: Messages are queued in FreeRTOS queue for processing
3. **JSON Formatting**: Log processor converts messages to JSON format with metadata
4. **MQTT Publishing**: Messages are published to the configured MQTT topic
5. **Offline Buffering**: If broker unavailable, messages are buffered for later transmission

### Status Reporting

1. **Periodic Status**: System status is published every 30 seconds
2. **Event-Driven Status**: Status updates sent on significant events (WiFi connect/disconnect, etc.)
3. **Health Monitoring**: Continuous monitoring of system metrics (heap, WiFi, connectivity)

## API Reference

### Core Functions

```c
// Initialize MQTT logger
esp_err_t mqtt_logger_init(const mqtt_logger_config_t* config);

// Send formatted log message
esp_err_t mqtt_logger_logf(esp_log_level_t level, const char* tag, 
                          const char* format, ...);

// Publish device status
esp_err_t mqtt_logger_publish_status(const char* status_json);

// Get logger statistics
esp_err_t mqtt_logger_get_stats(mqtt_logger_stats_t* stats);

// Check connection status
bool mqtt_logger_is_connected(void);
```

### Convenience Macros

```c
// Log level macros
MQTT_LOGE(tag, format, ...)  // Error level
MQTT_LOGW(tag, format, ...)  // Warning level
MQTT_LOGI(tag, format, ...)  // Info level
MQTT_LOGD(tag, format, ...)  // Debug level
MQTT_LOGV(tag, format, ...)  // Verbose level
```

## Monitoring and Debugging

### Subscribing to Logs

Using mosquitto client tools:

```bash
# Subscribe to all log messages
mosquitto_sub -h 192.168.0.100 -t "robocar/logs" -v

# Subscribe to status messages
mosquitto_sub -h 192.168.0.100 -t "robocar/status" -v

# Subscribe to all robocar topics
mosquitto_sub -h 192.168.0.100 -t "robocar/#" -v
```

### Log Analysis

Log messages can be processed using various tools:

- **jq**: Parse and filter JSON log messages
- **MQTT clients**: Real-time log monitoring
- **Log aggregation**: Integration with ELK stack or similar

### Example Log Filtering

```bash
# Filter error messages only
mosquitto_sub -h 192.168.0.100 -t "robocar/logs" | jq 'select(.level == "ERROR")'

# Monitor heap memory usage
mosquitto_sub -h 192.168.0.100 -t "robocar/status" | jq '.heap_free'
```

## Performance Considerations

### Message Rate Limiting

The system implements configurable rate limiting to prevent overwhelming the MQTT broker:

- **Log Level Filtering**: Only messages above configured level are sent
- **Buffer Management**: Fixed-size circular buffer for offline storage
- **QoS Configuration**: Configurable Quality of Service levels

### Network Efficiency

- **JSON Compression**: Minimal JSON structure for efficient transmission
- **Batching**: Multiple log entries can be batched in single message
- **Persistent Sessions**: MQTT persistent sessions for reliable delivery

## Troubleshooting

### Common Issues

1. **MQTT Connection Failed**
   - Check broker is running: `make mosquitto-start`
   - Verify network connectivity
   - Check firewall settings

2. **Service Discovery Not Working**
   - Verify mDNS is functioning on network
   - Check DNS-SD or Avahi installation
   - Review network configuration

3. **Messages Not Appearing**
   - Check log level configuration
   - Verify topic subscriptions
   - Review MQTT client configuration

### Debug Commands

```bash
# Check MQTT broker status
ps aux | grep mosquitto

# Test MQTT connectivity
mosquitto_pub -h 192.168.0.100 -t "test" -m "hello"

# Monitor all MQTT traffic
mosquitto_sub -h 192.168.0.100 -t "#" -v
```

## Integration Examples

### Python MQTT Client

```python
import paho.mqtt.client as mqtt
import json

def on_message(client, userdata, message):
    try:
        log_data = json.loads(message.payload.decode())
        print(f"[{log_data['level']}] {log_data['tag']}: {log_data['message']}")
    except json.JSONDecodeError:
        print(f"Raw message: {message.payload.decode()}")

client = mqtt.Client()
client.on_message = on_message
client.connect("192.168.0.100", 1883, 60)
client.subscribe("robocar/logs")
client.loop_forever()
```

### Node.js MQTT Client

```javascript
const mqtt = require('mqtt');
const client = mqtt.connect('mqtt://192.168.0.100:1883');

client.on('connect', () => {
    client.subscribe('robocar/logs');
    client.subscribe('robocar/status');
});

client.on('message', (topic, message) => {
    try {
        const data = JSON.parse(message.toString());
        console.log(`[${data.level}] ${data.tag}: ${data.message}`);
    } catch (e) {
        console.log(`Raw: ${message.toString()}`);
    }
});
```

## Future Enhancements

- **Log Persistence**: Database storage for historical analysis
- **Alerting**: Automated alerts based on log patterns
- **Metrics Dashboard**: Real-time visualization of system metrics
- **Log Rotation**: Automatic log cleanup and archiving
- **Security**: TLS encryption and authentication
- **Compression**: Message compression for bandwidth optimization

## Security Considerations

- **Network Security**: Consider TLS encryption for production deployments
- **Authentication**: Implement MQTT username/password authentication
- **Access Control**: Restrict topic access based on client identity
- **Data Privacy**: Ensure sensitive information is not logged