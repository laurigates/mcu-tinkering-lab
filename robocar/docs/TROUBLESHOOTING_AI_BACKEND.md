# AI Backend Troubleshooting Guide

## Improved Diagnostic Logging

The ESP32-CAM firmware now includes comprehensive diagnostic logging to help identify why the AI backend might not be available.

## Understanding the Log Messages

### During Startup (AI Backend Initialization)

The system will now provide detailed information about:

1. **WiFi Connection Status**
   ```
   I (xxxx) esp32-cam-robocar: WiFi connected successfully, attempting to initialize AI backend...
   ```
   OR
   ```
   W (xxxx) esp32-cam-robocar: WiFi connection failed - skipping AI backend initialization
   ```

2. **Backend Configuration**
   ```
   I (xxxx) esp32-cam-robocar: Configuration: Ollama backend selected
   I (xxxx) esp32-cam-robocar: Ollama fallback URL: http://192.168.0.115:11434/api/generate
   I (xxxx) esp32-cam-robocar: Ollama model: gemma3:4b
   I (xxxx) esp32-cam-robocar: Service discovery enabled, will attempt mDNS lookup for: _ollama._tcp.local
   ```

3. **Service Discovery Process**
   ```
   I (xxxx) ollama_backend: Initializing service discovery for SRV record: _ollama._tcp.local
   I (xxxx) ollama_discovery: Service discovery initialized successfully
   I (xxxx) ollama_backend: Searching for Ollama services...
   I (xxxx) ollama_backend: Found 1 Ollama service(s)
   I (xxxx) ollama_backend: Service 0: ollama.local:11434 (192.168.0.115) priority=0 weight=0
   I (xxxx) ollama_backend: Selected best service: ollama.local:11434
   I (xxxx) ollama_backend: Using discovered Ollama service: http://192.168.0.115:11434/api/generate
   ```

### During Runtime (Image Analysis)

When the AI backend is not available, you'll see:

```
W (xxxx) esp32-cam-robocar: AI backend not available, skipping analysis
W (xxxx) esp32-cam-robocar: Diagnostic: WiFi connected: YES
W (xxxx) esp32-cam-robocar: Diagnostic: AI backend was available during init but failed
W (xxxx) esp32-cam-robocar: Diagnostic: Check previous logs for backend initialization errors
W (xxxx) esp32-cam-robocar: Diagnostic: Ollama backend configured - check service discovery and fallback URL
W (xxxx) esp32-cam-robocar: Diagnostic: Ollama URL should be: http://192.168.0.115:11434/api/generate
W (xxxx) esp32-cam-robocar: Diagnostic: Ollama model: gemma3:4b
```

## Common Issues and Solutions

### 1. WiFi Not Connected

**Symptoms:**
```
W (xxxx) esp32-cam-robocar: WiFi connection failed - skipping AI backend initialization
W (xxxx) esp32-cam-robocar: Diagnostic: WiFi connected: NO
W (xxxx) esp32-cam-robocar: Diagnostic: WiFi not connected - this is likely the root cause
```

**Solutions:**
- Check WiFi credentials in `credentials.h`
- Verify WiFi network is available and in range
- Check WiFi signal strength
- Ensure correct WiFi password

### 2. No AI Backend Configured

**Symptoms:**
```
E (xxxx) esp32-cam-robocar: ERROR: No AI backend defined in config.h!
E (xxxx) esp32-cam-robocar: CRITICAL: ai_backend_get_current() returned NULL!
```

**Solutions:**
- Ensure `#define CONFIG_AI_BACKEND_OLLAMA` is uncommented in `config.h`
- Verify the build system includes the correct backend files

### 3. Service Discovery Failed

**Symptoms:**
```
W (xxxx) ollama_backend: No Ollama services discovered: ESP_ERR_NOT_FOUND
W (xxxx) ollama_backend: Service discovery troubleshooting:
W (xxxx) ollama_backend:   1. Check if Ollama mDNS service is published
W (xxxx) ollama_backend:   2. Verify both devices are on the same network
```

**Solutions:**
- Publish mDNS service on your computer:
  ```bash
  dns-sd -R "Ollama AI" _ollama._tcp . 11434 path=/api/generate &
  ```
- Test mDNS is working:
  ```bash
  dns-sd -B _ollama._tcp
  ```
- Ensure both devices are on the same WiFi network
- Check firewall settings

### 4. Backend Initialization Failed

**Symptoms:**
```
E (xxxx) ollama_backend: AI backend initialization FAILED with error: ESP_FAIL
E (xxxx) ollama_backend: Ollama troubleshooting:
E (xxxx) ollama_backend:   1. Check if Ollama is running at http://192.168.0.115:11434/api/generate
```

**Solutions:**
- Verify Ollama is running: `curl http://192.168.0.115:11434/api/generate`
- Check network connectivity between ESP32-CAM and Ollama server
- Test if the configured model is available in Ollama
- Verify firewall allows HTTP traffic on port 11434

### 5. mDNS Initialization Failed

**Symptoms:**
```
E (xxxx) ollama_backend: Failed to initialize service discovery: ESP_ERR_NO_MEM
E (xxxx) ollama_backend: mDNS initialization failed - check network configuration
```

**Solutions:**
- Check available memory on ESP32-CAM
- Ensure WiFi is fully connected before attempting mDNS
- Try reducing other memory usage in the application

## Testing Commands

### Test mDNS Service Publication (macOS)
```bash
# Publish service
dns-sd -R "Ollama AI" _ollama._tcp . 11434 path=/api/generate &

# Browse for services
dns-sd -B _ollama._tcp

# Test hostname resolution
ping ollama.local
```

### Test Ollama Server Connectivity
```bash
# Test if Ollama is responding
curl http://192.168.0.115:11434/api/generate

# List available models
curl http://192.168.0.115:11434/api/tags

# Check if specific model is available
curl http://192.168.0.115:11434/api/show -d '{"name":"gemma3:4b"}'
```

### Monitor ESP32-CAM Logs
```bash
cd esp32-cam-idf
make monitor
```

Look for the detailed diagnostic messages to identify the exact failure point.

## Log Level Configuration

To see more detailed logs, you can adjust the log level in the ESP-IDF menuconfig:

```bash
idf.py menuconfig
# Navigate to: Component config → Log output → Default log verbosity
# Set to "Debug" for maximum detail
```

This will show additional debug messages from the service discovery and network components.