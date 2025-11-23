---
description: Flash firmware to an ESP32 device
argument-hint: "[project-name] [port]"
allowed-tools: Bash(make:*), Bash(ls:*)
---

# Flash Firmware to ESP32 Device

Flash compiled firmware to an ESP32 device.

## Parse Arguments

$ARGUMENTS may contain:
- Project name (first word)
- Serial port (second word, or detect automatically)

## Available Projects

Same as /build command:
- `robocar-main`, `robocar-cam`, `esp32-webserver`, `esp32-audio`, `llm-telegram`

## Port Detection

If no port specified:
1. Check for common ports:
   ```bash
   ls /dev/cu.usbserial-* /dev/ttyUSB* 2>/dev/null || echo "No devices found"
   ```
2. Use default: /dev/cu.usbserial-0001 (macOS) or /dev/ttyUSB0 (Linux)

## Flash Commands

Run the appropriate flash command with PORT variable:

- `robocar-main`: `make robocar-flash-main PORT=/dev/xxx`
- `robocar-cam`: `make robocar-flash-cam PORT=/dev/xxx`
- `esp32-webserver`: `make esp32-webserver-flash PORT=/dev/xxx`
- `esp32-audio`: `make esp32-audio-flash PORT=/dev/xxx`
- `llm-telegram`: `make llm-telegram-flash PORT=/dev/xxx`

## ESP32-CAM Note

For ESP32-CAM projects (robocar-cam, esp32-webserver, esp32-audio, llm-telegram):
- Remind the user: "Connect GPIO0 to GND before flashing"
- After successful flash: "Disconnect GPIO0 from GND and reset the device"

## Error Handling

If flashing fails:
1. Check if port exists
2. Suggest checking USB connection
3. For permission errors, suggest: `sudo chmod 666 /dev/xxx`
